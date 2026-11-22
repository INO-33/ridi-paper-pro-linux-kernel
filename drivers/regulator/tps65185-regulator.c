/*
 * Minimal regulator driver for the TI TPS65185 E Ink panel PMIC.
 *
 * GPIO sequencing and ED078KH1 timing values come from the GPL
 * Netronix/Tolino Epos driver. OTP programming and VCOM calibration writes
 * are deliberately omitted. Runtime rail retention and thermal
 * sampling follow the stock Netronix policy.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/tps65185.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define TPS65185_REG_TMST_VALUE	0x00
#define TPS65185_REG_TMST1	0x0d
#define TPS65185_READ_THERM	BIT(7)
#define TPS65185_CONV_END	BIT(5)
#define TPS65185_RAIL_HOLD_MS	500

#define TPS65185_REG_ENABLE	0x01
#define TPS65185_REG_VCOM1	0x03
#define TPS65185_REG_VCOM2	0x04
#define TPS65185_REG_INT_EN1	0x05
#define TPS65185_REG_INT_EN2	0x06
#define TPS65185_REG_UPSEQ0	0x09
#define TPS65185_REG_UPSEQ1	0x0a
#define TPS65185_REG_DWNSEQ0	0x0b
#define TPS65185_REG_REVID	0x10

#define TPS65185_ENABLE_RAILS	0x3f
#define TPS65185_VCOM8		BIT(0)

enum tps65185_regulator_id {
	TPS65185_DISPLAY,
	TPS65185_VCOM,
	TPS65185_V3P3,
	TPS65185_NUM_REGULATORS,
};

struct tps65185 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *vin;
	struct gpio_desc *wakeup;
	struct gpio_desc *powerup;
	struct gpio_desc *pwrgood;
	struct gpio_desc *vcom;
	struct mutex lock;
	/* display_on is the consumer vote; rails_on is the physical state. */
	bool display_on;
	bool rails_on;
	bool powerdown_failed;
	bool suspended;
	bool stopping;
	bool pwrgood_irq;
	wait_queue_head_t pwrgood_wait;
	struct delayed_work powerdown_work;
};

static irqreturn_t tps65185_pwrgood_irq(int irq, void *data)
{
	struct tps65185 *tps = data;

	wake_up_all(&tps->pwrgood_wait);
	return IRQ_HANDLED;
}

static bool tps65185_pwrgood_ready(struct tps65185 *tps, bool on)
{
	int value = gpiod_get_value_cansleep(tps->pwrgood);

	/* Wake on GPIO read errors too; the caller propagates them. */
	return value < 0 || !!value == on;
}

static int tps65185_wait_pwrgood(struct tps65185 *tps, bool on)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(500);
	int value;

	if (!tps->pwrgood) {
		msleep(on ? 30 : 50);
		return 0;
	}
	if (tps->pwrgood_irq) {
		wait_event_timeout(tps->pwrgood_wait,
			tps65185_pwrgood_ready(tps, on), msecs_to_jiffies(500));
	} else {
		/* Bounded polling fallback if the board has no usable IRQ. */
		while (!tps65185_pwrgood_ready(tps, on) &&
		       time_before(jiffies, deadline))
			usleep_range(1000, 2000);
	}
	value = gpiod_get_value_cansleep(tps->pwrgood);
	if (value < 0)
		return value;
	if (!!value == on)
		return 0;
	dev_err(tps->dev, "panel power-good did not become %s\n",
		on ? "active" : "inactive");
	return -ETIMEDOUT;
}

/* tps->lock serializes the worker against enable, temperature and PM. */
static int tps65185_rails_off_locked(struct tps65185 *tps)
{
	int ret;

	if (!tps->rails_on && !tps->powerdown_failed)
		return 0;
	gpiod_set_value_cansleep(tps->vcom, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(tps->powerup, 0);
	ret = tps65185_wait_pwrgood(tps, false);
	/* PWRUP is low on timeout; require discharge before reuse. */
	tps->rails_on = false;
	tps->powerdown_failed = !!ret;
	return ret;
}

static void tps65185_powerdown_work(struct work_struct *work)
{
	struct tps65185 *tps = container_of(to_delayed_work(work),
					  struct tps65185, powerdown_work);
	int ret = 0;

	mutex_lock(&tps->lock);
	if (!tps->display_on)
		ret = tps65185_rails_off_locked(tps);
	mutex_unlock(&tps->lock);
	if (ret)
		dev_warn(tps->dev, "deferred panel powerdown failed: %d\n",
			 ret);
}

static int tps65185_display_enable(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);
	int ret = 0;

	/* Never cancel_sync while holding the lock needed by the worker. */
	cancel_delayed_work_sync(&tps->powerdown_work);
	mutex_lock(&tps->lock);
	if (tps->suspended || tps->stopping) {
		ret = -ESHUTDOWN;
		goto out;
	}
	if (tps->display_on)
		goto out;
	if (tps->powerdown_failed) {
		ret = tps65185_wait_pwrgood(tps, false);
		if (ret)
			goto out;
		tps->powerdown_failed = false;
	}
	if (!tps->rails_on) {
		/* VIN/WAKEUP and configuration survive normal standby. */
		gpiod_set_value_cansleep(tps->powerup, 1);
		ret = tps65185_wait_pwrgood(tps, true);
		if (ret) {
			gpiod_set_value_cansleep(tps->powerup, 0);
			tps->powerdown_failed = true;
			goto out;
		}
		tps->rails_on = true;
	}
	tps->display_on = true;
out:
	mutex_unlock(&tps->lock);
	return ret;
}

static int tps65185_display_disable(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);
	int ret = 0;

	mutex_lock(&tps->lock);
	gpiod_set_value_cansleep(tps->vcom, 0);
	tps->display_on = false;
	if (tps->suspended || tps->stopping)
		ret = tps65185_rails_off_locked(tps);
	else if (tps->rails_on)
		mod_delayed_work(system_wq, &tps->powerdown_work,
				 msecs_to_jiffies(TPS65185_RAIL_HOLD_MS));
	mutex_unlock(&tps->lock);
	return ret;
}

/* Used after EPDC has drained and dropped its DISPLAY vote. */
int tps65185_flush_powerdown(struct regulator *display)
{
	struct tps65185 *tps = regulator_get_drvdata(display);
	int ret;

	cancel_delayed_work_sync(&tps->powerdown_work);
	mutex_lock(&tps->lock);
	ret = tps->display_on ? -EBUSY : tps65185_rails_off_locked(tps);
	mutex_unlock(&tps->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps65185_flush_powerdown);

int tps65185_read_temperature(struct regulator *display, int *temperature)
{
	struct tps65185 *tps = regulator_get_drvdata(display);
	unsigned long deadline = jiffies + msecs_to_jiffies(500);
	unsigned int value;
	int ret;

	mutex_lock(&tps->lock);
	if (tps->suspended || tps->stopping) {
		ret = -ESHUTDOWN;
		goto out;
	}
	/* Write READ_THERM even if a previous conversion timed out. */
	ret = regmap_read(tps->regmap, TPS65185_REG_TMST1, &value);
	if (ret)
		goto out;
	ret = regmap_write(tps->regmap, TPS65185_REG_TMST1,
			   value | TPS65185_READ_THERM);
	if (ret)
		goto out;
	for (;;) {
		ret = regmap_read(tps->regmap, TPS65185_REG_TMST1, &value);
		if (ret)
			goto out;
		if (value & TPS65185_CONV_END)
			break;
		if (time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			goto out;
		}
		usleep_range(1000, 2000);
	}
	ret = regmap_read(tps->regmap, TPS65185_REG_TMST_VALUE, &value);
	if (!ret)
		*temperature = (s8)value;
out:
	mutex_unlock(&tps->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tps65185_read_temperature);

static int tps65185_display_is_enabled(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);

	return READ_ONCE(tps->display_on);
}

static int tps65185_vcom_enable(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);
	int ret = 0;

	mutex_lock(&tps->lock);
	if (!tps->display_on || tps->suspended || tps->stopping) {
		ret = -EIO;
		goto out;
	}
	/* Preserve the stock settling intervals without HZ=100 rounding. */
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(tps->vcom, 1);
	usleep_range(5000, 6000);
out:
	mutex_unlock(&tps->lock);
	return ret;
}

static int tps65185_vcom_disable(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);

	mutex_lock(&tps->lock);
	gpiod_set_value_cansleep(tps->vcom, 0);
	mutex_unlock(&tps->lock);
	return 0;
}

static int tps65185_vcom_is_enabled(struct regulator_dev *rdev)
{
	return gpiod_get_value_cansleep(
		((struct tps65185 *)rdev_get_drvdata(rdev))->vcom);
}

static int tps65185_vcom_get_voltage(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);
	unsigned int low, high;
	int ret;

	ret = regmap_read(tps->regmap, TPS65185_REG_VCOM1, &low);
	if (ret)
		return ret;
	ret = regmap_read(tps->regmap, TPS65185_REG_VCOM2, &high);
	if (ret)
		return ret;
	return ((low & 0xff) | ((high & TPS65185_VCOM8) << 8)) * 10000;
}

static int tps65185_v3p3_enable(struct regulator_dev *rdev)
{
	/* GPIO4_3 is TP_3V3_ON on E70Q10, so it must not be toggled here. */
	return 0;
}

static int tps65185_v3p3_is_enabled(struct regulator_dev *rdev)
{
	return 1;
}

static const struct regulator_ops tps65185_display_ops = {
	.enable = tps65185_display_enable,
	.disable = tps65185_display_disable,
	.is_enabled = tps65185_display_is_enabled,
};

static const struct regulator_ops tps65185_vcom_ops = {
	.enable = tps65185_vcom_enable,
	.disable = tps65185_vcom_disable,
	.is_enabled = tps65185_vcom_is_enabled,
	.get_voltage = tps65185_vcom_get_voltage,
};

static const struct regulator_ops tps65185_v3p3_ops = {
	.enable = tps65185_v3p3_enable,
	.disable = tps65185_v3p3_enable,
	.is_enabled = tps65185_v3p3_is_enabled,
};

static const struct regulator_desc tps65185_regulators[] = {
	[TPS65185_DISPLAY] = {
		.name = "DISPLAY", .of_match = "DISPLAY",
		.regulators_node = "regulators", .id = TPS65185_DISPLAY,
		.ops = &tps65185_display_ops, .type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[TPS65185_VCOM] = {
		.name = "VCOM", .of_match = "VCOM",
		.regulators_node = "regulators", .id = TPS65185_VCOM,
		.ops = &tps65185_vcom_ops, .type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	[TPS65185_V3P3] = {
		.name = "V3P3", .of_match = "V3P3",
		.regulators_node = "regulators", .id = TPS65185_V3P3,
		.ops = &tps65185_v3p3_ops, .type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static const struct regmap_config tps65185_regmap_config = {
	.reg_bits = 8, .val_bits = 8, .max_register = TPS65185_REG_REVID,
};

static int tps65185_configure(struct tps65185 *tps)
{
	static const struct reg_sequence sequence[] = {
		{ TPS65185_REG_ENABLE, TPS65185_ENABLE_RAILS },
		{ TPS65185_REG_INT_EN1, 0x7f },
		{ TPS65185_REG_INT_EN2, 0xff },
		{ TPS65185_REG_UPSEQ0, 0xe1 },
		{ TPS65185_REG_UPSEQ1, 0x00 },
		{ TPS65185_REG_DWNSEQ0, 0x1e },
	};

	return regmap_multi_reg_write(tps->regmap, sequence,
				     ARRAY_SIZE(sequence));
}

static int tps65185_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct tps65185 *tps;
	unsigned int revid, vcom1, vcom2;
	int i, irq, ret;

	tps = devm_kzalloc(&client->dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;
	tps->dev = &client->dev;
	mutex_init(&tps->lock);
	init_waitqueue_head(&tps->pwrgood_wait);
	INIT_DELAYED_WORK(&tps->powerdown_work, tps65185_powerdown_work);
	i2c_set_clientdata(client, tps);

	tps->vin = devm_gpiod_get(&client->dev, "vin", GPIOD_OUT_HIGH);
	if (IS_ERR(tps->vin))
		return PTR_ERR(tps->vin);
	usleep_range(5000, 6000);
	tps->wakeup = devm_gpiod_get(&client->dev, "wakeup", GPIOD_OUT_HIGH);
	if (IS_ERR(tps->wakeup)) {
		dev_err(&client->dev, "failed to acquire WAKEUP GPIO: %ld\n",
			PTR_ERR(tps->wakeup));
		return PTR_ERR(tps->wakeup);
	}
	/* Datasheet/vendor sequence requires settling before the first I2C read. */
	usleep_range(2000, 3000);
	tps->powerup = devm_gpiod_get(&client->dev, "powerup", GPIOD_OUT_LOW);
	if (IS_ERR(tps->powerup))
		return PTR_ERR(tps->powerup);
	tps->vcom = devm_gpiod_get(&client->dev, "vcom", GPIOD_OUT_LOW);
	if (IS_ERR(tps->vcom))
		return PTR_ERR(tps->vcom);
	tps->pwrgood = devm_gpiod_get_optional(&client->dev, "pwrgood",
					       GPIOD_IN);
	if (IS_ERR(tps->pwrgood))
		return PTR_ERR(tps->pwrgood);

	tps->regmap = devm_regmap_init_i2c(client, &tps65185_regmap_config);
	if (IS_ERR(tps->regmap))
		return PTR_ERR(tps->regmap);
	/* Stock retries detection while VIN/WAKEUP and the internal I2C block rise. */
	for (i = 0; i < 30; i++) {
		ret = regmap_read(tps->regmap, TPS65185_REG_REVID, &revid);
		if (!ret)
			break;
		msleep(10);
	}
	if (ret) {
		dev_err(&client->dev, "REVID read failed after wake-up: %d\n", ret);
		return ret;
	}
	ret = regmap_read(tps->regmap, TPS65185_REG_VCOM1, &vcom1);
	if (ret)
		return ret;
	ret = regmap_read(tps->regmap, TPS65185_REG_VCOM2, &vcom2);
	if (ret)
		return ret;

	ret = tps65185_configure(tps);
	if (ret)
		return ret;
	/* Once per VIN rise, not once per frame/standby exit. */
	usleep_range(40000, 41000);
	if (tps->pwrgood) {
		irq = gpiod_to_irq(tps->pwrgood);
		if (irq == -EPROBE_DEFER)
			return irq;
		ret = irq < 0 ? irq : devm_request_threaded_irq(&client->dev,
			irq, NULL, tps65185_pwrgood_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
			IRQF_ONESHOT,
			"tps65185-pwrgood", tps);
		if (!ret)
			tps->pwrgood_irq = true;
		else
			dev_warn(tps->dev,
				 "PWRGOOD IRQ unavailable (%d), polling\n",
				 ret);
	}

	config.dev = &client->dev;
	config.driver_data = tps;
	config.regmap = tps->regmap;
	config.of_node = client->dev.of_node;
	for (i = 0; i < TPS65185_NUM_REGULATORS; i++) {
		rdev = devm_regulator_register(&client->dev,
					       &tps65185_regulators[i], &config);
		if (IS_ERR(rdev))
			return PTR_ERR(rdev);
	}

	dev_info(&client->dev, "TPS65185 rev 0x%02x, VCOM magnitude %u mV\n",
		 revid, (unsigned int)(((vcom1 & 0xff) |
		 ((vcom2 & TPS65185_VCOM8) << 8)) * 10));
	return 0;
}

static int tps65185_suspend(struct device *dev)
{
	struct tps65185 *tps = dev_get_drvdata(dev);
	int ret;

	cancel_delayed_work_sync(&tps->powerdown_work);
	mutex_lock(&tps->lock);
	/* EPDC must have drained and released DISPLAY before PMIC suspend. */
	if (tps->display_on) {
		ret = -EBUSY;
	} else {
		ret = tps65185_rails_off_locked(tps);
		if (!ret)
			tps->suspended = true;
	}
	mutex_unlock(&tps->lock);
	return ret;
}

static int tps65185_resume(struct device *dev)
{
	struct tps65185 *tps = dev_get_drvdata(dev);

	mutex_lock(&tps->lock);
	tps->suspended = false;
	mutex_unlock(&tps->lock);
	return 0;
}

static const struct dev_pm_ops tps65185_pm_ops = {
	/* Late suspend follows the EPDC's normal suspend/drain callback. */
	SET_LATE_SYSTEM_SLEEP_PM_OPS(tps65185_suspend, tps65185_resume)
};

static void tps65185_shutdown(struct i2c_client *client)
{
	struct tps65185 *tps = i2c_get_clientdata(client);

	mutex_lock(&tps->lock);
	tps->stopping = true;
	mutex_unlock(&tps->lock);
	cancel_delayed_work_sync(&tps->powerdown_work);
	mutex_lock(&tps->lock);
	tps->display_on = false;
	tps65185_rails_off_locked(tps);
	gpiod_set_value_cansleep(tps->vcom, 0);
	gpiod_set_value_cansleep(tps->powerup, 0);
	mutex_unlock(&tps->lock);
}

static int tps65185_remove(struct i2c_client *client)
{
	tps65185_shutdown(client);
	return 0;
}

static const struct of_device_id tps65185_of_match[] = {
	{ .compatible = "ti,tps65185" }, { }
};
MODULE_DEVICE_TABLE(of, tps65185_of_match);

static const struct i2c_device_id tps65185_id[] = {
	{ "tps65185", 0 }, { }
};
MODULE_DEVICE_TABLE(i2c, tps65185_id);

static struct i2c_driver tps65185_driver = {
	.driver = {
		.name = "tps65185-regulator",
		.of_match_table = tps65185_of_match,
		.pm = &tps65185_pm_ops,
	},
	.probe = tps65185_probe,
	.shutdown = tps65185_shutdown,
	.remove = tps65185_remove,
	.id_table = tps65185_id,
};
module_i2c_driver(tps65185_driver);

MODULE_AUTHOR("E70Q10 porting project");
MODULE_DESCRIPTION("TI TPS65185 E Ink panel regulator driver");
MODULE_LICENSE("GPL v2");
