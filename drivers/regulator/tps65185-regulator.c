/*
 * Minimal regulator driver for the TI TPS65185 E Ink panel PMIC.
 *
 * GPIO sequencing and ED078KH1 timing values come from the GPL
 * Netronix/Tolino Epos driver. OTP programming, VCOM calibration writes and
 * interrupt handling are deliberately omitted during E70Q10 bring-up.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

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
	bool display_on;
};

static int tps65185_wait_pwrgood(struct tps65185 *tps, bool on)
{
	int timeout;

	if (!tps->pwrgood) {
		msleep(on ? 30 : 50);
		return 0;
	}
	for (timeout = 0; timeout < 500; timeout += 5) {
		if (!!gpiod_get_value_cansleep(tps->pwrgood) == on)
			return 0;
		msleep(5);
	}
	dev_err(tps->dev, "panel power-good did not become %s\n",
		on ? "active" : "inactive");
	return -ETIMEDOUT;
}

static int tps65185_display_enable(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&tps->lock);
	if (tps->display_on) {
		ret = 0;
		goto out;
	}
	gpiod_set_value_cansleep(tps->vin, 1);
	gpiod_set_value_cansleep(tps->wakeup, 1);
	msleep(2);

	ret = regmap_write(tps->regmap, TPS65185_REG_ENABLE,
			   TPS65185_ENABLE_RAILS);
	if (ret)
		goto out;
	ret = regmap_write(tps->regmap, TPS65185_REG_INT_EN1, 0x7f);
	if (ret)
		goto out;
	ret = regmap_write(tps->regmap, TPS65185_REG_INT_EN2, 0xff);
	if (ret)
		goto out;
	ret = regmap_write(tps->regmap, TPS65185_REG_UPSEQ0, 0xe1);
	if (ret)
		goto out;
	ret = regmap_write(tps->regmap, TPS65185_REG_UPSEQ1, 0x00);
	if (ret)
		goto out;
	ret = regmap_write(tps->regmap, TPS65185_REG_DWNSEQ0, 0x1e);
	if (ret)
		goto out;

	msleep(40);
	gpiod_set_value_cansleep(tps->powerup, 1);
	ret = tps65185_wait_pwrgood(tps, true);
	if (ret) {
		gpiod_set_value_cansleep(tps->powerup, 0);
		goto out;
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
	if (tps->display_on) {
		gpiod_set_value_cansleep(tps->vcom, 0);
		msleep(2);
		gpiod_set_value_cansleep(tps->powerup, 0);
		ret = tps65185_wait_pwrgood(tps, false);
		tps->display_on = false;
	}
	mutex_unlock(&tps->lock);
	return ret;
}

static int tps65185_display_is_enabled(struct regulator_dev *rdev)
{
	return ((struct tps65185 *)rdev_get_drvdata(rdev))->display_on;
}

static int tps65185_vcom_enable(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);

	if (!tps->display_on)
		return -EIO;
	gpiod_set_value_cansleep(tps->vcom, 1);
	msleep(5);
	return 0;
}

static int tps65185_vcom_disable(struct regulator_dev *rdev)
{
	struct tps65185 *tps = rdev_get_drvdata(rdev);

	gpiod_set_value_cansleep(tps->vcom, 0);
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

static int tps65185_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct tps65185 *tps;
	unsigned int revid, vcom1, vcom2;
	int i, ret;

	tps = devm_kzalloc(&client->dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;
	tps->dev = &client->dev;
	mutex_init(&tps->lock);
	i2c_set_clientdata(client, tps);

	tps->vin = devm_gpiod_get(&client->dev, "vin", GPIOD_OUT_HIGH);
	if (IS_ERR(tps->vin))
		return PTR_ERR(tps->vin);
	msleep(5);
	tps->wakeup = devm_gpiod_get(&client->dev, "wakeup", GPIOD_OUT_HIGH);
	if (IS_ERR(tps->wakeup)) {
		dev_err(&client->dev, "failed to acquire WAKEUP GPIO: %ld\n",
			PTR_ERR(tps->wakeup));
		return PTR_ERR(tps->wakeup);
	}
	/* Datasheet/vendor sequence requires settling before the first I2C read. */
	msleep(2);
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

static void tps65185_shutdown(struct i2c_client *client)
{
	struct tps65185 *tps = i2c_get_clientdata(client);

	gpiod_set_value_cansleep(tps->vcom, 0);
	gpiod_set_value_cansleep(tps->powerup, 0);
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
	},
	.probe = tps65185_probe,
	.shutdown = tps65185_shutdown,
	.id_table = tps65185_id,
};
module_i2c_driver(tps65185_driver);

MODULE_AUTHOR("E70Q10 porting project");
MODULE_DESCRIPTION("TI TPS65185 E Ink panel regulator driver");
MODULE_LICENSE("GPL v2");
