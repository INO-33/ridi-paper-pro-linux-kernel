/*
 * MFD core driver for Ricoh RN5T618 PMIC
 *
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
 * Copyright (C) 2016 Toradex AG
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rn5t618.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>

static const struct mfd_cell rn5t618_cells[] = {
	{ .name = "rn5t618-regulator" },
	{ .name = "rn5t618-wdt" },
};

/*
 * RC5T619 has the RN5T618-compatible charger and fuel gauge block.
 * Keep the power child variant-specific so RN5T567 users do not probe
 * registers which do not exist on that part.
 */
static const struct mfd_cell rc5t619_cells[] = {
	{ .name = "rn5t618-regulator" },
	{ .name = "rn5t618-wdt" },
	{ .name = "rn5t618-power" },
	{ .name = "ricoh619-rtc" },
};

static bool rn5t618_volatile_reg(struct device *dev, unsigned int reg)
{
	/* Bank1 profile/test registers must reflect the actual PMIC, including
	 * write protection and learned capacity. Never serve them from cache.
	 */
	if (reg >= 0x100)
		return true;
	switch (reg) {
	/* These reflect live power history / self-clearing command state. */
	case RN5T618_PONHIS:
	case RN5T618_POFFHIS:
	case RN5T618_SLPCNT:
	case RN5T618_REPCNT:
	case RN5T618_WATCHDOGCNT:
	case RN5T618_DCIRQ:
	case RN5T618_ILIMDATAH ... RN5T618_AIN0DATAL:
	case RN5T618_IR_ADC1 ... RN5T618_IR_ADC3:
	case RN5T618_IR_GPR:
	case RN5T618_IR_GPF:
	case RN5T618_MON_IOIN:
	case RN5T618_INTMON:
	case RN5T618_CHGSTATE:
	case RN5T618_CHGCTRL_IRR ... RN5T618_CHGERR_MONI:
	case 0xd2 ... 0xd3: /* RC5T619 charger external-interface IRQ/status */
	case 0xa0 ... 0xaf: /* RC5T619 RTC time and status registers */
	case RN5T618_CONTROL ... RN5T618_CC_AVEREG0:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rn5t618_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.volatile_reg	= rn5t618_volatile_reg,
	.max_register	= RN5T618_MAX_REG,
	.cache_type	= REGCACHE_RBTREE,
};

/* Both banks share ONE regmap lock and selector cache. Every bank0 hardware
 * access also selects bank0, so RTC/regulator/IRQ I/O cannot hit bank1 after
 * a battery transaction. Do not add a second raw-I2C bank-switching path.
 */
static const struct regmap_range_cfg rc5t619_banks[] = {
	{
		.name = "rc5t619-banks",
		.range_min = 0,
		.range_max = RC5T619_BANK1(RN5T618_MAX_REG),
		.selector_reg = RC5T619_BANKSEL,
		.selector_mask = 1,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config rc5t619_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = rn5t618_volatile_reg,
	.max_register = RC5T619_BANK1(RN5T618_MAX_REG),
	.cache_type = REGCACHE_RBTREE,
	.ranges = rc5t619_banks,
	.num_ranges = ARRAY_SIZE(rc5t619_banks),
};


/* The physical PMIC line is shared: quarantine of the charger group must
 * never mask the ADC low-voltage or RTC wake consumers. Subgroup status is
 * acknowledged by its owning child, under the single bank-aware regmap.
 */
struct rc5t619_irq_data {
	struct rn5t618 *pmic;
	struct device *dev;
	struct irq_domain *domain;
	struct mutex lock;
	struct delayed_work recovery;
	unsigned int enabled;
	int parent_irq;
	bool fault, stopping;
};

static const unsigned int rc5t619_irq_groups[] = { BIT(6), BIT(3), BIT(2) };

static void rc5t619_irq_bus_lock(struct irq_data *d)
{
	struct rc5t619_irq_data *data = irq_data_get_irq_chip_data(d);
	mutex_lock(&data->lock);
}

static void rc5t619_irq_bus_sync_unlock(struct irq_data *d)
{
	struct rc5t619_irq_data *data = irq_data_get_irq_chip_data(d);
	int ret = regmap_write(data->pmic->regmap, RN5T618_INTEN,
			      data->fault || data->stopping ? 0 : data->enabled);
	if (ret) {
		if (!data->fault && !data->stopping) {
			data->fault = true;
			disable_irq_nosync(data->parent_irq);
			queue_delayed_work(system_freezable_wq, &data->recovery, 5 * HZ);
		}
		dev_err_ratelimited(data->dev, "PMIC IRQ mask write failed: %d\n", ret);
	}
	mutex_unlock(&data->lock);
}

static void rc5t619_irq_mask(struct irq_data *d)
{
	struct rc5t619_irq_data *data = irq_data_get_irq_chip_data(d);
	data->enabled &= ~rc5t619_irq_groups[d->hwirq];
}

static void rc5t619_irq_unmask(struct irq_data *d)
{
	struct rc5t619_irq_data *data = irq_data_get_irq_chip_data(d);
	data->enabled |= rc5t619_irq_groups[d->hwirq];
}

static int rc5t619_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct rc5t619_irq_data *data = irq_data_get_irq_chip_data(d);
	return irq_set_irq_wake(data->parent_irq, on);
}

static struct irq_chip rc5t619_irq_chip = {
	.name = "rc5t619",
	.irq_bus_lock = rc5t619_irq_bus_lock,
	.irq_bus_sync_unlock = rc5t619_irq_bus_sync_unlock,
	/* Nested dispatch does not perform the lazy hardware mask used by
	 * normal flow handlers. disable_irq_nosync() must mask this group now,
	 * or the FIFO parent keeps redispatching a disabled child forever.
	 */
	.irq_disable = rc5t619_irq_mask,
	.irq_mask = rc5t619_irq_mask,
	.irq_unmask = rc5t619_irq_unmask,
	.irq_set_wake = rc5t619_irq_set_wake,
};

static int rc5t619_irq_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip_and_handler(irq, &rc5t619_irq_chip, handle_simple_irq);
	irq_set_nested_thread(irq, 1);
	irq_set_noprobe(irq);
	return 0;
}

static const struct irq_domain_ops rc5t619_irq_domain_ops = {
	.map = rc5t619_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

int rc5t619_get_irq(struct rn5t618 *pmic, unsigned int index)
{
	if (!pmic->irq_data || index >= RC5T619_IRQ_COUNT)
		return -ENXIO;
	return irq_find_mapping(pmic->irq_data->domain, index) ?: -ENXIO;
}
EXPORT_SYMBOL_GPL(rc5t619_get_irq);

static void rc5t619_irq_recovery(struct work_struct *work)
{
	struct rc5t619_irq_data *data = container_of(to_delayed_work(work),
		struct rc5t619_irq_data, recovery);
	unsigned int status;
	int ret;

	mutex_lock(&data->lock);
	if (data->stopping) {
		mutex_unlock(&data->lock);
		return;
	}
	ret = regmap_read(data->pmic->regmap, RN5T618_INTMON, &status);
	if (!ret)
		ret = regmap_write(data->pmic->regmap, RN5T618_INTEN, data->enabled);
	if (!ret) {
		data->fault = false;
		enable_irq(data->parent_irq);
	} else {
		queue_delayed_work(system_freezable_wq, &data->recovery, 5 * HZ);
	}
	mutex_unlock(&data->lock);
}

/* INTMON reports latched factors even when their subgroup is masked.
 * Filter them before dispatch so an RTC wake cannot masquerade as USB. */
static int rc5t619_irq_pending(struct rc5t619_irq_data *data,
                              unsigned int *active)
{
	static const unsigned int chg_status[] = { 0xc2, 0xc3, 0xc4, 0xc5, 0xd2 };
	static const unsigned int chg_mask[] = { 0xbe, 0xbf, 0xc0, 0xc1, 0xd1 };
	unsigned int master, status, mask;
	int ret, i;

	*active = 0;
	ret = regmap_read(data->pmic->regmap, RN5T618_INTMON, &master);
	if (ret) return ret;
	master &= data->enabled;
	if (master & BIT(6)) {
		for (i = 0; i < ARRAY_SIZE(chg_status); i++) {
			ret = regmap_read(data->pmic->regmap, chg_status[i], &status);
			if (!ret)
				ret = regmap_read(data->pmic->regmap, chg_mask[i], &mask);
			if (ret) return ret;
			if (status & ~mask) *active |= BIT(6);
		}
	}
	if (master & BIT(3)) {
		ret = regmap_read(data->pmic->regmap, RN5T618_IR_ADC1, &status);
		if (!ret)
			ret = regmap_read(data->pmic->regmap, RN5T618_EN_ADCIR1, &mask);
		if (ret) return ret;
		if (status & mask & BIT(1)) *active |= BIT(3);
	}
	if (master & BIT(2)) {
		ret = regmap_read(data->pmic->regmap, 0xaf, &status);
		if (!ret)
			ret = regmap_read(data->pmic->regmap, 0xae, &mask);
		if (ret) return ret;
		if ((status & BIT(0)) && (mask & BIT(6))) *active |= BIT(2);
	}
	return 0;
}

static irqreturn_t rc5t619_irq_thread(int irq, void *arg)
{
	struct rc5t619_irq_data *data = arg;
	unsigned int pending, enabled;
	int i, ret;

	mutex_lock(&data->lock);
	ret = rc5t619_irq_pending(data, &pending);
	enabled = data->enabled;
	if (ret && !data->stopping) {
		/* Bound an I2C-fault interrupt storm. Recovery runs only while
		 * awake, and does not create an RTC/polling wake source. */
		data->fault = true;
		regmap_write(data->pmic->regmap, RN5T618_INTEN, 0);
		disable_irq_nosync(irq);
		queue_delayed_work(system_freezable_wq, &data->recovery, 5 * HZ);
		dev_err_ratelimited(data->dev, "PMIC IRQ read failed: %d\n", ret);
	}
	mutex_unlock(&data->lock);
	if (ret)
		return IRQ_HANDLED;
	if (!(pending & enabled)) {
		/* A masked, latched charger factor can keep the physical pin low
		 * until the normal-priority debounce worker acknowledges it.
		 * Immediately retriggering this FIFO/50 thread starves that
		 * worker and regmap readers indefinitely. Keep the ONESHOT line
		 * masked briefly, with no mutex held, so the owner can drain it.
		 * Real ADC/RTC factors remain latched for the next dispatch.
		 */
		msleep(20);
		return IRQ_NONE;
	}
	for (i = 0; i < RC5T619_IRQ_COUNT; i++)
		if (pending & enabled & rc5t619_irq_groups[i])
			handle_nested_irq(irq_find_mapping(data->domain, i));
	return IRQ_HANDLED;
}

static void rc5t619_irq_cleanup(void *arg)
{
	struct rc5t619_irq_data *data = arg;
	int i;

	mutex_lock(&data->lock);
	data->stopping = true;
	regmap_write(data->pmic->regmap, RN5T618_INTEN, 0);
	mutex_unlock(&data->lock);
	devm_free_irq(data->dev, data->parent_irq, data);
	cancel_delayed_work_sync(&data->recovery);
	for (i = 0; i < RC5T619_IRQ_COUNT; i++)
		irq_dispose_mapping(irq_find_mapping(data->domain, i));
	irq_domain_remove(data->domain);
	data->pmic->irq_data = NULL;
}

static int rc5t619_irq_init(struct i2c_client *client, struct rn5t618 *pmic)
{
	struct rc5t619_irq_data *data;
	int i, ret;

	if (client->irq <= 0 || !of_property_read_bool(client->dev.of_node,
						    "ricoh,charger-wakeup"))
		return 0;
	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->pmic = pmic;
	data->dev = &client->dev;
	data->parent_irq = client->irq;
	mutex_init(&data->lock);
	INIT_DELAYED_WORK(&data->recovery, rc5t619_irq_recovery);
	ret = regmap_write(pmic->regmap, RN5T618_INTEN, 0);
	if (ret)
		return ret;
	data->domain = irq_domain_add_linear(client->dev.of_node,
		RC5T619_IRQ_COUNT, &rc5t619_irq_domain_ops, data);
	if (!data->domain)
		return -ENOMEM;
	for (i = 0; i < RC5T619_IRQ_COUNT; i++) {
		if (!irq_create_mapping(data->domain, i)) {
			ret = -ENOMEM;
			goto mappings;
		}
	}
	ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
		rc5t619_irq_thread, IRQF_ONESHOT, "rc5t619-pmic", data);
	if (ret)
		goto mappings;
	pmic->irq_data = data;
	return devm_add_action_or_reset(&client->dev, rc5t619_irq_cleanup, data);
mappings:
	for (i = 0; i < RC5T619_IRQ_COUNT; i++)
		if (irq_find_mapping(data->domain, i))
			irq_dispose_mapping(irq_find_mapping(data->domain, i));
	irq_domain_remove(data->domain);
	return ret;
}

static struct rn5t618 *rn5t618_pm_power_off;
static struct notifier_block rn5t618_restart_handler;

/* Stock RC5T619 clears RTC alarm wake and the interrupt mask before off. */
#define RC5T619_RTC_CTRL1 0xae
#define RC5T619_RTC_DALE BIT(6)

static int rn5t618_trigger_poweroff_sequence(bool repower)
{
	struct rn5t618 *pmic = rn5t618_pm_power_off;
	unsigned int value;
	int ret;

	if (!pmic)
		return -ENODEV;

	if (pmic->variant == RC5T619) {
		/* Match stock: always send the whole register, even if the cache
		 * already reports zero. Never issue off if REPWRON remains set.
		 */
		ret = regmap_write(pmic->regmap, RN5T618_REPCNT,
				repower ? RN5T618_REPCNT_REPWRON : 0);
		if (ret)
			return ret;
		ret = regmap_read(pmic->regmap, RN5T618_REPCNT, &value);
		if (ret)
			return ret;
		if (!!(value & RN5T618_REPCNT_REPWRON) != repower) {
			pr_err("rn5t618: REPCNT verification failed: %#x (repower=%d)\n",
				value, repower);
			return -EIO;
		}
		return regmap_write(pmic->regmap, RN5T618_SLPCNT,
				RN5T618_SLPCNT_SWPWROFF);
	}

	ret = regmap_update_bits(pmic->regmap, RN5T618_REPCNT,
				RN5T618_REPCNT_REPWRON,
				repower ? RN5T618_REPCNT_REPWRON : 0);
	if (ret)
		return ret;
	return regmap_write_bits(pmic->regmap, RN5T618_SLPCNT,
				RN5T618_SLPCNT_SWPWROFF, RN5T618_SLPCNT_SWPWROFF);
}

static void rn5t618_power_off(void)
{
	int ret;

	pr_emerg("rn5t618: power-off requested (repower=0)\n");
	if (rn5t618_pm_power_off->variant != RC5T619) {
		ret = rn5t618_trigger_poweroff_sequence(false);
		if (ret)
			pr_err("rn5t618: power-off failed: %d\n", ret);
		return;
	}

	/* Like stock, never return to the reboot syscall if power remains on.
	 * A successful command normally cuts power before the delay expires.
	 */
	for (;;) {
		ret = rn5t618_trigger_poweroff_sequence(false);
		if (ret)
			pr_err("rn5t618: power-off transfer failed: %d\n", ret);
		mdelay(1000);
		pr_err("rn5t618: still powered; retrying power-off\n");
	}
}

static int rn5t618_restart(struct notifier_block *this,
			    unsigned long mode, void *cmd)
{
	int ret = rn5t618_trigger_poweroff_sequence(true);

	if (ret)
		pr_err("rn5t618: restart transfer failed: %d\n", ret);
	/* Allow the lower priority SoC handler to recover if PMIC reset fails. */
	mdelay(1);
	return NOTIFY_DONE;
}

static void rn5t618_i2c_shutdown(struct i2c_client *i2c)
{
	struct rn5t618 *priv = i2c_get_clientdata(i2c);
	int ret;

	if (priv != rn5t618_pm_power_off || priv->variant != RC5T619 ||
			system_state != SYSTEM_POWER_OFF)
		return;

	/* Do the stock preparation while I2C can still sleep. Preserve the
	 * RTC time and fuel gauge; only disable the pending alarm wake path.
	 */
	ret = regmap_write(priv->regmap, RN5T618_REPCNT, 0);
	if (ret)
		dev_err(&i2c->dev, "shutdown REPCNT clear failed: %d\n", ret);
	ret = regmap_write(priv->regmap, RN5T618_INTEN, 0);
	if (ret)
		dev_err(&i2c->dev, "shutdown interrupt mask failed: %d\n", ret);
	ret = regmap_update_bits(priv->regmap, RC5T619_RTC_CTRL1,
				RC5T619_RTC_DALE, 0);
	if (ret)
		dev_err(&i2c->dev, "shutdown RTC alarm disable failed: %d\n", ret);
}

static const struct of_device_id rn5t618_of_match[] = {
	{ .compatible = "ricoh,rn5t567", .data = (void *)RN5T567 },
	{ .compatible = "ricoh,rn5t618", .data = (void *)RN5T618 },
	{ .compatible = "ricoh,rc5t619", .data = (void *)RC5T619 },
	{ }
};
MODULE_DEVICE_TABLE(of, rn5t618_of_match);

static int rn5t618_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	const struct of_device_id *of_id;
	struct rn5t618 *priv;
	int ret;

	of_id = of_match_device(rn5t618_of_match, &i2c->dev);
	if (!of_id) {
		dev_err(&i2c->dev, "Failed to find matching DT ID\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&i2c->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(i2c, priv);
	priv->variant = (long)of_id->data;

	priv->regmap = devm_regmap_init_i2c(i2c, priv->variant == RC5T619 ?
			&rc5t619_regmap_config : &rn5t618_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&i2c->dev, "regmap init failed: %d\n", ret);
		return ret;
	}
	if (priv->variant == RC5T619) {
		ret = regmap_write(priv->regmap, RC5T619_BANKSEL, 0);
		if (ret)
			return ret;
	}

	if (priv->variant == RC5T619) {
		ret = rc5t619_irq_init(i2c, priv);
		if (ret)
			return ret;
	}

	if (priv->variant == RC5T619)
		ret = devm_mfd_add_devices(&i2c->dev, -1, rc5t619_cells,
				   ARRAY_SIZE(rc5t619_cells), NULL, 0, NULL);
	else
		ret = devm_mfd_add_devices(&i2c->dev, -1, rn5t618_cells,
				   ARRAY_SIZE(rn5t618_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(&i2c->dev, "failed to add sub-devices: %d\n", ret);
		return ret;
	}

	rn5t618_pm_power_off = priv;
	if (of_device_is_system_power_controller(i2c->dev.of_node)) {
		if (!pm_power_off) {
			pm_power_off = rn5t618_power_off;
			dev_info(&i2c->dev, "registered PMIC poweroff callback\n");
		} else {
			dev_warn(&i2c->dev, "Poweroff callback already assigned\n");
		}
	}

	rn5t618_restart_handler.notifier_call = rn5t618_restart;
	rn5t618_restart_handler.priority = 192;

	ret = register_restart_handler(&rn5t618_restart_handler);
	if (ret) {
		dev_err(&i2c->dev, "cannot register restart handler, %d\n", ret);
		return ret;
	}

	return 0;
}

static int rn5t618_i2c_remove(struct i2c_client *i2c)
{
	struct rn5t618 *priv = i2c_get_clientdata(i2c);

	if (priv == rn5t618_pm_power_off) {
		rn5t618_pm_power_off = NULL;
		pm_power_off = NULL;
	}

	return 0;
}

static const struct i2c_device_id rn5t618_i2c_id[] = {
	{ }
};
MODULE_DEVICE_TABLE(i2c, rn5t618_i2c_id);

static struct i2c_driver rn5t618_i2c_driver = {
	.driver = {
		.name = "rn5t618",
		.of_match_table = of_match_ptr(rn5t618_of_match),
	},
	.probe = rn5t618_i2c_probe,
	.shutdown = rn5t618_i2c_shutdown,
	.remove = rn5t618_i2c_remove,
	.id_table = rn5t618_i2c_id,
};

module_i2c_driver(rn5t618_i2c_driver);

MODULE_AUTHOR("Beniamino Galvani <b.galvani@gmail.com>");
MODULE_DESCRIPTION("Ricoh RN5T567/RN5T618/RC5T619 MFD driver");
MODULE_LICENSE("GPL v2");
