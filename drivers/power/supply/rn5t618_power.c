/*
 * Power supply driver for the RICOH RN5T618/RC5T619 PMIC family
 *
 * Backport for the NXP Android 8 i.MX6SL kernel used by E70Q10.
 * Property conversion follows the upstream RN5T618 power-supply driver.
 * The opt-in RC5T619 MFD IRQ domain shares charger, VBAT-low and RTC
 * wake events. Awake polling remains a fallback; no battery poll wakes suspend.
 *
 * Copyright (C) 2020 Andreas Kemnade
 * Backport adaptation 2026
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2.
 */

#include <linux/bitops.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/timekeeping.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mfd/rn5t618.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "rn5t618_stock_soc.h"

#define CHG_STATE_ADP_INPUT             0x40
#define CHG_STATE_USB_INPUT             0x80
#define CHG_STATE_MASK                  0x1f
#define CHG_STATE_REPORTED_MASK         (CHG_STATE_MASK | \
                                         CHG_STATE_ADP_INPUT | CHG_STATE_USB_INPUT)
#define CHG_STATE_CHG_OFF               0
#define CHG_STATE_CHG_READY_VADP        1
#define CHG_STATE_CHG_TRICKLE           2
#define CHG_STATE_CHG_RAPID             3
#define CHG_STATE_CHG_COMPLETE          4
#define CHG_STATE_SUSPEND               5
#define CHG_STATE_VCHG_OVER_VOL         6
#define CHG_STATE_BAT_ERROR             7
#define CHG_STATE_NO_BAT                8
#define CHG_STATE_BAT_OVER_VOL          9
#define CHG_STATE_BAT_TEMP_ERR          10
#define CHG_STATE_DIE_ERR               11
#define CHG_STATE_DIE_SHUTDOWN          12
#define CHG_STATE_NO_BAT2               13
#define CHG_STATE_CHG_READY_VUSB        14

#define FG_ENABLE                       BIT(0)
#define RN5T618_POLL_FAST_MS            2000
#define RN5T618_POLL_IDLE_MS            5000

static int rn5t618_sign_extend(u16 value, unsigned int sign_bit)
{
	unsigned int sign = BIT(sign_bit);

	return (int)((value ^ sign) - sign);
}

struct rn5t618_power_info {
	struct rn5t618 *rn5t618;
	struct platform_device *pdev;
	struct power_supply *battery;
	struct power_supply *usb;
	struct power_supply *adp;
	struct delayed_work poll_work;
	int low_battery_irq;
	bool low_battery_pending;
	bool low_battery_irq_fault;
	bool low_battery_stopping;
	int charger_irq;
	bool charger_irq_wake;
	bool charger_irq_fault;
	unsigned int saved_irq_regs[8];
	unsigned long charger_irq_window;
	unsigned int charger_irq_events;
	struct delayed_work charger_irq_work;
	bool charger_irq_stopping;
	unsigned int charger_irq_retries;
	unsigned long charger_irq_retry_after;
	unsigned int charger_stable_state;
	bool charger_stable_valid;
	unsigned int last_chgstate;
	unsigned int last_soc;
	bool poll_valid;
	bool chgstate_valid;
	struct mutex capacity_lock;
	int reported_soc;
	bool reported_valid;
	bool reported_charging;
	bool stock_enabled;
	bool stock_profile_ready;
	bool stock_saved_recent;
	int stock_saved_percent;
	bool stock_resume_pending;
	bool stock_cc_unpause_pending;
	u64 stock_slept_sec;
	u64 stock_low_since;
	u64 stock_last_checkpoint;
	unsigned int charger_irq_generation;
	struct ridi_stock_soc stock_soc;
};

static enum power_supply_property rn5t618_usb_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property rn5t618_adp_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property rn5t618_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static int rn5t618_battery_read_doublereg(struct rn5t618_power_info *info,
					  u8 reg, u16 *result)
{
	int ret, i;
	u8 data[2];
	u16 old = 0, new = 0;

	/* The gauge may update a two-byte value between I2C reads. */
	for (i = 0; i < 3; i++) {
		ret = regmap_bulk_read(info->rn5t618->regmap, reg,
				       data, sizeof(data));
		if (ret)
			return ret;

		new = ((u16)data[0] << 8) | data[1];
		if (new == old)
			break;
		old = new;
	}

	*result = new;
	return 0;
}

static int rn5t618_decode_status(unsigned int status)
{
	switch (status & CHG_STATE_MASK) {
	case CHG_STATE_CHG_OFF:
	case CHG_STATE_SUSPEND:
	case CHG_STATE_VCHG_OVER_VOL:
	case CHG_STATE_DIE_SHUTDOWN:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	case CHG_STATE_CHG_TRICKLE:
	case CHG_STATE_CHG_RAPID:
		return POWER_SUPPLY_STATUS_CHARGING;
	case CHG_STATE_CHG_COMPLETE:
		return POWER_SUPPLY_STATUS_FULL;
	default:
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
}

static int rn5t618_battery_status(struct rn5t618_power_info *info,
				  union power_supply_propval *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &v);
	if (ret)
		return ret;

	if (v & (CHG_STATE_ADP_INPUT | CHG_STATE_USB_INPUT))
		val->intval = rn5t618_decode_status(v);
	else
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;
}

static int rn5t618_battery_health(struct rn5t618_power_info *info,
				  union power_supply_propval *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &v);
	if (ret)
		return ret;

	switch (v & CHG_STATE_MASK) {
	case CHG_STATE_NO_BAT:
	case CHG_STATE_NO_BAT2:
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	case CHG_STATE_VCHG_OVER_VOL:
	case CHG_STATE_BAT_OVER_VOL:
		val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	case CHG_STATE_BAT_ERROR:
	case CHG_STATE_BAT_TEMP_ERR:
	case CHG_STATE_DIE_ERR:
	case CHG_STATE_DIE_SHUTDOWN:
		/* CHGSTATE does not distinguish hot from cold here. */
		val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	default:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}

	return 0;
}

static int rn5t618_battery_present(struct rn5t618_power_info *info,
				   union power_supply_propval *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &v);
	if (ret)
		return ret;

	v &= CHG_STATE_MASK;
	val->intval = !((v == CHG_STATE_NO_BAT) || (v == CHG_STATE_NO_BAT2));
	return 0;
}

static int rn5t618_battery_voltage_now(struct rn5t618_power_info *info,
				       union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_battery_read_doublereg(info, RN5T618_VOLTAGE_1, &res);
	if (ret)
		return ret;

	/* Stock keeps 100uV resolution and this exact integer operation order. */
	if (info->stock_enabled)
		val->intval = (res & 0x0fff) * 50000 / 4095 * 100;
	else
		val->intval = (res & 0x0fff) * 2 * 2500 / 4095 * 1000;
	return 0;
}

static int rn5t618_battery_current_now(struct rn5t618_power_info *info,
				       union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_battery_read_doublereg(info, RN5T618_CC_AVEREG1, &res);
	if (ret)
		return ret;

	/* Negative while discharging; result is in microamps. */
	val->intval = rn5t618_sign_extend(res & 0x3fff, 13) * 1000;
	return 0;
}

static int rn5t618_battery_capacity(struct rn5t618_power_info *info,
				    union power_supply_propval *val)
{
	unsigned int v;
	int ret;

	mutex_lock(&info->capacity_lock);
	if (info->reported_valid) {
		val->intval = info->reported_soc;
		mutex_unlock(&info->capacity_lock);
		return 0;
	}
	mutex_unlock(&info->capacity_lock);

	ret = regmap_read(info->rn5t618->regmap, RN5T618_SOC, &v);
	if (ret)
		return ret;

	val->intval = min_t(unsigned int, v, 100);
	return 0;
}

static int rn5t618_battery_temp(struct rn5t618_power_info *info,
				union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_battery_read_doublereg(info, RN5T618_TEMP_1, &res);
	if (ret)
		return ret;

	/* power_supply temperature is reported in tenths of a degree C. */
	val->intval = rn5t618_sign_extend(res & 0x0fff, 11) * 10 / 16;
	return 0;
}

/* All functions in this section run under capacity_lock. */
static u64 rn5t618_stock_now(void)
{
	return div_u64(ktime_get_boot_ns(), NSEC_PER_SEC);
}

static int rn5t618_stock_read_cc(struct rn5t618_power_info *info, u32 *cc)
{
	u8 data[4];
	u32 old = 0, value;
	int i, ret;

	/* Recheck a bulk read across counter rollover. Do not stop charging or
	 * change the charger IRQ mask just to obtain a gauge sample.
	 */
	for (i = 0; i < 5; i++) {
		ret = regmap_bulk_read(info->rn5t618->regmap,
				       RN5T618_CC_SUMREG3, data, sizeof(data));
		if (ret)
			return ret;
		value = ((u32)data[0] << 24) | ((u32)data[1] << 16) |
			((u32)data[2] << 8) | data[3];
		if (i && ((value == old) ||
		    ((value & 0xffffff00) == (old & 0xffffff00) &&
		     abs((int)(value & 255) - (int)(old & 255)) < 128))) {
			*cc = value;
			return 0;
		}
		old = value;
	}
	return -EAGAIN;
}

static int rn5t618_stock_rtc_hours(struct rn5t618_power_info *info, u32 *hours)
{
	struct rtc_time tm;
	u8 b[7];
	int ret;

	ret = regmap_bulk_read(info->rn5t618->regmap, 0xa0, b, sizeof(b));
	if (ret)
		return ret;
	tm.tm_sec = bcd2bin(b[0] & 0x7f);
	tm.tm_min = bcd2bin(b[1] & 0x7f);
	tm.tm_hour = bcd2bin(b[2] & 0x3f);
	tm.tm_wday = bcd2bin(b[3] & 7);
	tm.tm_mday = bcd2bin(b[4] & 0x3f);
	tm.tm_mon = bcd2bin(b[5] & 0x1f) - 1;
	tm.tm_year = bcd2bin(b[6]) + ((b[5] & 0x80) ? 100 : 0);
	ret = rtc_valid_tm(&tm);
	if (ret || tm.tm_year < 100)
		return -EINVAL;
	*hours = div_s64(rtc_tm_to_time64(&tm), 3600) & 0xffffff;
	return 0;
}

static void rn5t618_stock_load_saved(struct rn5t618_power_info *info)
{
	unsigned int pswr, lo, mid, hi;
	u32 now, saved;
	int ret;

	info->stock_saved_recent = false;
	ret = regmap_read(info->rn5t618->regmap, RN5T618_PSWR, &pswr);
	if (ret || !(pswr & 0x7f) || (pswr & 0x7f) > 100)
		return;
	info->stock_saved_percent = pswr & 0x7f;
	if (rn5t618_stock_rtc_hours(info, &now) ||
	    regmap_read(info->rn5t618->regmap, RC5T619_BANK1(0xb2), &lo) ||
	    regmap_read(info->rn5t618->regmap, RC5T619_BANK1(0xb3), &mid) ||
	    regmap_read(info->rn5t618->regmap, RC5T619_BANK1(0xdd), &hi))
		return;
	saved = lo | (mid << 8) | (hi << 16);
	/* A reset/invalid RTC is not evidence that old PSWR is recent. */
	info->stock_saved_recent = now >= saved && now - saved < 24;
}

static int rn5t618_stock_enable_gauge(struct regmap *map)
{
	unsigned int control;
	int ret = regmap_write(map, RN5T618_CONTROL, 0x11);

	if (ret)
		return ret;
	ret = regmap_read(map, RN5T618_CONTROL, &control);
	if (ret)
		return ret;
	return (control & 0x11) == 0x11 ? 0 : -EIO;
}

static int rn5t618_stock_profile_init(struct rn5t618_power_info *info)
{
	struct regmap *map = info->rn5t618->regmap;
	u8 previous[30], check[30];
	unsigned int control;
	int ret, restore;

	ret = regmap_read(map, RN5T618_CONTROL, &control);
	if (ret)
		return ret;
	ret = regmap_bulk_read(map, RC5T619_BANK1(0xbc), previous,
			       sizeof(previous));
	if (ret)
		return ret;
	if (!memcmp(previous, ridi_stock_profile, sizeof(previous)))
		/* Stock enables FG_ACC (bit4) as well as FG_EN (bit0). Merely
		 * enabling bit0 can leave coulomb accumulation disabled.
		 */
		return rn5t618_stock_enable_gauge(map);

	/* Stock: disable FG while applying BC..D9, then enable with 0x11.
	 * Do not touch charger voltage/current, thermal protection or timers.
	 */
	ret = regmap_update_bits(map, RN5T618_CONTROL, FG_ENABLE, 0);
	if (ret)
		return ret;
	ret = regmap_bulk_write(map, RC5T619_BANK1(0xbc),
				ridi_stock_profile, sizeof(previous));
	if (!ret)
		ret = regmap_bulk_read(map, RC5T619_BANK1(0xbc), check, sizeof(check));
	if (!ret && memcmp(check, ridi_stock_profile, sizeof(check)))
		ret = -EIO;
	if (!ret)
		ret = rn5t618_stock_enable_gauge(map);
	if (ret) {
		restore = regmap_bulk_write(map, RC5T619_BANK1(0xbc),
					    previous, sizeof(previous));
		if (restore)
			dev_err(&info->pdev->dev, "stock profile rollback failed: %d\n", restore);
		restore = regmap_write(map, RN5T618_CONTROL, control);
		if (restore)
			dev_err(&info->pdev->dev, "fuel gauge restore failed: %d\n", restore);
		return ret;
	}
	info->stock_saved_recent = false;
	dev_info(&info->pdev->dev, "restored RIDI battery profile 4 (1123mAh)\n");
	return 0;
}

static int rn5t618_stock_sample(struct rn5t618_power_info *info,
			       unsigned int chgstate, struct ridi_stock_sample *s)
{
	union power_supply_propval voltage, current_prop, temp;
	u16 remaining, full;
	int ret;

	ret = rn5t618_battery_voltage_now(info, &voltage);
	if (ret)
		return ret;
	ret = rn5t618_battery_current_now(info, &current_prop);
	if (ret)
		return ret;
	ret = rn5t618_battery_temp(info, &temp);
	if (ret)
		return ret;
	ret = rn5t618_battery_read_doublereg(info, RN5T618_FA_CAP_H, &full);
	if (ret)
		return ret;
	ret = rn5t618_battery_read_doublereg(info, RN5T618_RE_CAP_H, &remaining);
	if (ret)
		return ret;
	full &= 0x7fff;
	remaining &= 0x7fff;
	if (voltage.intval < 2000000 || voltage.intval > 4500000 ||
	    abs(current_prop.intval) > 1500000 || temp.intval < -400 ||
	    temp.intval > 850 || full < 500 || full > 2000)
		return -ERANGE;
	ret = rn5t618_stock_read_cc(info, &s->cc);
	if (ret)
		return ret;
	s->voltage_uv = voltage.intval;
	s->current_ma = current_prop.intval / 1000;
	s->temp_decic = temp.intval;
	s->full_mah = full;
	s->remaining_mah = remaining;
	s->raw_soc = ridi_stock_temperature_soc(min(10000, remaining * 10000 / full),
					       temp.intval);
	s->online = !!(chgstate & (CHG_STATE_ADP_INPUT | CHG_STATE_USB_INPUT));
	s->complete = (chgstate & CHG_STATE_MASK) == CHG_STATE_CHG_COMPLETE;
	return 0;
}

static int rn5t618_stock_save_time(struct rn5t618_power_info *info)
{
	static const unsigned int regs[] = {
		RC5T619_BANK1(0xb2), RC5T619_BANK1(0xb3), RC5T619_BANK1(0xdd),
	};
	static const u8 unlock[] = { 0xaa, 0x55, 0xaa, 0x55 };
	unsigned int value;
	u32 hours;
	int i, ret;

	ret = rn5t618_stock_rtc_hours(info, &hours);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(unlock); i++) {
		ret = regmap_write(info->rn5t618->regmap, RC5T619_BANK1(0xb0), unlock[i]);
		if (ret)
			return ret;
	}
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		value = (hours >> (i * 8)) & 255;
		ret = regmap_write(info->rn5t618->regmap, regs[i], value);
		if (ret)
			return ret;
		ret = regmap_read(info->rn5t618->regmap, regs[i], &value);
		if (ret)
			return ret;
		if (value != ((hours >> (i * 8)) & 255))
			return -EIO;
	}
	return 0;
}

/* Commit PSWR + zero CC as a pair. CC must be unpaused on EVERY exit. */
static int rn5t618_stock_checkpoint(struct rn5t618_power_info *info)
{
	struct ridi_stock_soc *soc = &info->stock_soc;
	struct regmap *map = info->rn5t618->regmap;
	struct ridi_stock_sample s;
	u8 zero[4] = { 0 };
	unsigned int chgstate;
	int ret, resume_ret, attempt;
	s64 fraction;
	bool wrote_pswr = false;

	if (!soc->initialized || rn5t618_stock_now() < soc->settle_until)
		return 0;
	ret = regmap_write(map, RN5T618_CC_CTRL, 1);
	if (ret)
		goto resume;
	ret = regmap_read(map, RN5T618_CHGSTATE, &chgstate);
	if (ret)
		goto resume;
	ret = rn5t618_stock_sample(info, chgstate, &s);
	if (ret)
		goto resume;
	ridi_stock_step(soc, &s, rn5t618_stock_now());
	ret = regmap_write(map, RN5T618_PSWR, max(1, (soc->dsoc + 50) / 100));
	if (ret)
		goto resume;
	wrote_pswr = true;
	ret = regmap_bulk_write(map, RN5T618_CC_SUMREG3, zero, sizeof(zero));
	if (!ret) {
		fraction = soc->cc_fraction;
		s.cc = 0;
		ridi_stock_rebase(soc, &s);
		soc->cc_fraction = fraction;
	}
resume:
	for (attempt = 0; attempt < 3; attempt++) {
		resume_ret = regmap_write(map, RN5T618_CC_CTRL, 0);
		if (!resume_ret)
			break;
	}
	info->stock_cc_unpause_pending = !!resume_ret;
	if (!ret)
		ret = resume_ret;
	if (ret && wrote_pswr) {
		/* A partial CC clear cannot be combined with the new PSWR at boot.
		 * 0x7f tells our next boot to use voltage instead of double counting.
		 */
		regmap_write(map, RN5T618_PSWR, 0x7f);
		soc->cc_valid = false;
	}
	if (!ret)
		ret = rn5t618_stock_save_time(info);
	if (!ret)
		info->stock_last_checkpoint = rn5t618_stock_now();
	info->reported_soc = (soc->dsoc + 50) / 100;
	return ret;
}

static int rn5t618_stock_update(struct rn5t618_power_info *info,
				unsigned int chgstate)
{
	struct ridi_stock_soc *soc = &info->stock_soc;
	struct ridi_stock_sample s;
	u64 now = rn5t618_stock_now();
	int ret, old_soc;

	mutex_lock(&info->capacity_lock);
	old_soc = info->reported_valid ? info->reported_soc : -1;
	if (info->stock_cc_unpause_pending) {
		ret = regmap_write(info->rn5t618->regmap, RN5T618_CC_CTRL, 0);
		if (ret)
			goto out;
		info->stock_cc_unpause_pending = false;
		soc->cc_valid = false;
	}
	if (!info->stock_profile_ready) {
		ret = rn5t618_stock_profile_init(info);
		if (ret)
			goto out;
		info->stock_profile_ready = true;
	}
	if (now < soc->settle_until) {
		ret = 0;
		goto out;
	}
	ret = rn5t618_stock_sample(info, chgstate, &s);
	if (ret)
		goto out;
	if (!soc->initialized)
		ridi_stock_start(soc, &s, info->stock_saved_percent, info->stock_saved_recent);
	if (soc->settle_until) {
		ridi_stock_rebase(soc, &s);
		soc->settle_until = 0;
	}
	if (info->stock_resume_pending) {
		unsigned int pending = 0;
		bool cable_event = !info->charger_irq || info->charger_irq_fault ||
			READ_ONCE(info->charger_irq_generation) != soc->suspend_irq_events;

		/* Also inspect attach latches before the debounce worker ACKs them.
		 * A masked IRQ must not conceal charging during the sleep interval.
		 */
		if (regmap_read(info->rn5t618->regmap, RN5T618_CHGCTRL_IRR, &pending) ||
		    (pending & 3))
			cable_event = true;
		ridi_stock_sleep(soc, &s, info->stock_slept_sec,
				 cable_event, now);
		info->stock_resume_pending = false;
	} else {
		ridi_stock_step(soc, &s, now);
	}
	/* Stock LOW_VOL decreases at a slow cadence, not every 2s USB poll. */
	if (s.current_ma < 0 && s.voltage_uv <= RIDI_STOCK_CUTOFF_UV &&
	    now - info->stock_low_since >= 60) {
		soc->integrated = max(0, soc->integrated - 100);
		ridi_stock_finish(soc, &s, now);
		info->stock_low_since = now;
	} else if (s.voltage_uv > RIDI_STOCK_CUTOFF_UV || s.current_ma >= 0) {
		info->stock_low_since = now;
	}
	if (READ_ONCE(info->low_battery_pending)) {
		WRITE_ONCE(info->low_battery_pending, false);
		ridi_stock_low_battery(soc, &s);
	}
	info->reported_soc = (soc->dsoc + 50) / 100;
	info->reported_valid = true;
	info->reported_charging = s.online;
	/* Qualified FULL must persist through a reboot/detach. */
	if (old_soc != info->reported_soc || now - info->stock_last_checkpoint >= 29) {
		ret = rn5t618_stock_checkpoint(info);
		if (ret)
			dev_warn_ratelimited(&info->pdev->dev, "stock SOC save failed: %d\n", ret);
	}
	if (soc->full && !soc->reset_done && now - soc->full_since >= RIDI_STOCK_FULL_RESET_SEC) {
		ret = regmap_write(info->rn5t618->regmap, RN5T618_CONTROL, 0x51);
		if (!ret) {
			soc->reset_done = true;
			soc->settle_until = now + RIDI_STOCK_FG_SETTLE_SEC;
			soc->cc_valid = false;
		} else {
			soc->full_since = now; /* bounded retry, no rapid reset loop */
			dev_warn_ratelimited(&info->pdev->dev, "stock full FG reset failed: %d\n", ret);
		}
	}
	ret = old_soc != info->reported_soc;
out:
	mutex_unlock(&info->capacity_lock);
	return ret;
}

static int rn5t618_update_capacity(struct rn5t618_power_info *info,
				 unsigned int raw_soc, unsigned int chgstate)
{
	int old, level = min_t(unsigned int, raw_soc, 100);

	if (info->stock_enabled)
		return rn5t618_stock_update(info, chgstate);
	mutex_lock(&info->capacity_lock);
	old = info->reported_valid ? info->reported_soc : -1;
	info->reported_soc = level;
	info->reported_valid = true;
	info->reported_charging = !!(chgstate & (CHG_STATE_ADP_INPUT | CHG_STATE_USB_INPUT));
	mutex_unlock(&info->capacity_lock);
	return level != old;
}

static ssize_t stock_soc_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct rn5t618_power_info *info = dev_get_drvdata(dev);
	struct ridi_stock_soc *soc = &info->stock_soc;
	ssize_t len;

	mutex_lock(&info->capacity_lock);
	len = scnprintf(buf, PAGE_SIZE,
		"enabled=%d profile_ready=%d dsoc=%d integrated=%d raw=%d full=%d full_anchor=%d online=%d complete=%d zero=%d cc_origin=0x%08x sample_cc=0x%08x voltage_uv=%d current_ma=%d last_sleep_sec=%llu sleep_model=%d sleep_cc_mas=%d cc_discontinuities=%u low_irq=%d low_pending=%d low_irq_fault=%d\n",
		info->stock_enabled, info->stock_profile_ready, soc->dsoc, soc->integrated,
		soc->last.raw_soc, soc->full, soc->full_anchor,
		soc->last.online, soc->last.complete, soc->zero, soc->last_cc, soc->last.cc,
		soc->last.voltage_uv, soc->last.current_ma,
		(unsigned long long)soc->last_sleep_sec, soc->used_sleep_model,
		soc->last_sleep_cc_mas,
		soc->cc_discontinuities, info->low_battery_irq,
		READ_ONCE(info->low_battery_pending), info->low_battery_irq_fault);
	mutex_unlock(&info->capacity_lock);
	return len;
}
static DEVICE_ATTR_RO(stock_soc);


static int rn5t618_battery_charge_full(struct rn5t618_power_info *info,
				       union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_battery_read_doublereg(info, RN5T618_FA_CAP_H, &res);
	if (ret)
		return ret;

	val->intval = res * 1000;
	return 0;
}

static int rn5t618_battery_charge_now(struct rn5t618_power_info *info,
				      union power_supply_propval *val)
{
	u16 res;
	int ret;

	ret = rn5t618_battery_read_doublereg(info, RN5T618_RE_CAP_H, &res);
	if (ret)
		return ret;

	val->intval = res * 1000;
	return 0;
}

/* A fuel-gauge reset temporarily exposes uninitialized ADC/capacity registers.
 * Hold the same lock as the reset command across live property reads, and use
 * the last validated sample until the poll has accepted a post-reset sample.
 * Raw helpers remain separate: stock_sample already holds capacity_lock.
 */
static int rn5t618_battery_fg_property(struct rn5t618_power_info *info,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct ridi_stock_soc *soc = &info->stock_soc;
	int ret = 0;

	mutex_lock(&info->capacity_lock);
	if (info->stock_enabled && soc->initialized && soc->settle_until) {
		switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = soc->last.voltage_uv;
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = soc->last.current_ma * 1000;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = soc->last.temp_decic;
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			val->intval = soc->last.full_mah * 1000;
			break;
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			val->intval = soc->last.remaining_mah * 1000;
			break;
		default:
			ret = -EINVAL;
		}
	} else {
		switch (psp) {
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = rn5t618_battery_voltage_now(info, val);
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			ret = rn5t618_battery_current_now(info, val);
			break;
		case POWER_SUPPLY_PROP_TEMP:
			ret = rn5t618_battery_temp(info, val);
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			ret = rn5t618_battery_charge_full(info, val);
			break;
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			ret = rn5t618_battery_charge_now(info, val);
			break;
		default:
			ret = -EINVAL;
		}
	}
	mutex_unlock(&info->capacity_lock);
	return ret;
}

static int rn5t618_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct rn5t618_power_info *info = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return rn5t618_battery_status(info, val);
	case POWER_SUPPLY_PROP_HEALTH:
		return rn5t618_battery_health(info, val);
	case POWER_SUPPLY_PROP_PRESENT:
		return rn5t618_battery_present(info, val);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		return rn5t618_battery_fg_property(info, psp, val);
	case POWER_SUPPLY_PROP_CAPACITY:
		return rn5t618_battery_capacity(info, val);
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	default:
		return -EINVAL;
	}
}

static int rn5t618_adp_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct rn5t618_power_info *info = power_supply_get_drvdata(psy);
	unsigned int chgstate;
	bool online;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &chgstate);
	if (ret)
		return ret;

	online = !!(chgstate & CHG_STATE_ADP_INPUT);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = online;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		if (!online) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			return 0;
		}
		val->intval = rn5t618_decode_status(chgstate);
		return 0;
	default:
		return -EINVAL;
	}
}

static int rn5t618_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct rn5t618_power_info *info = power_supply_get_drvdata(psy);
	unsigned int chgstate;
	bool online;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &chgstate);
	if (ret)
		return ret;

	online = !!(chgstate & CHG_STATE_USB_INPUT);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = online;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		if (!online) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			return 0;
		}
		val->intval = rn5t618_decode_status(chgstate);
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc rn5t618_battery_desc = {
	.name = "rn5t618-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = rn5t618_battery_props,
	.num_properties = ARRAY_SIZE(rn5t618_battery_props),
	.get_property = rn5t618_battery_get_property,
};

static const struct power_supply_desc rn5t618_adp_desc = {
	.name = "rn5t618-adp",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = rn5t618_adp_props,
	.num_properties = ARRAY_SIZE(rn5t618_adp_props),
	.get_property = rn5t618_adp_get_property,
};

static const struct power_supply_desc rn5t618_usb_desc = {
	.name = "rn5t618-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = rn5t618_usb_props,
	.num_properties = ARRAY_SIZE(rn5t618_usb_props),
	.get_property = rn5t618_usb_get_property,
};

static void rn5t618_power_notify(struct rn5t618_power_info *info,
				 bool charger_changed)
{
	power_supply_changed(info->battery);
	if (charger_changed) {
		power_supply_changed(info->adp);
		power_supply_changed(info->usb);
	}
}

/* Stock ricoh619-irq.c: charger masks use 1=masked, IRQ requests
 * clear by writing the complement, and CHG is master interrupt bit 6.
 * The MFD owns the physical line; this consumer owns only the charger group.
 */
#define RC5T619_CHARGER_MASTER BIT(6)
#define RC5T619_ATTACH_DET (BIT(0) | BIT(1))
#define RC5T619_CHARGE_STATE_DET (BIT(0) | BIT(1))
#define RC5T619_EVENT_HOLD_MS 2000
#define RC5T619_CHGEXTIF_MASK 0xd1
#define RC5T619_CHGEXTIF_IRQ 0xd2

static const unsigned int rc5t619_irq_regs[] = {
	RN5T618_CHGCTRL_IRFMASK,
	RN5T618_CHGSTAT_IRFMASK1, RN5T618_CHGSTAT_IRFMASK2,
	RN5T618_CHGERR_IRFMASK, RN5T618_CHGCTRL_DETMOD1,
	RN5T618_CHGSTAT_DETMOD1, RC5T619_CHGEXTIF_MASK,
};

static void rn5t618_restore_irq_regs(struct rn5t618_power_info *info)
{
	int i;

	/* Master enables are owned exclusively by the shared MFD IRQ domain. */
	for (i = ARRAY_SIZE(rc5t619_irq_regs) - 1; i >= 0; i--)
		if (regmap_write(info->rn5t618->regmap, rc5t619_irq_regs[i],
				info->saved_irq_regs[i]))
			dev_warn(&info->pdev->dev, "cannot restore IRQ register %#x\n",
				 rc5t619_irq_regs[i]);
}

#define RC5T619_IRQ_SETTLE_MS 250
#define RC5T619_IRQ_RECOVERY_MS 5000
#define RC5T619_IRQ_SETTLE_TRIES 4

static void rn5t618_queue_charger_work(struct rn5t618_power_info *info)
{
	if (!READ_ONCE(info->charger_irq_stopping))
		queue_delayed_work(system_freezable_wq, &info->charger_irq_work,
				   msecs_to_jiffies(RC5T619_IRQ_SETTLE_MS));
}

static void rn5t618_charger_irq_work(struct work_struct *work)
{
	struct rn5t618_power_info *info = container_of(to_delayed_work(work),
			struct rn5t618_power_info, charger_irq_work);
	struct regmap *map = info->rn5t618->regmap;
	unsigned int state, pending;
	bool activity = false;
	unsigned int reg;
	int ret;

	WRITE_ONCE(info->charger_irq_generation, READ_ONCE(info->charger_irq_generation) + 1);
	if (READ_ONCE(info->charger_irq_stopping))
		return;
	/* The physical line and charger master stay masked while a faulty
	 * connector bounces. Drain all charger-group requests owned here,
	 * including masked events; do not acknowledge other master groups.
	 */
	for (reg = RN5T618_CHGCTRL_IRR; reg <= RN5T618_CHGERR_IRR; reg++) {
		ret = regmap_read(map, reg, &pending);
		if (ret)
			goto fault;
		activity |= pending != 0;
		if (pending) {
			ret = regmap_write(map, reg, (~pending) & 0xff);
			if (ret)
				goto fault;
		}
	}
	/* Stock has a fifth charger interrupt bank (external USB interface).
	 * Leaving it unmasked/unacknowledged can hold the shared line LOW.
	 */
	ret = regmap_read(map, RC5T619_CHGEXTIF_IRQ, &pending);
	if (ret)
		goto fault;
	activity |= pending != 0;
	if (pending) {
		ret = regmap_write(map, RC5T619_CHGEXTIF_IRQ, (~pending) & 0xff);
		if (ret)
			goto fault;
	}
	ret = regmap_read(map, RN5T618_CHGSTATE, &state);
	if (ret)
		goto fault;
	state &= CHG_STATE_REPORTED_MASK;
	if (!info->charger_stable_valid || state != info->charger_stable_state || activity) {
		info->charger_stable_state = state;
		info->charger_stable_valid = true;
		if (++info->charger_irq_retries < RC5T619_IRQ_SETTLE_TRIES) {
			rn5t618_queue_charger_work(info);
			return;
		}
		ret = -EAGAIN;
		goto fault;
	}
	/* Two matching readings separated by 250 ms: publish only the final
	 * state, then rearm. Rapid raw events cannot continually extend wake.
	 */
	rn5t618_power_notify(info, true);
	if (!info->charger_irq_wake) {
		ret = enable_irq_wake(info->charger_irq);
		if (ret)
			goto fault;
		info->charger_irq_wake = true;
	}
	info->charger_irq_fault = false;
	enable_irq(info->charger_irq);
	return;
fault:
	if (info->charger_irq_wake) {
		disable_irq_wake(info->charger_irq);
		info->charger_irq_wake = false;
	}
	info->charger_stable_valid = false;
	info->charger_irq_retry_after = jiffies +
		msecs_to_jiffies(RC5T619_IRQ_RECOVERY_MS);
	dev_warn(&info->pdev->dev,
		 "charger IRQ quarantined (%d); recover on stable awake poll\n", ret);
}

static irqreturn_t rn5t618_charger_irq(int irq, void *data)
{
	struct rn5t618_power_info *info = data;
	int ret;

	WRITE_ONCE(info->charger_irq_generation, READ_ONCE(info->charger_irq_generation) + 1);
	if (READ_ONCE(info->charger_irq_stopping))
		return IRQ_HANDLED;
	/* Mask immediately, as stock does. A burst is coalesced into a single
	 * bounded debounce worker, instead of repeatedly running the handler.
	 */
	disable_irq_nosync(irq);
	info->charger_irq_fault = true;
	info->charger_stable_valid = false;
	info->charger_irq_retries = 0;
	info->charger_irq_retry_after = jiffies +
		msecs_to_jiffies(RC5T619_IRQ_RECOVERY_MS);
	ret = 0; /* disable_irq_nosync masks only the nested charger group. */
	if (time_after(jiffies, info->charger_irq_window + HZ)) {
		info->charger_irq_window = jiffies;
		info->charger_irq_events = 0;
	}
	if (ret || ++info->charger_irq_events > 2) {
		if (!ret)
			ret = -ELOOP;
		if (info->charger_irq_wake) {
			disable_irq_wake(irq);
			info->charger_irq_wake = false;
		}
		info->charger_irq_retry_after = jiffies +
			msecs_to_jiffies(RC5T619_IRQ_RECOVERY_MS);
		dev_warn(&info->pdev->dev, "charger IRQ burst masked (%d)\n", ret);
		return IRQ_HANDLED;
	}
	pm_wakeup_event(&info->pdev->dev, RC5T619_EVENT_HOLD_MS);
	rn5t618_queue_charger_work(info);
	return IRQ_HANDLED;
}

static void rn5t618_setup_charger_irq(struct rn5t618_power_info *info)
{
	struct device *parent = info->pdev->dev.parent;
	int irq = rc5t619_get_irq(info->rn5t618, RC5T619_IRQ_CHARGER);
	struct regmap *map = info->rn5t618->regmap;
	int i, ret;

	if (info->rn5t618->variant != RC5T619 ||
	    !of_property_read_bool(parent->of_node, "ricoh,charger-wakeup"))
		return;
	if (irq <= 0) {
		dev_warn(&info->pdev->dev, "charger wake has no physical PMIC IRQ\n");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(rc5t619_irq_regs); i++) {
		ret = regmap_read(map, rc5t619_irq_regs[i], &info->saved_irq_regs[i]);
		if (ret)
			goto warn;
	}
	/* Other PMIC groups remain owned by their nested IRQ consumers. */
	ret = regmap_write(map, RN5T618_CHGCTRL_IRFMASK, 0xff);
	if (ret)
		goto restore;
	ret = regmap_write(map, RN5T618_CHGSTAT_IRFMASK1, 0xff);
	if (ret)
		goto restore;
	ret = regmap_write(map, RN5T618_CHGSTAT_IRFMASK2, 0xff);
	if (ret)
		goto restore;
	ret = regmap_write(map, RN5T618_CHGERR_IRFMASK, 0xff);
	if (ret)
		goto restore;
	ret = regmap_write(map, RC5T619_CHGEXTIF_MASK, 0xff);
	if (ret)
		goto restore;
	/* Stock: [3:2]=11 for VUSB and [1:0]=11 for VADP, both edges. */
	ret = regmap_update_bits(map, RN5T618_CHGCTRL_DETMOD1, 0x0f, 0x0f);
	if (ret)
		goto restore;
	ret = regmap_write(map, RN5T618_CHGCTRL_IRR,
			   (~RC5T619_ATTACH_DET) & 0xff);
	if (ret)
		goto restore;
	/* Stock rising edges: charging begins / completes. The attach IRQ can
	 * precede CHARGING; this second event ensures the LED updates then.
	 */
	ret = regmap_update_bits(map, RN5T618_CHGSTAT_DETMOD1, 0x0f, 0x05);
	if (ret)
		goto restore;
	ret = regmap_write(map, RN5T618_CHGSTAT_IRR1,
			   (~RC5T619_CHARGE_STATE_DET) & 0xff);
	if (ret)
		goto restore;
	ret = devm_request_threaded_irq(&info->pdev->dev, irq, NULL,
			rn5t618_charger_irq, IRQF_ONESHOT, "rc5t619-charger", info);
	if (ret)
		goto wake_cleanup;
	info->charger_irq = irq;
	info->charger_irq_window = jiffies;
	ret = enable_irq_wake(irq);
	if (ret)
		goto irq_cleanup;
	info->charger_irq_wake = true;
	ret = regmap_write(map, RN5T618_CHGCTRL_IRFMASK,
			   (~RC5T619_ATTACH_DET) & 0xff);
	if (ret)
		goto irq_cleanup;
	ret = regmap_write(map, RN5T618_CHGSTAT_IRFMASK1,
			   (~RC5T619_CHARGE_STATE_DET) & 0xff);
	if (ret)
		goto irq_cleanup;
	dev_info(&info->pdev->dev, "charger attach/detach wake IRQ enabled (debounced) (%d)\n",
		 info->charger_irq);
	return;

irq_cleanup:
	WRITE_ONCE(info->charger_irq_stopping, true);
	disable_irq(irq);
	cancel_delayed_work_sync(&info->charger_irq_work);
	if (info->charger_irq_wake) {
		disable_irq_wake(irq);
		info->charger_irq_wake = false;
	}
	devm_free_irq(&info->pdev->dev, irq, info);
	info->charger_irq = 0;
wake_cleanup:
restore:
	rn5t618_restore_irq_regs(info);
warn:
	dev_warn(&info->pdev->dev,
		 "charger IRQ setup failed (%d); retaining polling\n", ret);
}

static void rn5t618_stop_charger_irq(struct rn5t618_power_info *info,
				    bool restore)
{
	if (!info->charger_irq)
		return;
	/* Stop notifications before poweroff/remove. Poweroff must never
	 * re-enable inherited wake sources by restoring the bootloader mask.
	 */
	WRITE_ONCE(info->charger_irq_stopping, true);
	disable_irq(info->charger_irq);
	cancel_delayed_work_sync(&info->charger_irq_work);
	if (info->charger_irq_wake) {
		disable_irq_wake(info->charger_irq);
		info->charger_irq_wake = false;
	}
	devm_free_irq(&info->pdev->dev, info->charger_irq, info);
	info->charger_irq = 0;
	if (restore)
		rn5t618_restore_irq_regs(info);
}

/* Return the next awake poll delay, or an I2C error. Resume calls this
 * synchronously after the I2C parent has resumed, before returning to userspace.
 */
static int rn5t618_refresh_power(struct rn5t618_power_info *info, bool force)
{
	unsigned int chgstate, soc = 0;
	bool charger_changed, sample_valid = false;
	int capacity_changed = 0, sample_error = 0;
	int ret;

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &chgstate);
	if (ret)
		return ret; /* No connection state may be invented on I2C failure. */
	chgstate &= CHG_STATE_REPORTED_MASK;
	charger_changed = !info->chgstate_valid || chgstate != info->last_chgstate;
	ret = regmap_read(info->rn5t618->regmap, RN5T618_SOC, &soc);
	if (ret) {
		sample_error = ret;
		goto notify;
	}
	capacity_changed = rn5t618_update_capacity(info, soc, chgstate);
	if (capacity_changed < 0) {
		dev_warn_ratelimited(&info->pdev->dev, "battery SOC sample failed: %d; retaining last valid level\n", capacity_changed);
		sample_error = capacity_changed;
		goto notify;
	}
	sample_valid = true;
	/* Restore an ADC consumer quarantined by a transient I2C failure. */
	mutex_lock(&info->capacity_lock);
	if (info->low_battery_irq_fault && !info->low_battery_stopping) {
		ret = regmap_update_bits(info->rn5t618->regmap, RN5T618_EN_ADCIR1, BIT(1), 0);
		if (!ret)
			ret = regmap_write(info->rn5t618->regmap, RN5T618_IR_ADC1, (~BIT(1)) & 0xff);
		if (!ret) {
			info->low_battery_irq_fault = false;
			enable_irq(info->low_battery_irq);
		}
	}
	mutex_unlock(&info->capacity_lock);
notify:
	/* Cable status and charger IRQ recovery are independent of the gauge.
	 * A rejected capacity sample must not retain an old plugged-in state
	 * or leave a quarantined charger group disabled indefinitely.
	 * Recovery uses existing awake polls, not a new suspend wake timer.
	 */
	if (info->charger_irq && info->charger_irq_fault &&
	    !READ_ONCE(info->charger_irq_stopping) &&
	    time_after_eq(jiffies, info->charger_irq_retry_after) &&
	    !delayed_work_pending(&info->charger_irq_work)) {
		info->charger_irq_retry_after = jiffies +
			msecs_to_jiffies(RC5T619_IRQ_RECOVERY_MS);
		info->charger_irq_retries = 0;
		rn5t618_queue_charger_work(info);
	}
	if (force || charger_changed ||
	    (sample_valid && (!info->poll_valid || soc != info->last_soc || capacity_changed))) {
		info->last_chgstate = chgstate;
		info->chgstate_valid = true;
		if (sample_valid) {
			info->last_soc = soc;
			info->poll_valid = true;
		}
		rn5t618_power_notify(info, force || charger_changed);
	}
	if (sample_error)
		return sample_error;
	return (chgstate & (CHG_STATE_ADP_INPUT | CHG_STATE_USB_INPUT))
		? RN5T618_POLL_FAST_MS : RN5T618_POLL_IDLE_MS;
}

static void rn5t618_power_poll(struct work_struct *work)
{
	struct rn5t618_power_info *info = container_of(to_delayed_work(work),
						      struct rn5t618_power_info,
						      poll_work);
	int delay_ms = rn5t618_refresh_power(info, false);

	schedule_delayed_work(&info->poll_work,
		msecs_to_jiffies(delay_ms < 0 ? RN5T618_POLL_IDLE_MS : delay_ms));
}


#define RC5T619_VBAT_LOW BIT(1)

static irqreturn_t rn5t618_low_battery_irq(int irq, void *arg)
{
	struct rn5t618_power_info *info = arg;
	int ret;
	unsigned int pending;

	if (READ_ONCE(info->low_battery_stopping))
		return IRQ_HANDLED;
	ret = regmap_read(info->rn5t618->regmap, RN5T618_IR_ADC1, &pending);
	if (ret) {
		disable_irq_nosync(irq);
		info->low_battery_irq_fault = true;
		pm_wakeup_event(&info->pdev->dev, 5000);
		mod_delayed_work(system_wq, &info->poll_work, 0);
		return IRQ_HANDLED; /* no invented low-battery event on I2C failure */
	}
	if (!(pending & RC5T619_VBAT_LOW))
		return IRQ_NONE;
	/* A low level must not retrigger forever. Re-arm on the next suspend;
	 * retain the software latch until a complete validated gauge sample. */
	ret = regmap_update_bits(info->rn5t618->regmap, RN5T618_EN_ADCIR1,
				 RC5T619_VBAT_LOW, 0);
	if (!ret)
		ret = regmap_write(info->rn5t618->regmap, RN5T618_IR_ADC1,
				   (~RC5T619_VBAT_LOW) & 0xff);
	if (ret) {
		disable_irq_nosync(irq);
		info->low_battery_irq_fault = true;
		dev_err(&info->pdev->dev, "VBAT IRQ acknowledge failed: %d\n", ret);
	}
	WRITE_ONCE(info->low_battery_pending, true);
	pm_wakeup_event(&info->pdev->dev, 5000);
	mod_delayed_work(system_wq, &info->poll_work, 0);
	return IRQ_HANDLED;
}

static int rn5t618_setup_low_battery_irq(struct rn5t618_power_info *info)
{
	struct regmap *map = info->rn5t618->regmap;
	int irq, ret;

	if (!info->stock_enabled)
		return 0;
	irq = rc5t619_get_irq(info->rn5t618, RC5T619_IRQ_VBAT_LOW);
	if (irq < 0)
		return irq;
	/* Stock LP_PANEL: VBAT threshold 3550mV, 250ms auto ADC, four samples.
	 * This configures a warning comparator, not the PMIC power-cut voltage. */
	ret = regmap_write(map, RN5T618_EN_ADCIR1, 0);
	if (!ret)
		ret = regmap_write(map, RN5T618_EN_ADCIR2, 0);
	if (!ret)
		ret = regmap_write(map, RN5T618_EN_ADCIR3, 0);
	if (!ret)
		ret = regmap_write(map, RN5T618_ADCCNT3, 0);
	if (!ret)
		ret = regmap_write(map, RN5T618_ADCCNT2, 0);
	if (!ret)
		ret = regmap_write(map, RN5T618_ADCCNT1, 0x12);
	if (!ret)
		ret = regmap_write(map, RN5T618_VBATTHL, 3550 * 255 / 5000);
	if (!ret)
		ret = regmap_write(map, RN5T618_IR_ADC1, (~RC5T619_VBAT_LOW) & 0xff);
	if (!ret)
		ret = regmap_write(map, RN5T618_ADCCNT3, 0x28);
	if (ret)
		return ret;
	ret = devm_request_threaded_irq(&info->pdev->dev, irq, NULL,
		rn5t618_low_battery_irq, IRQF_ONESHOT, "rc5t619-vbat-low", info);
	if (ret)
		return ret;
	ret = enable_irq_wake(irq);
	if (ret)
		return ret;
	info->low_battery_irq = irq;
	ret = regmap_update_bits(map, RN5T618_EN_ADCIR1,
				 RC5T619_VBAT_LOW, RC5T619_VBAT_LOW);
	if (ret) {
		disable_irq_wake(irq);
		info->low_battery_irq = 0;
	}
	return ret;
}

static void rn5t618_stop_low_battery_irq(struct rn5t618_power_info *info)
{
	mutex_lock(&info->capacity_lock);
	WRITE_ONCE(info->low_battery_stopping, true);
	mutex_unlock(&info->capacity_lock);
	if (info->low_battery_irq) {
		regmap_update_bits(info->rn5t618->regmap, RN5T618_EN_ADCIR1,
				   RC5T619_VBAT_LOW, 0);
		disable_irq_wake(info->low_battery_irq);
		devm_free_irq(&info->pdev->dev, info->low_battery_irq, info);
		info->low_battery_irq = 0;
	}
}

static void rn5t618_wake_cleanup(void *arg)
{
	struct rn5t618_power_info *info = arg;
	device_init_wakeup(&info->pdev->dev, false);
}

static void rn5t618_poll_cleanup(void *arg)
{
	struct rn5t618_power_info *info = arg;
	cancel_delayed_work_sync(&info->poll_work);
	cancel_delayed_work_sync(&info->charger_irq_work);
}

static int rn5t618_power_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = { };
	struct rn5t618_power_info *info;
	unsigned int v;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->pdev = pdev;
	info->rn5t618 = dev_get_drvdata(pdev->dev.parent);
	if (!info->rn5t618)
		return -ENODEV;
	platform_set_drvdata(pdev, info);
	mutex_init(&info->capacity_lock);
	info->stock_enabled = info->rn5t618->variant == RC5T619 &&
		of_property_read_bool(pdev->dev.parent->of_node, "ricoh,ridi-stock-battery");
	if (info->stock_enabled)
		rn5t618_stock_load_saved(info);

	ret = regmap_read(info->rn5t618->regmap, RN5T618_CONTROL, &v);
	if (ret)
		return ret;

	if (!(v & FG_ENABLE)) {
		/* Some e-reader vendor kernels disable the gauge at shutdown. */
		dev_warn(&pdev->dev,
			 "fuel gauge was disabled; enabling it (initial SOC may need time to settle)\n");
		ret = regmap_update_bits(info->rn5t618->regmap, RN5T618_CONTROL,
					 FG_ENABLE, FG_ENABLE);
		if (ret)
			return ret;
	}

	/* Seed the published level before healthd can request it. */
	ret = regmap_read(info->rn5t618->regmap, RN5T618_CHGSTATE, &v);
	if (!ret) {
		unsigned int soc;

		ret = regmap_read(info->rn5t618->regmap, RN5T618_SOC, &soc);
		if (!ret)
			ret = rn5t618_update_capacity(info, soc, v);
	}

	psy_cfg.drv_data = info;

	info->battery = devm_power_supply_register(&pdev->dev,
						   &rn5t618_battery_desc,
						   &psy_cfg);
	if (IS_ERR(info->battery)) {
		ret = PTR_ERR(info->battery);
		dev_err(&pdev->dev, "failed to register battery: %d\n", ret);
		return ret;
	}

	info->adp = devm_power_supply_register(&pdev->dev,
					       &rn5t618_adp_desc,
					       &psy_cfg);
	if (IS_ERR(info->adp)) {
		ret = PTR_ERR(info->adp);
		dev_err(&pdev->dev, "failed to register adapter: %d\n", ret);
		return ret;
	}

	info->usb = devm_power_supply_register(&pdev->dev,
					       &rn5t618_usb_desc,
					       &psy_cfg);
	if (IS_ERR(info->usb)) {
		ret = PTR_ERR(info->usb);
		dev_err(&pdev->dev, "failed to register usb: %d\n", ret);
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_stock_soc);
	if (ret)
		dev_warn(&pdev->dev, "stock SOC diagnostic unavailable: %d\n", ret);
	INIT_DELAYED_WORK(&info->poll_work, rn5t618_power_poll);
	INIT_DELAYED_WORK(&info->charger_irq_work, rn5t618_charger_irq_work);
	if (info->rn5t618->irq_data) {
		ret = device_init_wakeup(&pdev->dev, true);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(&pdev->dev, rn5t618_wake_cleanup, info);
		if (ret)
			return ret;
	}
	ret = devm_add_action_or_reset(&pdev->dev, rn5t618_poll_cleanup, info);
	if (ret)
		return ret;
	ret = rn5t618_setup_low_battery_irq(info);
	if (ret)
		return ret;
	rn5t618_setup_charger_irq(info);
	schedule_delayed_work(&info->poll_work, 0);

	dev_info(&pdev->dev,
		 "registered battery/adapter/USB power supplies (charger IRQ %s)\n",
		 info->charger_irq ? "enabled" : "unavailable; polling");
	return 0;
}

static int rn5t618_power_remove(struct platform_device *pdev)
{
	struct rn5t618_power_info *info = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_stock_soc);
	rn5t618_stop_charger_irq(info, true);
	rn5t618_stop_low_battery_irq(info);
	cancel_delayed_work_sync(&info->poll_work);
	return 0;
}

static int rn5t618_power_suspend(struct device *dev)
{
	struct rn5t618_power_info *info = dev_get_drvdata(dev);

	int ret;

	/* No periodic I2C polling while the PMIC/I2C parent is suspended. */
	cancel_delayed_work_sync(&info->poll_work);
	if (info->stock_enabled) {
		mutex_lock(&info->capacity_lock);
		ret = rn5t618_stock_checkpoint(info);
		if (ret)
			dev_warn(dev, "pre-suspend SOC save failed: %d\n", ret);
		info->stock_soc.suspend_online = info->stock_soc.last.online;
		info->stock_soc.suspend_valid = info->stock_soc.initialized &&
			!info->stock_cc_unpause_pending;
		info->stock_soc.suspend_irq_events = READ_ONCE(info->charger_irq_generation);
		info->stock_soc.suspend_boot_ns = ktime_get_boot_ns();
		info->stock_soc.suspend_mono_ns = ktime_get_ns();
		mutex_unlock(&info->capacity_lock);
	}
	if (info->low_battery_irq_fault) {
		schedule_delayed_work(&info->poll_work, 0);
		return -EIO;
	}
	if (info->low_battery_irq) {
		ret = regmap_update_bits(info->rn5t618->regmap, RN5T618_EN_ADCIR1,
				 RC5T619_VBAT_LOW, RC5T619_VBAT_LOW);
		if (ret) {
			schedule_delayed_work(&info->poll_work, 0);
			return ret;
		}
	}
	return 0;
}

static int rn5t618_power_resume(struct device *dev)
{
	struct rn5t618_power_info *info = dev_get_drvdata(dev);

	int delay_ms;
	unsigned int pending;

	/* Stock resume reads the latched VBAT request before userspace. */
	if (info->low_battery_irq &&
	    !regmap_read(info->rn5t618->regmap, RN5T618_IR_ADC1, &pending) &&
	    (pending & RC5T619_VBAT_LOW)) {
		regmap_update_bits(info->rn5t618->regmap, RN5T618_EN_ADCIR1,
				   RC5T619_VBAT_LOW, 0);
		regmap_write(info->rn5t618->regmap, RN5T618_IR_ADC1,
			     (~RC5T619_VBAT_LOW) & 0xff);
		WRITE_ONCE(info->low_battery_pending, true);
		pm_wakeup_event(dev, 5000);
	}

	if (info->stock_enabled && info->stock_soc.suspend_valid) {
		u64 boot_elapsed = ktime_get_boot_ns() - info->stock_soc.suspend_boot_ns;
		u64 mono_elapsed = ktime_get_ns() - info->stock_soc.suspend_mono_ns;

		info->stock_slept_sec = boot_elapsed > mono_elapsed ?
			div_u64(boot_elapsed - mono_elapsed, NSEC_PER_SEC) : 0;
		info->stock_resume_pending = true;
		info->stock_soc.suspend_valid = false;
	}
	/* Force battery/USB/adapter uevents even when SOC is unchanged:
	 * voltage, current and status still need refreshing after long sleep.
	 */
	delay_ms = rn5t618_refresh_power(info, true);
	if (delay_ms < 0)
		dev_warn(dev, "resume battery refresh failed: %d; retry queued\n", delay_ms);
	schedule_delayed_work(&info->poll_work,
		msecs_to_jiffies(delay_ms < 0 ? 0 : delay_ms));
	return 0;
}

static void rn5t618_power_shutdown(struct platform_device *pdev)
{
	struct rn5t618_power_info *info = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&info->poll_work);
	rn5t618_stop_charger_irq(info, false);
	rn5t618_stop_low_battery_irq(info);
	if (info->stock_enabled) {
		int ret;

		mutex_lock(&info->capacity_lock);
		ret = rn5t618_stock_checkpoint(info);
		mutex_unlock(&info->capacity_lock);
		if (ret)
			dev_err(&pdev->dev, "shutdown SOC save failed: %d\n", ret);
	}
}

static SIMPLE_DEV_PM_OPS(rn5t618_power_pm_ops,
			rn5t618_power_suspend, rn5t618_power_resume);

static struct platform_driver rn5t618_power_driver = {
	.probe = rn5t618_power_probe,
	.remove = rn5t618_power_remove,
	.shutdown = rn5t618_power_shutdown,
	.driver = {
		.name = "rn5t618-power",
		.pm = &rn5t618_power_pm_ops,
	},
};
module_platform_driver(rn5t618_power_driver);

MODULE_ALIAS("platform:rn5t618-power");
MODULE_DESCRIPTION("RN5T618/RC5T619 charger and fuel gauge power-supply driver");
MODULE_AUTHOR("Andreas Kemnade; E70Q10 backport adaptation");
MODULE_LICENSE("GPL v2");
