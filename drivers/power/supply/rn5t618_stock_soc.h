/* SPDX-License-Identifier: GPL-2.0 */
/*
 * E70Q10 SOC accounting. Constants/conditions recovered from the RIDI stock
 * Image and the Ricoh/NTX driver. This is an adaptation, not a claim that the
 * unavailable RIDI C source has been reproduced byte for byte.
 *
 * Keep arithmetic separate from I2C: failed samples must never alter SOC.
 */
#ifndef RN5T618_STOCK_SOC_H
#define RN5T618_STOCK_SOC_H

#include <linux/math64.h>

#define RIDI_STOCK_CAP_MAH 1123
#define RIDI_STOCK_R_MOHM 238
#define RIDI_STOCK_CUTOFF_UV 3600000
#define RIDI_STOCK_SUSPEND_UA 4970
#define RIDI_STOCK_FULL_RESET_SEC (4 * 29)
#define RIDI_STOCK_FG_SETTLE_SEC 6

/* Exact 32-byte profile at stock Image offset 0x75e4f8 (LP-TFT 1200 mAh). */
static const u8 ridi_stock_profile[] = {
	0x0b, 0xb4, 0x0b, 0xd7, 0x0b, 0xfd, 0x0c, 0x11,
	0x0c, 0x1d, 0x0c, 0x32, 0x0c, 0x69, 0x0c, 0x91,
	0x0c, 0xbe, 0x0c, 0xfd, 0x0d, 0x42, 0x04, 0x63,
	0x00, 0x64, 0x0f, 0xc8, 0x05, 0x2c, 0x22, 0x56,
};

struct ridi_stock_sample {
	u32 cc; /* two's-complement mAs, including wrap */
	int voltage_uv;
	int current_ma;
	int temp_decic;
	int full_mah;
	int remaining_mah;
	int raw_soc; /* 0.01%, temperature compensated like calc_capacity_2 */
	bool online;
	bool complete;
};

struct ridi_stock_soc {
	int dsoc; /* 0.01%; retain fractional charge between polls */
	int integrated;
	u32 last_cc;
	bool cc_valid;
	s64 cc_fraction;
	int last_full_mah;
	bool initialized;
	bool full;
	bool full_anchor; /* a qualified/saved 100%, independent of cable state */
	bool zero;
	bool reset_done;
	u64 full_since;
	u64 settle_until;
	u64 suspend_boot_ns;
	u64 suspend_mono_ns;
	unsigned int suspend_irq_events;
	bool suspend_online;
	bool suspend_valid;
	s64 sleep_fraction;
	u64 last_sleep_sec;
	s32 last_sleep_cc_mas;
	bool used_sleep_model;
	u64 last_sample_sec;
	bool critical_irq; /* stock critical_low_flag, independent of CC history */
	bool critical_pending;
	u64 critical_since;
	unsigned int cc_discontinuities;
	struct ridi_stock_sample last;
};

static int ridi_stock_ocv_uv(int index)
{
	unsigned int adc = (ridi_stock_profile[index * 2] << 8) |
			   ridi_stock_profile[index * 2 + 1];

	/* Preserve stock integer operation order, 100 uV resolution. */
	return adc * 50000 / 4095 * 100;
}

static int ridi_stock_interpolate(int voltage_uv)
{
	int i, low, high;

	if (voltage_uv <= ridi_stock_ocv_uv(0))
		return 0;
	for (i = 1; i <= 10; i++) {
		low = ridi_stock_ocv_uv(i - 1);
		high = ridi_stock_ocv_uv(i);
		if (voltage_uv < high)
			return (i - 1) * 1000 +
				(voltage_uv - low) * 1000 / (high - low);
	}
	return 10000;
}

static int ridi_stock_voltage_soc(const struct ridi_stock_sample *s)
{
	int soc;

	/* LP-TFT branch in stock calc_soc_by_voltageMethod. */
	if (s->voltage_uv > 4100000)
		soc = 10000;
	else if (s->voltage_uv <= 3650000)
		soc = 0;
	else
		soc = ridi_stock_interpolate(s->voltage_uv);
	if (s->complete && s->online && s->current_ma >= -6 &&
	    s->voltage_uv - s->current_ma * RIDI_STOCK_R_MOHM > ridi_stock_ocv_uv(9))
		return 10000;
	return min(soc, 9900) / 100 * 100;
}

static int ridi_stock_temperature_soc(int soc, int temp_decic)
{
	int temperature = temp_decic / 10;
	int nt;

	if (temperature >= 25)
		nt = 0;
	else if (temperature >= 5)
		nt = (25 - temperature) * 5 * 625 / 100;
	else
		nt = 625 + (5 - temperature) * 6 * 625 / 100;
	return clamp(soc * 10000 / (10000 - nt), 0, 10000);
}

static int ridi_stock_cc_soc(s32 cc_mas, int full_mah, s64 *fraction)
{
	s64 numerator = (s64)cc_mas * 10000 + *fraction;
	int denominator = full_mah * 3600;
	int delta = div_s64(numerator, denominator);

	*fraction = numerator - (s64)delta * denominator;
	return delta;
}

static bool ridi_stock_qualified_full(const struct ridi_stock_sample *s)
{
	int ocv = s->voltage_uv - s->current_ma * RIDI_STOCK_R_MOHM;

	return s->online && s->complete && s->current_ma >= -6 &&
		ocv > ridi_stock_ocv_uv(9);
}

static void ridi_stock_rebase(struct ridi_stock_soc *soc,
			      const struct ridi_stock_sample *s)
{
	soc->last_cc = s->cc;
	soc->cc_valid = true;
	soc->cc_fraction = 0;
	soc->last_full_mah = s->full_mah;
}

/* Invalid/reset CC gets a new origin without counting its discontinuity. */
static int ridi_stock_delta(struct ridi_stock_soc *soc,
			    const struct ridi_stock_sample *s, u64 now)
{
	s32 delta = (s32)(s->cc - soc->last_cc);
	u64 elapsed = now >= soc->last_sample_sec ? now - soc->last_sample_sec : 0;
	u64 limit = min_t(u64, max_t(u64, elapsed, 1), 86400) * 1500;

	if (!soc->cc_valid || soc->last_full_mah != s->full_mah ||
	    delta > s->full_mah * 3600 || delta < -s->full_mah * 3600 ||
	    (s64)delta > (s64)limit || -(s64)delta > (s64)limit) {
		if (soc->cc_valid)
			soc->cc_discontinuities++;
		ridi_stock_rebase(soc, s);
		return 0;
	}
	soc->last_cc = s->cc;
	return ridi_stock_cc_soc(delta, s->full_mah, &soc->cc_fraction);
}

static void ridi_stock_start(struct ridi_stock_soc *soc,
			     const struct ridi_stock_sample *s,
			     int saved_percent, bool saved_recent)
{
	s64 fraction = 0;
	s32 cc = (s32)s->cc;

	if (saved_recent && saved_percent >= 1 && saved_percent <= 100 &&
	    cc <= s->full_mah * 3600 && cc >= -s->full_mah * 3600)
		soc->dsoc = clamp(saved_percent * 100 +
			ridi_stock_cc_soc(cc, s->full_mah, &fraction), 0, 10000);
	else
		soc->dsoc = ridi_stock_voltage_soc(s);
	soc->integrated = soc->dsoc;
	/* Stock restores its full flag from recent PSWR=100 and small CC loss.
	 * A voltage-only startup estimate is never a qualified full anchor.
	 */
	soc->full_anchor = saved_recent && saved_percent == 100 &&
		soc->dsoc >= 9950;
	soc->initialized = true;
	ridi_stock_rebase(soc, s);
}

static void ridi_stock_finish(struct ridi_stock_soc *soc,
			      const struct ridi_stock_sample *s, u64 now)
{
	if (ridi_stock_qualified_full(s)) {
		if (!soc->full)
			soc->full_since = now;
		soc->full = true;
		soc->full_anchor = true;
		soc->zero = false;
		soc->integrated = 10000;
		soc->dsoc = 10000;
		ridi_stock_rebase(soc, s);
	} else {
		/* Keep the full anchor after detach; NEVER revert to raw SOC. */
		soc->full = false;
		soc->full_since = 0;
		soc->reset_done = false;
		soc->dsoc = clamp(soc->integrated, 0, 10000);
		/* Losing CHG_COMPLETE (detach/recharge) is not charge consumption.
		 * Retire an established full anchor only after real accounting loss.
		 * Without an anchor, keep BOTH current directions below rounded 100:
		 * lifting a charging-only cap on detach invented the extra percent.
		 */
		if (soc->dsoc < 9950)
			soc->full_anchor = false;
		if (!soc->full_anchor && soc->dsoc >= 9950)
			soc->dsoc = 9949;
	}
	/* Stock zero latch + 1% reserve. A raw gauge zero alone is insufficient. */
	if (soc->zero && (s->current_ma >= 0 ||
			s->voltage_uv >= RIDI_STOCK_CUTOFF_UV + 100000))
		soc->zero = false;
	if (soc->zero)
		soc->dsoc = 0;
	else if (soc->dsoc < 50) {
		if (s->current_ma < 0 && s->voltage_uv < RIDI_STOCK_CUTOFF_UV) {
			soc->dsoc = 0;
			soc->zero = true;
		} else {
			soc->dsoc = 100;
		}
	}
	/* Awake fallback for the stock 3.55V comparator. Match stock's
	 * external-power and >10% false-critical guards; the IRQ path below
	 * handles suspend without a polling wake timer.
	 */
	if (!s->online && s->current_ma < 0 && s->voltage_uv <= 3550000 &&
	    (soc->dsoc + 50) / 100 <= 10) {
		if (!soc->critical_pending) {
			soc->critical_pending = true;
			soc->critical_since = now;
		} else if (now - soc->critical_since >= 10) {
			soc->integrated = 0;
			soc->dsoc = 0;
			soc->full_anchor = false;
			soc->zero = true;
		}
	} else {
		soc->critical_pending = false;
	}
	/* Stock keeps the IRQ flag until power returns or the underlying
	 * displayed estimate is >10%. A checkpoint/re-sample cannot erase it. */
	if (soc->critical_irq) {
		if (s->online || (soc->integrated + 50) / 100 > 10) {
			soc->critical_irq = false;
			soc->zero = false;
			soc->dsoc = clamp(soc->integrated, 0, 10000);
		} else {
			soc->dsoc = 0;
			soc->zero = true;
			soc->full_anchor = false;
		}
	}
	soc->last_sample_sec = now;
	soc->last = *s;
}

/* Stock critical IRQ guard: external power or >10% rejects false critical. */
static void ridi_stock_low_battery(struct ridi_stock_soc *soc,
				   const struct ridi_stock_sample *s)
{
	if (s->online || (soc->dsoc + 50) / 100 > 10)
		return;
	soc->critical_irq = true;
	soc->dsoc = 0;
	soc->full_anchor = false;
	soc->zero = true;
	soc->critical_pending = false;
}

static void ridi_stock_step(struct ridi_stock_soc *soc,
			    const struct ridi_stock_sample *s, u64 now)
{
	int delta = ridi_stock_delta(soc, s, now);

	soc->integrated = clamp(soc->integrated + delta, 0, 10000);
	/* Genuine low voltage cannot be concealed by a saved/full SOC anchor.
	 * Stock LOW_VOL decreases 1% per iteration below its cutoff. Apply that
	 * at the stock LOW_VOL_DOWN_TIME cadence in the caller, not at the
	 * faster USB poll rate.
	 */
	ridi_stock_finish(soc, s, now);
}

static void ridi_stock_sleep(struct ridi_stock_soc *soc,
			     const struct ridi_stock_sample *s,
			     u64 slept_sec, bool charger_event, u64 now)
{
	s32 cc = (s32)(s->cc - soc->last_cc);
	s64 charge, denominator = RIDI_STOCK_CAP_MAH * 360;
	int delta, voltage_soc;

	soc->last_sleep_sec = slept_sec;
	soc->last_sleep_cc_mas = cc;
	soc->used_sleep_model = false;
	if (slept_sec && !soc->suspend_online && !s->online &&
	    !charger_event && cc <= 0) {
		/* Stock E70Q1X low-current software integration. Use time actually
		 * spent suspended (boottime - monotonic), never screen-off time.
		 */
		charge = (s64)min_t(u64, slept_sec, 30 * 86400) *
			 RIDI_STOCK_SUSPEND_UA + soc->sleep_fraction;
		delta = div_s64(charge, (int)denominator);
		soc->sleep_fraction = charge - (s64)delta * denominator;
		soc->integrated = max(0, soc->integrated - delta);
		voltage_soc = ridi_stock_voltage_soc(s);
		/* Stock long-sleep fallback: >=24h and >=15% disagreement. */
		if (slept_sec >= 86400 && soc->integrated - voltage_soc >= 1500)
			soc->integrated = voltage_soc;
		ridi_stock_rebase(soc, s);
		soc->used_sleep_model = true;
		ridi_stock_finish(soc, s, now);
	} else {
		ridi_stock_step(soc, s, now);
	}
}

#endif
