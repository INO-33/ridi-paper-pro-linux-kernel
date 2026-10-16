/*
 * RTC timekeeping and alarm wake for the RC5T619 PMIC used by RIDI PAPER PRO.
 *
 * The register format follows the stock ricoh61x RTC driver: BCD time at
 * 0xa0..0xa6 and the century flag in bit 7 of the month register.
 *
 * Copyright (C) 2026 E70Q10 Android port
 * SPDX-License-Identifier: GPL-2.0
 */
#include <linux/bcd.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/err.h>
#include <linux/mfd/rn5t618.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/rtc.h>

#define RICOH619_RTC_SECONDS    0xa0
#define RICOH619_RTC_CTRL1      0xae
#define RICOH619_RTC_CTRL2      0xaf
#define RICOH619_RTC_24H        0x20
#define RICOH619_RTC_PON        0x10
#define RICOH619_RTC_ALARM      0xa8
#define RICOH619_RTC_DALE       0x40
#define RICOH619_RTC_DAF        0x01

struct ricoh619_rtc {
	struct rn5t618 *pmic;
	struct rtc_device *rtc;
	struct mutex lock;
	int irq;
	bool irq_fault;
};

static int ricoh619_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rn5t618 *pmic = dev_get_drvdata(dev->parent);
	unsigned int ctrl2;
	u8 data[7];
	int ret;

	ret = regmap_read(pmic->regmap, RICOH619_RTC_CTRL2, &ctrl2);
	if (ret)
		return ret;
	/* Stock treats PON as a lost PMIC RTC clock, not a valid date. */
	if (ctrl2 & RICOH619_RTC_PON)
		return -EINVAL;

	ret = regmap_bulk_read(pmic->regmap, RICOH619_RTC_SECONDS,
			       data, sizeof(data));
	if (ret)
		return ret;

	tm->tm_sec = bcd2bin(data[0] & 0x7f);
	tm->tm_min = bcd2bin(data[1] & 0x7f);
	tm->tm_hour = bcd2bin(data[2] & 0x3f);
	tm->tm_wday = bcd2bin(data[3] & 0x07);
	tm->tm_mday = bcd2bin(data[4] & 0x3f);
	tm->tm_mon = bcd2bin(data[5] & 0x1f) - 1;
	tm->tm_year = bcd2bin(data[6]) + ((data[5] & 0x80) ? 100 : 0);

	if (tm->tm_year < 70 || tm->tm_year > 199)
		return -EINVAL;
	return rtc_valid_tm(tm);
}

static int ricoh619_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rn5t618 *pmic = dev_get_drvdata(dev->parent);
	u8 data[7];
	int ret;

	if (rtc_valid_tm(tm) || tm->tm_year < 70 || tm->tm_year > 199)
		return -EINVAL;

	data[0] = bin2bcd(tm->tm_sec);
	data[1] = bin2bcd(tm->tm_min);
	data[2] = bin2bcd(tm->tm_hour);
	data[3] = bin2bcd(tm->tm_wday);
	data[4] = bin2bcd(tm->tm_mday);
	data[5] = bin2bcd(tm->tm_mon + 1);
	if (tm->tm_year >= 100)
		data[5] |= 0x80;
	data[6] = bin2bcd(tm->tm_year % 100);

	ret = regmap_bulk_write(pmic->regmap, RICOH619_RTC_SECONDS,
				data, sizeof(data));
	if (ret)
		return ret;

	/* Clear stock PON/VDET status only after writing a valid time. */
	return regmap_update_bits(pmic->regmap, RICOH619_RTC_CTRL2,
				  0x7b, 0x20);
}


static int ricoh619_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct ricoh619_rtc *rtc = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&rtc->lock);
	ret = regmap_update_bits(rtc->pmic->regmap, RICOH619_RTC_CTRL1,
				 RICOH619_RTC_DALE, enabled ? RICOH619_RTC_DALE : 0);
	mutex_unlock(&rtc->lock);
	return ret;
}

static int ricoh619_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct ricoh619_rtc *rtc = dev_get_drvdata(dev);
	unsigned int month, ctrl1, ctrl2;
	u8 data[6];
	int ret;

	mutex_lock(&rtc->lock);
	ret = regmap_bulk_read(rtc->pmic->regmap, RICOH619_RTC_ALARM, data, 6);
	if (!ret)
		ret = regmap_read(rtc->pmic->regmap, 0xa5, &month);
	if (!ret)
		ret = regmap_read(rtc->pmic->regmap, RICOH619_RTC_CTRL1, &ctrl1);
	if (!ret)
		ret = regmap_read(rtc->pmic->regmap, RICOH619_RTC_CTRL2, &ctrl2);
	if (!ret) {
		memset(&alarm->time, 0, sizeof(alarm->time));
		alarm->time.tm_sec = bcd2bin(data[0] & 0x7f);
		alarm->time.tm_min = bcd2bin(data[1] & 0x7f);
		alarm->time.tm_hour = bcd2bin(data[2] & 0x3f);
		alarm->time.tm_mday = bcd2bin(data[3] & 0x3f);
		alarm->time.tm_mon = bcd2bin(data[4] & 0x1f) - 1;
		alarm->time.tm_year = bcd2bin(data[5]) + ((month & 0x80) ? 100 : 0);
		alarm->enabled = !!(ctrl1 & RICOH619_RTC_DALE);
		alarm->pending = !!(ctrl2 & RICOH619_RTC_DAF);
	}
	mutex_unlock(&rtc->lock);
	return ret;
}

static int ricoh619_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct ricoh619_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time now;
	u8 data[6];
	int ret;

	if (rtc_valid_tm(&alarm->time) || alarm->time.tm_year < 70 ||
	    alarm->time.tm_year > 199)
		return -EINVAL;
	mutex_lock(&rtc->lock);
	ret = ricoh619_read_time(dev, &now);
	/* The alarm shares the live RTC century bit; never silently wrap it. */
	if (!ret && now.tm_year / 100 != alarm->time.tm_year / 100)
		ret = -EINVAL;
	if (!ret)
		ret = regmap_update_bits(rtc->pmic->regmap, RICOH619_RTC_CTRL1,
					 RICOH619_RTC_DALE, 0);
	if (!ret)
		ret = regmap_update_bits(rtc->pmic->regmap, RICOH619_RTC_CTRL2, 0x81, 0);
	data[0] = bin2bcd(alarm->time.tm_sec);
	data[1] = bin2bcd(alarm->time.tm_min);
	data[2] = bin2bcd(alarm->time.tm_hour);
	data[3] = bin2bcd(alarm->time.tm_mday) | 0x80; /* stock DAL_EXT */
	data[4] = bin2bcd(alarm->time.tm_mon + 1);
	data[5] = bin2bcd(alarm->time.tm_year % 100);
	if (!ret)
		ret = regmap_bulk_write(rtc->pmic->regmap, RICOH619_RTC_ALARM, data, 6);
	if (!ret && alarm->enabled)
		ret = regmap_update_bits(rtc->pmic->regmap, RICOH619_RTC_CTRL1,
					 RICOH619_RTC_DALE, RICOH619_RTC_DALE);
	if (!ret && rtc->irq_fault) {
		rtc->irq_fault = false;
		enable_irq(rtc->irq);
	}
	mutex_unlock(&rtc->lock);
	return ret;
}

static irqreturn_t ricoh619_alarm_irq(int irq, void *arg)
{
	struct ricoh619_rtc *rtc = arg;
	unsigned int status;
	int ret;

	mutex_lock(&rtc->lock);
	ret = regmap_read(rtc->pmic->regmap, RICOH619_RTC_CTRL2, &status);
	if (!ret && (status & RICOH619_RTC_DAF)) {
		ret = regmap_update_bits(rtc->pmic->regmap, RICOH619_RTC_CTRL1,
					 RICOH619_RTC_DALE, 0);
		if (!ret)
			ret = regmap_update_bits(rtc->pmic->regmap, RICOH619_RTC_CTRL2, 0x81, 0);
		if (!ret) {
			pm_wakeup_event(rtc->rtc->dev.parent, 2000);
			rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);
		}
	}
	if (ret) {
		disable_irq_nosync(irq);
		rtc->irq_fault = true;
		dev_err(rtc->rtc->dev.parent, "RTC alarm IRQ failed: %d\n", ret);
	}
	mutex_unlock(&rtc->lock);
	return IRQ_HANDLED;
}

static void ricoh619_alarm_cleanup(void *arg)
{
	struct ricoh619_rtc *rtc = arg;
	disable_irq_wake(rtc->irq);
	device_init_wakeup(rtc->rtc->dev.parent, false);
}

static const struct rtc_class_ops ricoh619_time_only_ops = {
	.read_time = ricoh619_read_time,
	.set_time = ricoh619_set_time,
};

static const struct rtc_class_ops ricoh619_rtc_ops = {
	.read_time = ricoh619_read_time,
	.set_time = ricoh619_set_time,
	.read_alarm = ricoh619_read_alarm,
	.set_alarm = ricoh619_set_alarm,
	.alarm_irq_enable = ricoh619_alarm_irq_enable,
};

static int ricoh619_rtc_probe(struct platform_device *pdev)
{
	struct rn5t618 *pmic = dev_get_drvdata(pdev->dev.parent);
	struct rtc_device *rtc;
	struct ricoh619_rtc *data;
	struct rtc_time saved_time;
	struct rtc_time default_time = {
		.tm_sec = 0, .tm_min = 0, .tm_hour = 0,
		.tm_wday = 3, .tm_mday = 1, .tm_mon = 1, .tm_year = 112,
	};
	unsigned int ctrl2;
	int ret;

	if (!pmic || pmic->variant != RC5T619)
		return -ENODEV;

	/* Use stock 24-hour mode; preserve every other control bit. */
	ret = regmap_update_bits(pmic->regmap, RICOH619_RTC_CTRL1,
				 RICOH619_RTC_24H, RICOH619_RTC_24H);
	if (ret)
		return ret;

	ret = regmap_read(pmic->regmap, RICOH619_RTC_CTRL2, &ctrl2);
	if (ret)
		return ret;
	if ((ctrl2 & RICOH619_RTC_PON) ||
	    ricoh619_read_time(&pdev->dev, &saved_time)) {
		/* A lost or invalid PMIC clock gets stock's 2012 fallback. */
		ret = ricoh619_set_time(&pdev->dev, &default_time);
		if (ret)
			return ret;
		dev_warn(&pdev->dev, "RTC invalid; initialized stock fallback date\n");
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->pmic = pmic;
	data->irq = rc5t619_get_irq(pmic, RC5T619_IRQ_RTC_ALARM);
	mutex_init(&data->lock);
	platform_set_drvdata(pdev, data);
	if (data->irq < 0) {
		/* Non-opt-in boards retain the previous timekeeping-only RTC. */
		rtc = devm_rtc_device_register(&pdev->dev, "ricoh619-rtc",
					       &ricoh619_time_only_ops, THIS_MODULE);
		return PTR_ERR_OR_ZERO(rtc);
	}
	/* Only the day alarm is consumed here; mask inherited periodic modes. */
	ret = regmap_update_bits(pmic->regmap, RICOH619_RTC_CTRL1, 0x4f, 0);
	if (!ret)
		ret = regmap_update_bits(pmic->regmap, RICOH619_RTC_CTRL2, 0x85, 0);
	if (ret)
		return ret;
	ret = device_init_wakeup(&pdev->dev, true);
	if (ret)
		return ret;
	rtc = devm_rtc_device_register(&pdev->dev, "ricoh619-rtc",
				       &ricoh619_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		device_init_wakeup(&pdev->dev, false);
		return PTR_ERR(rtc);
	}
	data->rtc = rtc;
	ret = devm_request_threaded_irq(&pdev->dev, data->irq, NULL,
		ricoh619_alarm_irq, IRQF_ONESHOT, "rc5t619-rtc-alarm", data);
	if (ret)
		goto wake_cleanup;
	ret = enable_irq_wake(data->irq);
	if (ret)
		goto wake_cleanup;
	return devm_add_action_or_reset(&pdev->dev, ricoh619_alarm_cleanup, data);
wake_cleanup:
	device_init_wakeup(&pdev->dev, false);
	return ret;
}

static struct platform_driver ricoh619_rtc_driver = {
	.probe = ricoh619_rtc_probe,
	.driver = {
		.name = "ricoh619-rtc",
	},
};
module_platform_driver(ricoh619_rtc_driver);

MODULE_DESCRIPTION("RIDI PAPER PRO Ricoh RC5T619 RTC timekeeping");
MODULE_LICENSE("GPL");
