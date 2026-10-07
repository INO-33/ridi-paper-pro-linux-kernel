/*
 * RTC timekeeping for the RC5T619 PMIC used by RIDI PAPER PRO.
 *
 * The register format follows the stock ricoh61x RTC driver: BCD time at
 * 0xa0..0xa6 and the century flag in bit 7 of the month register.
 *
 * Copyright (C) 2026 E70Q10 Android port
 * SPDX-License-Identifier: GPL-2.0
 */
#include <linux/bcd.h>
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

static const struct rtc_class_ops ricoh619_rtc_ops = {
	.read_time = ricoh619_read_time,
	.set_time = ricoh619_set_time,
};

static int ricoh619_rtc_probe(struct platform_device *pdev)
{
	struct rn5t618 *pmic = dev_get_drvdata(pdev->dev.parent);
	struct rtc_device *rtc;
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

	rtc = devm_rtc_device_register(&pdev->dev, "ricoh619-rtc",
				       &ricoh619_rtc_ops, THIS_MODULE);
	return PTR_ERR_OR_ZERO(rtc);
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
