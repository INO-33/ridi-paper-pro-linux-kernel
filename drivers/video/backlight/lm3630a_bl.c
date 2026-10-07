/*
* Simple driver for Texas Instruments LM3630A Backlight driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/platform_data/lm3630a_bl.h>

#define REG_CTRL	0x00
#define REG_BOOST	0x02
#define REG_CONFIG	0x01
#define REG_BRT_A	0x03
#define REG_BRT_B	0x04
#define REG_I_A		0x05
#define REG_I_B		0x06
#define REG_INT_STATUS	0x09
#define REG_INT_EN	0x0A
#define REG_FAULT	0x0B
#define REG_PWM_OUTLOW	0x12
#define REG_PWM_OUTHIGH	0x13
#define REG_REV		0x1F
#define REG_FILTER_STRENGTH	0x50
#define REG_MAX		REG_FILTER_STRENGTH

#define E70Q10_MAX_BRIGHTNESS	100
#define E70Q10_MAX_COLOR	10
#define E70Q10_CURRENT_A	4
#define E70Q10_CURRENT_B	7

#define INT_DEBOUNCE_MSEC	10
struct lm3630a_chip {
	struct device *dev;
	struct delayed_work work;

	int irq;
	struct workqueue_struct *irqthread;
	struct lm3630a_platform_data *pdata;
	struct backlight_device *bleda;
	struct backlight_device *bledb;
	struct backlight_device *bled;
	struct regmap *regmap;
	struct pwm_device *pwmd;
	struct mutex lock;
	u8 color;
	u8 current_a;
	u8 current_b;
};

/*
 * Verified channel-A curve from the public Netronix/Tolino Epos driver.
 * Keep the stock current limits (A=4, B=7).  Exact optical calibration can
 * later replace only the two-colour mix table.
 */
static const u8 e70q10_brightness_lut[E70Q10_MAX_BRIGHTNESS + 1] = {
	0,
	43, 66, 79, 90, 96, 107, 114, 120, 126, 132,
	135, 139, 143, 145, 147, 150, 152, 154, 156, 158,
	160, 163, 165, 168, 170, 172, 174, 176, 178, 180,
	181, 183, 185, 186, 188, 190, 191, 192, 193, 194,
	195, 196, 197, 198, 199, 200, 201, 202, 203, 204,
	205, 206, 207, 208, 209, 210, 211, 212, 213, 214,
	215, 216, 217, 218, 219, 220, 221, 222, 223, 224,
	225, 226, 227, 228, 229, 230, 231, 232, 233, 234,
	235, 236, 237, 238, 239, 240, 241, 242, 243, 244,
	245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
};

/* i2c access */
static int lm3630a_read(struct lm3630a_chip *pchip, unsigned int reg)
{
	int rval;
	unsigned int reg_val;

	rval = regmap_read(pchip->regmap, reg, &reg_val);
	if (rval < 0)
		return rval;
	return reg_val & 0xFF;
}

static int lm3630a_write(struct lm3630a_chip *pchip,
			 unsigned int reg, unsigned int data)
{
	return regmap_write(pchip->regmap, reg, data);
}

static int lm3630a_update(struct lm3630a_chip *pchip,
			  unsigned int reg, unsigned int mask,
			  unsigned int data)
{
	return regmap_update_bits(pchip->regmap, reg, mask, data);
}

/* initialize chip */
static int lm3630a_chip_init(struct lm3630a_chip *pchip)
{
	int rval;

	usleep_range(1000, 2000);
	/*
	 * Values captured from an untouched RIDI 4.4.2 boot.  Keep both LED
	 * banks disabled during probe; userspace explicitly turns the light on.
	 */
	rval = lm3630a_write(pchip, REG_FILTER_STRENGTH, 0x03);
	rval |= lm3630a_write(pchip, REG_CONFIG, 0x18);
	rval |= lm3630a_write(pchip, REG_BOOST, 0x63);
	rval |= lm3630a_write(pchip, REG_I_A, 0);
	rval |= lm3630a_write(pchip, REG_I_B, 0);
	rval |= lm3630a_write(pchip, REG_BRT_A, 0);
	rval |= lm3630a_write(pchip, REG_BRT_B, 0);
	rval |= lm3630a_write(pchip, REG_CTRL, 0);

	if (rval < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");
	return rval;
}

/* interrupt handling */
static void lm3630a_delayed_func(struct work_struct *work)
{
	int rval;
	struct lm3630a_chip *pchip;

	pchip = container_of(work, struct lm3630a_chip, work.work);

	rval = lm3630a_read(pchip, REG_INT_STATUS);
	if (rval < 0) {
		dev_err(pchip->dev,
			"i2c failed to access REG_INT_STATUS Register\n");
		return;
	}

	dev_info(pchip->dev, "REG_INT_STATUS Register is 0x%x\n", rval);
}

static irqreturn_t lm3630a_isr_func(int irq, void *chip)
{
	int rval;
	struct lm3630a_chip *pchip = chip;
	unsigned long delay = msecs_to_jiffies(INT_DEBOUNCE_MSEC);

	queue_delayed_work(pchip->irqthread, &pchip->work, delay);

	rval = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (rval < 0) {
		dev_err(pchip->dev, "i2c failed to access register\n");
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}

static int lm3630a_intr_config(struct lm3630a_chip *pchip)
{
	int rval;

	rval = lm3630a_write(pchip, REG_INT_EN, 0x87);
	if (rval < 0)
		return rval;

	INIT_DELAYED_WORK(&pchip->work, lm3630a_delayed_func);
	pchip->irqthread = create_singlethread_workqueue("lm3630a-irqthd");
	if (!pchip->irqthread) {
		dev_err(pchip->dev, "create irq thread fail\n");
		return -ENOMEM;
	}
	if (request_threaded_irq
	    (pchip->irq, NULL, lm3630a_isr_func,
	     IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "lm3630a_irq", pchip)) {
		dev_err(pchip->dev, "request threaded irq fail\n");
		destroy_workqueue(pchip->irqthread);
		return -ENOMEM;
	}
	return rval;
}

static void lm3630a_pwm_ctrl(struct lm3630a_chip *pchip, int br, int br_max)
{
	unsigned int period = pchip->pdata->pwm_period;
	unsigned int duty = br * period / br_max;

	pwm_config(pchip->pwmd, duty, period);
	if (duty)
		pwm_enable(pchip->pwmd);
	else
		pwm_disable(pchip->pwmd);
}

/* update and get brightness */
static int lm3630a_bank_a_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630a_chip *pchip = bl_get_data(bl);
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	/* pwm control */
	if ((pwm_ctrl & LM3630A_PWM_BANK_A) != 0) {
		lm3630a_pwm_ctrl(pchip, bl->props.brightness,
				 bl->props.max_brightness);
		return bl->props.brightness;
	}

	/* disable sleep */
	ret = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (ret < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
	/* minimum brightness is 0x04 */
	ret = lm3630a_write(pchip, REG_BRT_A, bl->props.brightness);
	if (bl->props.brightness < 0x4)
		ret |= lm3630a_update(pchip, REG_CTRL, LM3630A_LEDA_ENABLE, 0);
	else
		ret |= lm3630a_update(pchip, REG_CTRL,
				      LM3630A_LEDA_ENABLE, LM3630A_LEDA_ENABLE);
	if (ret < 0)
		goto out_i2c_err;
	return bl->props.brightness;

out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access\n");
	return bl->props.brightness;
}

static int lm3630a_bank_a_get_brightness(struct backlight_device *bl)
{
	int brightness, rval;
	struct lm3630a_chip *pchip = bl_get_data(bl);
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if ((pwm_ctrl & LM3630A_PWM_BANK_A) != 0) {
		rval = lm3630a_read(pchip, REG_PWM_OUTHIGH);
		if (rval < 0)
			goto out_i2c_err;
		brightness = (rval & 0x01) << 8;
		rval = lm3630a_read(pchip, REG_PWM_OUTLOW);
		if (rval < 0)
			goto out_i2c_err;
		brightness |= rval;
		goto out;
	}

	/* disable sleep */
	rval = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (rval < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
	rval = lm3630a_read(pchip, REG_BRT_A);
	if (rval < 0)
		goto out_i2c_err;
	brightness = rval;

out:
	bl->props.brightness = brightness;
	return bl->props.brightness;
out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return 0;
}

static const struct backlight_ops lm3630a_bank_a_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630a_bank_a_update_status,
	.get_brightness = lm3630a_bank_a_get_brightness,
};

/* update and get brightness */
static int lm3630a_bank_b_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630a_chip *pchip = bl_get_data(bl);
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	/* pwm control */
	if ((pwm_ctrl & LM3630A_PWM_BANK_B) != 0) {
		lm3630a_pwm_ctrl(pchip, bl->props.brightness,
				 bl->props.max_brightness);
		return bl->props.brightness;
	}

	/* disable sleep */
	ret = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (ret < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
	/* minimum brightness is 0x04 */
	ret = lm3630a_write(pchip, REG_BRT_B, bl->props.brightness);
	if (bl->props.brightness < 0x4)
		ret |= lm3630a_update(pchip, REG_CTRL, LM3630A_LEDB_ENABLE, 0);
	else
		ret |= lm3630a_update(pchip, REG_CTRL,
				      LM3630A_LEDB_ENABLE, LM3630A_LEDB_ENABLE);
	if (ret < 0)
		goto out_i2c_err;
	return bl->props.brightness;

out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access REG_CTRL\n");
	return bl->props.brightness;
}

static int lm3630a_bank_b_get_brightness(struct backlight_device *bl)
{
	int brightness, rval;
	struct lm3630a_chip *pchip = bl_get_data(bl);
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if ((pwm_ctrl & LM3630A_PWM_BANK_B) != 0) {
		rval = lm3630a_read(pchip, REG_PWM_OUTHIGH);
		if (rval < 0)
			goto out_i2c_err;
		brightness = (rval & 0x01) << 8;
		rval = lm3630a_read(pchip, REG_PWM_OUTLOW);
		if (rval < 0)
			goto out_i2c_err;
		brightness |= rval;
		goto out;
	}

	/* disable sleep */
	rval = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (rval < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
	rval = lm3630a_read(pchip, REG_BRT_B);
	if (rval < 0)
		goto out_i2c_err;
	brightness = rval;

out:
	bl->props.brightness = brightness;
	return bl->props.brightness;
out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return 0;
}

static const struct backlight_ops lm3630a_bank_b_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630a_bank_b_update_status,
	.get_brightness = lm3630a_bank_b_get_brightness,
};

static int e70q10_frontlight_apply(struct lm3630a_chip *pchip,
				   unsigned int brightness)
{
	unsigned int level_a, level_b;
	u8 duty_a = 0, duty_b = 0, ctrl = 0;
	int ret;

	if (brightness > E70Q10_MAX_BRIGHTNESS)
		brightness = E70Q10_MAX_BRIGHTNESS;

	if (brightness) {
		/*
		 * Split the requested *level* between the two banks instead of
		 * splitting the PWM code.
		 *
		 * e70q10_brightness_lut[] is a strongly compressive curve
		 * (level 1 already maps to code 43), so scaling the code by the
		 * colour weight removes far more light than the weight implies.
		 * Mixing in the code domain therefore made every colour-
		 * temperature change look like a brightness change, with the
		 * largest loss in the middle of the colour range.
		 *
		 * Because level_a + level_b == brightness for every colour
		 * value, the combined output now stays on the level the user
		 * asked for, and both end points keep their stock single-bank
		 * code.  The per-bank current limits stay at their calibrated
		 * values, so the mix only shifts colour temperature.
		 */
		level_b = DIV_ROUND_CLOSEST(brightness * pchip->color,
					    E70Q10_MAX_COLOR);
		level_a = brightness - level_b;

		if (level_a) {
			duty_a = e70q10_brightness_lut[level_a];
			ctrl |= LM3630A_LEDA_ENABLE;
		}
		if (level_b) {
			duty_b = e70q10_brightness_lut[level_b];
			ctrl |= LM3630A_LEDB_ENABLE;
		}
	}

	mutex_lock(&pchip->lock);
	ret = lm3630a_update(pchip, REG_CTRL, 0x80, 0);
	if (!ret)
		usleep_range(1000, 2000);
	if (!ret)
		ret = lm3630a_write(pchip, REG_BRT_A, duty_a);
	if (!ret)
		ret = lm3630a_write(pchip, REG_BRT_B, duty_b);
	if (!ret)
		ret = lm3630a_write(pchip, REG_I_A,
				    duty_a ? pchip->current_a : 0);
	if (!ret)
		ret = lm3630a_write(pchip, REG_I_B,
				    duty_b ? pchip->current_b : 0);
	if (!ret)
		ret = lm3630a_update(pchip, REG_CTRL,
				    LM3630A_LEDA_ENABLE |
				    LM3630A_LEDB_ENABLE, ctrl);
	mutex_unlock(&pchip->lock);

	return ret;
}

static int e70q10_frontlight_update_status(struct backlight_device *bl)
{
	struct lm3630a_chip *pchip = bl_get_data(bl);
	unsigned int brightness = bl->props.brightness;
	int ret;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & (BL_CORE_FBBLANK | BL_CORE_SUSPENDED))
		brightness = 0;

	ret = e70q10_frontlight_apply(pchip, brightness);
	return ret < 0 ? ret : 0;
}

static int e70q10_frontlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops e70q10_frontlight_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = e70q10_frontlight_update_status,
	.get_brightness = e70q10_frontlight_get_brightness,
};

static ssize_t color_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct lm3630a_chip *pchip = bl_get_data(bl);

	return scnprintf(buf, PAGE_SIZE, "%u\n", pchip->color);
}

static ssize_t color_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct lm3630a_chip *pchip = bl_get_data(bl);
	unsigned int color;
	int ret;

	ret = kstrtouint(buf, 0, &color);
	if (ret)
		return ret;
	if (color > E70Q10_MAX_COLOR)
		return -ERANGE;

	pchip->color = color;
	ret = backlight_update_status(bl);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_RW(color);

static ssize_t max_color_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", E70Q10_MAX_COLOR);
}
static DEVICE_ATTR_RO(max_color);

static int lm3630a_backlight_register(struct lm3630a_chip *pchip)
{
	struct backlight_properties props;
	struct lm3630a_platform_data *pdata = pchip->pdata;
	int ret;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	if (pdata->leda_ctrl != LM3630A_LEDA_DISABLE) {
		props.brightness = pdata->leda_init_brt;
		props.max_brightness = pdata->leda_max_brt;
		pchip->bleda =
		    devm_backlight_device_register(pchip->dev, "lm3630a_leda",
						   pchip->dev, pchip,
						   &lm3630a_bank_a_ops, &props);
		if (IS_ERR(pchip->bleda))
			return PTR_ERR(pchip->bleda);
	}

	if ((pdata->ledb_ctrl != LM3630A_LEDB_DISABLE) &&
	    (pdata->ledb_ctrl != LM3630A_LEDB_ON_A)) {
		props.brightness = pdata->ledb_init_brt;
		props.max_brightness = pdata->ledb_max_brt;
		pchip->bledb =
		    devm_backlight_device_register(pchip->dev, "lm3630a_ledb",
						   pchip->dev, pchip,
						   &lm3630a_bank_b_ops, &props);
		if (IS_ERR(pchip->bledb))
			return PTR_ERR(pchip->bledb);
	}

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = E70Q10_MAX_BRIGHTNESS;
	props.brightness = 0;
	pchip->bled = devm_backlight_device_register(pchip->dev,
						     "lm3630a_led",
						     pchip->dev, pchip,
						     &e70q10_frontlight_ops,
						     &props);
	if (IS_ERR(pchip->bled))
		return PTR_ERR(pchip->bled);

	ret = device_create_file(&pchip->bled->dev, &dev_attr_color);
	if (ret)
		return ret;
	ret = device_create_file(&pchip->bled->dev, &dev_attr_max_color);
	if (ret)
		device_remove_file(&pchip->bled->dev, &dev_attr_color);
	if (ret)
		return ret;

	return 0;
}

static const struct regmap_config lm3630a_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static int lm3630a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct lm3630a_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm3630a_chip *pchip;
	int rval;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check\n");
		return -EOPNOTSUPP;
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3630a_chip),
			     GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;
	pchip->dev = &client->dev;
	mutex_init(&pchip->lock);
	pchip->current_a = E70Q10_CURRENT_A;
	pchip->current_b = E70Q10_CURRENT_B;
	pchip->color = 0;

	of_property_read_u8(client->dev.of_node, "ridi,current-a",
			    &pchip->current_a);
	of_property_read_u8(client->dev.of_node, "ridi,current-b",
			    &pchip->current_b);
	if (pchip->current_a > 0x1f || pchip->current_b > 0x1f) {
		dev_err(&client->dev,
			"frontlight current exceeds LM3630A limit\n");
		return -EINVAL;
	}

	pchip->regmap = devm_regmap_init_i2c(client, &lm3630a_regmap);
	if (IS_ERR(pchip->regmap)) {
		rval = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "fail : allocate reg. map: %d\n", rval);
		return rval;
	}

	i2c_set_clientdata(client, pchip);
	if (pdata == NULL) {
		pdata = devm_kzalloc(pchip->dev,
				     sizeof(struct lm3630a_platform_data),
				     GFP_KERNEL);
		if (pdata == NULL)
			return -ENOMEM;
		/* default values */
		pdata->leda_ctrl = LM3630A_LEDA_ENABLE;
		pdata->ledb_ctrl = LM3630A_LEDB_ENABLE;
		pdata->leda_max_brt = LM3630A_MAX_BRIGHTNESS;
		pdata->ledb_max_brt = LM3630A_MAX_BRIGHTNESS;
		pdata->leda_init_brt = 0;
		pdata->ledb_init_brt = 0;
	}
	pchip->pdata = pdata;

	/* chip initialize */
	rval = lm3630a_chip_init(pchip);
	if (rval < 0) {
		dev_err(&client->dev, "fail : init chip\n");
		return rval;
	}
	/* backlight register */
	rval = lm3630a_backlight_register(pchip);
	if (rval < 0) {
		dev_err(&client->dev, "fail : backlight register.\n");
		return rval;
	}
	/* pwm */
	if (pdata->pwm_ctrl != LM3630A_PWM_DISABLE) {
		pchip->pwmd = devm_pwm_get(pchip->dev, "lm3630a-pwm");
		if (IS_ERR(pchip->pwmd)) {
			dev_err(&client->dev, "fail : get pwm device\n");
			return PTR_ERR(pchip->pwmd);
		}

		/*
		 * FIXME: pwm_apply_args() should be removed when switching to
		 * the atomic PWM API.
		 */
		pwm_apply_args(pchip->pwmd);
	}

	/* interrupt enable  : irq 0 is not allowed */
	pchip->irq = client->irq;
	if (pchip->irq) {
		rval = lm3630a_intr_config(pchip);
		if (rval < 0)
			return rval;
	}
	dev_info(&client->dev, "LM3630A backlight register OK.\n");
	return 0;
}

static int lm3630a_remove(struct i2c_client *client)
{
	int rval;
	struct lm3630a_chip *pchip = i2c_get_clientdata(client);

	e70q10_frontlight_apply(pchip, 0);
	rval = lm3630a_write(pchip, REG_BRT_A, 0);
	if (rval < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");

	rval = lm3630a_write(pchip, REG_BRT_B, 0);
	if (rval < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");

	if (pchip->bled) {
		device_remove_file(&pchip->bled->dev, &dev_attr_max_color);
		device_remove_file(&pchip->bled->dev, &dev_attr_color);
	}

	if (pchip->irq) {
		free_irq(pchip->irq, pchip);
		flush_workqueue(pchip->irqthread);
		destroy_workqueue(pchip->irqthread);
	}
	return 0;
}

static void lm3630a_shutdown(struct i2c_client *client)
{
	struct lm3630a_chip *pchip = i2c_get_clientdata(client);

	e70q10_frontlight_apply(pchip, 0);
}

static const struct i2c_device_id lm3630a_id[] = {
	{LM3630A_NAME, 0},
	{"lm3630a", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3630a_id);

static const struct of_device_id lm3630a_of_match[] = {
	{ .compatible = "ti,lm3630a" },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3630a_of_match);

static struct i2c_driver lm3630a_i2c_driver = {
	.driver = {
		   .name = LM3630A_NAME,
		   .of_match_table = lm3630a_of_match,
		   },
	.probe = lm3630a_probe,
	.remove = lm3630a_remove,
	.shutdown = lm3630a_shutdown,
	.id_table = lm3630a_id,
};

module_i2c_driver(lm3630a_i2c_driver);

MODULE_DESCRIPTION("Texas Instruments Backlight driver for LM3630A");
MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("LDD MLP <ldd-mlp@list.ti.com>");
MODULE_LICENSE("GPL v2");
