/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_REGULATOR_TPS65185_H
#define _LINUX_REGULATOR_TPS65185_H

#include <linux/errno.h>
#include <linux/kconfig.h>

struct regulator;

/* These helpers require the TPS65185 DISPLAY supply. */
#if IS_REACHABLE(CONFIG_REGULATOR_TPS65185)
int tps65185_read_temperature(struct regulator *display, int *temperature);
/* Caller must first release its DISPLAY vote after EPDC hardware is idle. */
int tps65185_flush_powerdown(struct regulator *display);
#else
static inline int tps65185_read_temperature(struct regulator *display,
					  int *temperature)
{
	return -EOPNOTSUPP;
}
static inline int tps65185_flush_powerdown(struct regulator *display)
{
	return -EOPNOTSUPP;
}
#endif
#endif
