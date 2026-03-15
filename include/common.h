/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Compatibility shim for vendor code that still includes <common.h>.
 * Upstream U-Boot removed this header; include the individual pieces instead.
 */

#ifndef __COMMON_H_
#define __COMMON_H_	1

#ifndef __ASSEMBLY__
#include <config.h>
#include <errno.h>
#include <time.h>
#include <linux/types.h>
#include <linux/string.h>
#include <stdarg.h>
#include <stdio.h>
#include <linux/kernel.h>
#include <asm/u-boot.h>
#include <vsprintf.h>
#endif

#ifdef DO_DEPS_ONLY
# include <env_internal.h>
#endif

#endif	/* __COMMON_H_ */
