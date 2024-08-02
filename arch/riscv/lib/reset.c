// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018, Bin Meng <bmeng.cn@gmail.com>
 */

#include <command.h>
#include <cpu_func.h>
#include <asm/io.h>

#define RESET_REG	((void *)0x51828300UL)
int do_reset(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	reset_cpu();
	writel(0x1AC0FFE6, RESET_REG);

	return 0;
}
