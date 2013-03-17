/*
 *  linux/arch/arm/mach-cns3xxx/clock.c
 *
 *  Copyright (c) 2008 Cavium Networks 
 * 
 *  This file is free software; you can redistribute it and/or modify 
 *  it under the terms of the GNU General Public License, Version 2, as 
 *  published by the Free Software Foundation. 
 *
 *  This file is distributed in the hope that it will be useful, 
 *  but AS-IS and WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE, TITLE, or 
 *  NONINFRINGEMENT.  See the GNU General Public License for more details. 
 *
 *  You should have received a copy of the GNU General Public License 
 *  along with this file; if not, write to the Free Software 
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA or 
 *  visit http://www.gnu.org/licenses/. 
 *
 *  This file may also be available under a different license from Cavium. 
 *  Contact Cavium Networks for more information
 */

#include <linux/module.h>
#include <linux/kernel.h>
//#include <linux/list.h>
//#include <linux/errno.h>
//#include <linux/err.h>
#include <linux/clk.h>
//#include <linux/mutex.h>
//
//#include <asm/hardware/icst307.h>
//
//#include "clock.h"
//
//static LIST_HEAD(clocks);
//static DEFINE_MUTEX(clocks_mutex);
//
//struct clk *clk_get(struct device *dev, const char *id)
//{
//	struct clk *p, *clk = ERR_PTR(-ENOENT);
//
//	mutex_lock(&clocks_mutex);
//	list_for_each_entry(p, &clocks, node) {
//		if (strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
//			clk = p;
//			break;
//		}
//	}
//	mutex_unlock(&clocks_mutex);
//
//	return clk;
//}
//EXPORT_SYMBOL(clk_get);
//
//void clk_put(struct clk *clk)
//{
//	module_put(clk->owner);
//}
//EXPORT_SYMBOL(clk_put);

int clk_enable(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

//unsigned long clk_get_rate(struct clk *clk)
//{
//	return clk->rate;
//}
//EXPORT_SYMBOL(clk_get_rate);
//
//long clk_round_rate(struct clk *clk, unsigned long rate)
//{
//	return rate;
//}
//EXPORT_SYMBOL(clk_round_rate);
//
//int clk_set_rate(struct clk *clk, unsigned long rate)
//{
//	int ret = -EIO;
//
//	if (clk->setvco) {
//		struct icst307_vco vco;
//
//		vco = icst307_khz_to_vco(clk->params, rate / 1000);
//		clk->rate = icst307_khz(clk->params, vco) * 1000;
//
//		printk("Clock %s: setting VCO reg params: S=%d R=%d V=%d\n",
//			clk->name, vco.s, vco.r, vco.v);
//
//		clk->setvco(clk, vco);
//		ret = 0;
//	}
//	return ret;
//}
//EXPORT_SYMBOL(clk_set_rate);
//
///*
// * These are fixed clocks.
// */
//static struct clk uart_clk = {
//	.name	= "UARTCLK",
//	.rate	= 24000000,
//};
//
//int clk_register(struct clk *clk)
//{
//	mutex_lock(&clocks_mutex);
//	list_add(&clk->node, &clocks);
//	mutex_unlock(&clocks_mutex);
//	return 0;
//}
//EXPORT_SYMBOL(clk_register);
//
//void clk_unregister(struct clk *clk)
//{
//	mutex_lock(&clocks_mutex);
//	list_del(&clk->node);
//	mutex_unlock(&clocks_mutex);
//}
//EXPORT_SYMBOL(clk_unregister);
//
//static int __init clk_init(void)
//{
//	clk_register(&uart_clk);
//	return 0;
//}
//arch_initcall(clk_init);
