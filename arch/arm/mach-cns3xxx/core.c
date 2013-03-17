/*
 * Copyright 1999 - 2003 ARM Limited
 * Copyright 2000 Deep Blue Solutions Ltd
 * Copyright 2008 Cavium Networks
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/hardware/gic.h>
#include <mach/cns3xxx.h>
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include "core.h"

static struct map_desc cns3xxx_io_desc[] __initdata = {
#ifdef CONFIG_PAGE_SIZE_64K	
	{
		.virtual	= CNS3XXX_TC11MP_SCU_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_TC11MP_SCU_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	},
#else
	{
		.virtual	= CNS3XXX_TC11MP_TWD_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_TC11MP_TWD_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_TC11MP_GIC_CPU_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_TC11MP_GIC_CPU_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_TC11MP_GIC_DIST_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_TC11MP_GIC_DIST_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, 
#endif
	{
		.virtual	= CNS3XXX_TIMER1_2_3_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_TIMER1_2_3_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	},
#ifdef CONFIG_PAGE_SIZE_64K	
	{
		.virtual	= CNS3XXX_L2C_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_L2C_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	},
#else
	{
		.virtual	= CNS3XXX_L2C_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_L2C_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
#endif
	{
		.virtual	= CNS3XXX_GPIOA_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_GPIOA_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_GPIOB_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_GPIOB_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_MISC_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_MISC_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PM_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PM_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_SWITCH_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_SWITCH_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_SSP_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_SSP_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, 
#ifndef CONFIG_PAGE_SIZE_64K
	{
		.virtual	= CNS3XXX_PPE_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PPE_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_EMBEDDED_SRAM_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_EMBEDDED_SRAM_BASE),
		.length		= SZ_8K,
		.type		= MT_DEVICE,
	},
#endif 
	{ 
		.virtual	= CNS3XXX_PCIE0_MEM_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE0_MEM_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE0_HOST_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE0_HOST_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE0_CFG0_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE0_CFG0_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE0_CFG1_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE0_CFG1_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE0_MSG_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE0_MSG_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE0_IO_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE0_IO_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
        }, { 
		.virtual	= CNS3XXX_PCIE1_MEM_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE1_MEM_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE1_HOST_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE1_HOST_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE1_CFG0_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE1_CFG0_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE1_CFG1_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE1_CFG1_BASE),
		.length		= SZ_16M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE1_MSG_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE1_MSG_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_PCIE1_IO_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_PCIE1_IO_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
	}, {
		.virtual	= CNS3XXX_SATA2_BASE_VIRT,
		.pfn		= __phys_to_pfn(CNS3XXX_SATA2_BASE),
		.length		= PAGE_SIZE,
		.type		= MT_DEVICE,
        }, {
                .virtual        = CNS3XXX_USBOTG_BASE_VIRT,
                .pfn            = __phys_to_pfn(CNS3XXX_USBOTG_BASE),
                .length         = PAGE_SIZE,
                .type           = MT_DEVICE,
        }, {
                .virtual        = CNS3XXX_PM_BASE_VIRT,
                .pfn            = __phys_to_pfn(CNS3XXX_PM_BASE),
                .length         = PAGE_SIZE,
                .type           = MT_DEVICE,
        }, {
                .virtual        = CNS3XXX_CLCD_BASE_VIRT,
                .pfn            = __phys_to_pfn( CNS3XXX_CLCD_BASE),
                .length         = PAGE_SIZE,
                .type           = MT_DEVICE,
        }, {			
                .virtual        = CNS3XXX_DMAC_BASE_VIRT,
                .pfn            = __phys_to_pfn(CNS3XXX_DMAC_BASE),
                .length         = PAGE_SIZE,
                .type           = MT_DEVICE,

	}
};

void __init cns3xxx_map_io(void)
{
	iotable_init(cns3xxx_io_desc, ARRAY_SIZE(cns3xxx_io_desc));
}

/* used by entry-macro.S */
void __iomem *gic_cpu_base_addr;

void __init cns3xxx_init_irq(void)
{
	gic_cpu_base_addr = __io(CNS3XXX_TC11MP_GIC_CPU_BASE_VIRT);
	gic_dist_init(0, __io(CNS3XXX_TC11MP_GIC_DIST_BASE_VIRT), 29);
	gic_cpu_init(0, gic_cpu_base_addr);
}

void cns3xxx_power_off(void)
{
	u32 __iomem *pm_base = __io(CNS3XXX_PM_BASE_VIRT);
	u32 clkctrl;

	printk(KERN_INFO "powering system down...\n");

	clkctrl = readl(pm_base + PM_SYS_CLK_CTRL_OFFSET);
	clkctrl &= 0xfffff1ff;
	clkctrl |= (0x5 << 9);		/* Hibernate */
	writel(clkctrl, pm_base + PM_SYS_CLK_CTRL_OFFSET);

}

/*
 * CLCD support.
 */
#define SYS_CLCD_NLCDIOON       (1 << 2)
#define SYS_CLCD_VDDPOSSWITCH   (1 << 3)
#define SYS_CLCD_PWR3V5SWITCH   (1 << 4)
#define SYS_CLCD_ID_MASK        (0x1f << 8)
#define SYS_CLCD_ID_SANYO_3_8   (0x00 << 8)
#define SYS_CLCD_ID_UNKNOWN_8_4 (0x01 << 8)
#define SYS_CLCD_ID_EPSON_2_2   (0x02 << 8)
#define SYS_CLCD_ID_AMPIRE_4_0  (0x03 << 8)
#define SYS_CLCD_ID_SANYO_2_5   (0x07 << 8)
#define SYS_CLCD_ID_XGA         (0x08 << 8)
#define SYS_CLCD_ID_SVGA        (0x09 << 8)
#define SYS_CLCD_ID_INNOLUX_7_0 (0x0A << 8)
#define SYS_CLCD_ID_VGA         (0x1f << 8)

#ifdef CONFIG_CNS3XXX_DISP_7INCH
static struct clcd_panel innolux_7_0_in = {
		.mode           = {
                .name           = "Innolux 7.0",
                .refresh        = 60,
                .xres           = 800,
                .yres           = 480,
                .pixclock       = 30030,
                .left_margin    = 46,
                .right_margin   = 200,
                .upper_margin   = 23,
                .lower_margin   = 20,
                .hsync_len      = 10,
                .vsync_len      = 2,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED,
        },
        .width          = -1,
        .height         = -1,
        .tim2           = TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
        .cntl           = CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
        .bpp            = 16,
};
#endif

#ifdef CONFIG_CNS3XXX_DISP_XGA
static struct clcd_panel xga = {
		.mode           = {
                .name           = "XGA",
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .pixclock       = 15384,
                .left_margin    = 160,
                .right_margin   = 24,
                .upper_margin   = 29,
                .lower_margin   = 3,
                .hsync_len      = 136,
                .vsync_len      = 6,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED,
        },
        .width          = -1,
        .height         = -1,
        .tim2           = TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
        .cntl           = CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
        .bpp            = 16,
};
#endif

#ifdef CONFIG_CNS3XXX_DISP_SVGA
static struct clcd_panel svga = {
		.mode           = {
                .name           = "SVGA",
                .refresh        = 60,
                .xres           = 800,
                .yres           = 600,
                .pixclock       = 25000,
                .left_margin    = 88,
                .right_margin   = 40,
                .upper_margin   = 23,
                .lower_margin   = 1,
                .hsync_len      = 128,
                .vsync_len      = 4,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED,
        },
        .width          = -1,
        .height         = -1,
        .tim2           = TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
        .cntl           = CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
        .bpp            = 16,
};
#endif

#ifdef CONFIG_CNS3XXX_DISP_VGA
static struct clcd_panel vga = {
		.mode           = {
                .name           = "VGA",
                .refresh        = 60,
                .xres           = 640,
                .yres           = 480,
                .pixclock       = 39721,
                .left_margin    = 40,
                .right_margin   = 24,
                .upper_margin   = 32,
                .lower_margin   = 11,
                .hsync_len      = 96,
                .vsync_len      = 2,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED,
        },
        .width          = -1,
        .height         = -1,
        .tim2           = TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
        .cntl           = CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
        .bpp            = 32,
};
#endif

//#ifdef CONFIG_CNS3XXX_DISP_4INCH
static struct clcd_panel ampire_4_0_in = {
		.mode           = {
                .name           = "Ampire 4.0",
                .refresh        = 60,
                .xres           = 480,
                .yres           = 272,
                .pixclock       = 111111,
                .left_margin    = 6,
                .right_margin   = 6,
                .upper_margin   = 2,
                .lower_margin   = 2,
                .hsync_len      = 33,
                .vsync_len      = 10,
                .sync           = 0,
                .vmode          = FB_VMODE_NONINTERLACED,
        },
        .width          = -1,
        .height         = -1,
        .tim2           = TIM2_BCD | TIM2_IHS | TIM2_IVS | TIM2_IPC,
        .cntl           = CNTL_LCDTFT | CNTL_BGR | CNTL_LCDVCOMP(1),
        .bpp            = 32,

};
//#endif

/*
 * Detect which LCD panel is connected, and return the appropriate
 * clcd_panel structure.  Note: we do not have any information on
 * the required timings for the 8.4in panel, so we presently assume
 * VGA timings.
 */
static struct clcd_panel *cns3xxx_clcd_panel(void)
{
        /* void __iomem *sys_clcd = __io_address(CNS3XXX_SYS_BASE) + CNS3XXX_SYS_CLCD_OFFSET; */
        struct clcd_panel *panel = &ampire_4_0_in;

#if defined (CONFIG_CNS3XXX_DISP_4INCH)
        panel = &ampire_4_0_in;
#elif defined (CONFIG_CNS3XXX_DISP_7INCH)
        panel = &innolux_7_0_in;
#elif defined (CONFIG_CNS3XXX_DISP_VGA)
        panel = &vga;
#elif defined (CONFIG_CNS3XXX_DISP_SVGA)
        panel = &svga;
#elif defined (CONFIG_CNS3XXX_DISP_XGA)
        panel = &xga;
#else
        panel = &ampire_4_0_in;
#endif

        return panel;
}

/*
 * Disable all display connectors on the interface module.
 */
static void cns3xxx_clcd_disable(struct clcd_fb *fb)
{
/*
        void __iomem *sys_clcd = __io_address(CNS3XXX_SYS_BASE) + CNS3XXX_SYS_CLCD_OFFSET;
        u32 val;

        val = readl(sys_clcd);
        val &= ~SYS_CLCD_NLCDIOON | SYS_CLCD_PWR3V5SWITCH;
        writel(val, sys_clcd);
*/
}

/*
 * Enable the relevant connector on the interface module.
 */
static void cns3xxx_clcd_enable(struct clcd_fb *fb)
{
/*
        void __iomem *sys_clcd = __io_address(CNS3XXX_SYS_BASE) + CNS3XXX_SYS_CLCD_OFFSET;
        u32 val;
*/

        /*
         * Enable the PSUs
         */
/*
        val = readl(sys_clcd);
        val |= SYS_CLCD_NLCDIOON | SYS_CLCD_PWR3V5SWITCH;
        writel(val, sys_clcd);
*/
}

static unsigned long framesize = SZ_4M;

static int cns3xxx_clcd_setup(struct clcd_fb *fb)
{
        dma_addr_t dma;

        fb->panel               = cns3xxx_clcd_panel();

        fb->fb.screen_base = dma_alloc_writecombine(&fb->dev->dev, framesize,
                                                    &dma, GFP_KERNEL);
        if (!fb->fb.screen_base) {
                printk(KERN_ERR "CLCD: unable to map framebuffer\n");
                return -ENOMEM;
        }

        fb->fb.fix.smem_start   = dma;
        fb->fb.fix.smem_len     = framesize;

        return 0;
}

static int cns3xxx_clcd_mmap(struct clcd_fb *fb, struct vm_area_struct *vma)
{
        return dma_mmap_writecombine(&fb->dev->dev, vma,
                                     fb->fb.screen_base,
                                     fb->fb.fix.smem_start,
                                     fb->fb.fix.smem_len);
}

static void cns3xxx_clcd_remove(struct clcd_fb *fb)
{
        dma_free_writecombine(&fb->dev->dev, fb->fb.fix.smem_len,
                              fb->fb.screen_base, fb->fb.fix.smem_start);
}

struct clcd_board clcd_plat_data = {
        .name           = "CNS3XXX",
        .check          = clcdfb_check,
        .decode         = clcdfb_decode,
        .disable        = cns3xxx_clcd_disable,
        .enable         = cns3xxx_clcd_enable,
        .setup          = cns3xxx_clcd_setup,
        .mmap           = cns3xxx_clcd_mmap,
        .remove         = cns3xxx_clcd_remove,
};


/*
 * Timer
 */
static void __iomem *cns3xxx_tmr1;

static void cns3xxx_timer_set_mode(enum clock_event_mode mode,
				   struct clock_event_device *clk)
{
	unsigned long ctrl = readl(cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);
	int pclk = cns3xxx_cpu_clock() / 8;
	int reload;

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		reload = pclk * 20 / (3 * HZ) * 0x25000;
		writel(reload, cns3xxx_tmr1 + TIMER1_AUTO_RELOAD_OFFSET);
		ctrl |= (1 << 0) | (1 << 2) | (1 << 9);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		ctrl |= (1 << 2) | (1 << 9);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		ctrl = 0;
	}

	writel(ctrl, cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);
}

static int cns3xxx_timer_set_next_event(unsigned long evt,
					struct clock_event_device *unused)
{
	unsigned long ctrl = readl(cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);

	writel(evt, cns3xxx_tmr1 + TIMER1_AUTO_RELOAD_OFFSET);
	writel(ctrl | (1 << 0), cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);

	return 0;
}

static struct clock_event_device cns3xxx_tmr1_clockevent = {
	.name		= "cns3xxx timer1",
	.shift		= 8,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode	= cns3xxx_timer_set_mode,
	.set_next_event	= cns3xxx_timer_set_next_event,
	.rating		= 350,
	.cpumask	= cpu_all_mask,
};

static void __init cns3xxx_clockevents_init(unsigned int timer_irq)
{
	cns3xxx_tmr1_clockevent.irq = timer_irq;
	cns3xxx_tmr1_clockevent.mult =
		div_sc((cns3xxx_cpu_clock() >> 3) * 1000000, NSEC_PER_SEC,
		       cns3xxx_tmr1_clockevent.shift);
	cns3xxx_tmr1_clockevent.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &cns3xxx_tmr1_clockevent);
	cns3xxx_tmr1_clockevent.min_delta_ns =
		clockevent_delta2ns(0xf, &cns3xxx_tmr1_clockevent);

	clockevents_register_device(&cns3xxx_tmr1_clockevent);
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t cns3xxx_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &cns3xxx_tmr1_clockevent;
	u32 __iomem *stat = cns3xxx_tmr1 + TIMER1_2_INTERRUPT_STATUS_OFFSET;
	u32 val;

	/* Clear the interrupt */
	val = readl(stat);
	writel(val & ~(1 << 2), stat);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction cns3xxx_timer_irq = {
	.name		= "timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= cns3xxx_timer_interrupt,
};

/*
 * Set up the clock source and clock events devices
 */
static void __init __cns3xxx_timer_init(unsigned int timer_irq)
{
	u32 val;
	u32 irq_mask;

	/*
	 * Initialise to a known state (all timers off)
	 */

	/* disable timer1 and timer2 */
	writel(0, cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);
	/* stop free running timer3 */
	writel(0, cns3xxx_tmr1 + TIMER_FREERUN_CONTROL_OFFSET);

	/* timer1 */
	writel(0x5C800, cns3xxx_tmr1 + TIMER1_COUNTER_OFFSET);
	writel(0x5C800, cns3xxx_tmr1 + TIMER1_AUTO_RELOAD_OFFSET);

	writel(0, cns3xxx_tmr1 + TIMER1_MATCH_V1_OFFSET);
	writel(0, cns3xxx_tmr1 + TIMER1_MATCH_V2_OFFSET);

	/* mask irq, non-mask timer1 overflow */
	irq_mask = readl(cns3xxx_tmr1 + TIMER1_2_INTERRUPT_MASK_OFFSET);
	irq_mask &= ~(1 << 2);
	irq_mask |= 0x03;
	writel(irq_mask, cns3xxx_tmr1 + TIMER1_2_INTERRUPT_MASK_OFFSET);

	/* down counter */
	val = readl(cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);
	val |= (1 << 9);
	writel(val, cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);

	/* timer2 */
	writel(0, cns3xxx_tmr1 + TIMER2_MATCH_V1_OFFSET);
	writel(0, cns3xxx_tmr1 + TIMER2_MATCH_V2_OFFSET);

	/* mask irq */
	irq_mask = readl(cns3xxx_tmr1 + TIMER1_2_INTERRUPT_MASK_OFFSET);
	irq_mask |= ((1 << 3) | (1 << 4) | (1 << 5));
	writel(irq_mask, cns3xxx_tmr1 + TIMER1_2_INTERRUPT_MASK_OFFSET);

	/* down counter */
	val = readl(cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);
	val |= (1 << 10);
	writel(val, cns3xxx_tmr1 + TIMER1_2_CONTROL_OFFSET);

	/* Make irqs happen for the system timer */
	setup_irq(timer_irq, &cns3xxx_timer_irq);

	cns3xxx_clockevents_init(timer_irq);
}

static void __init cns3xxx_timer_init(void)
{
	cns3xxx_tmr1 = __io(CNS3XXX_TIMER1_2_3_BASE_VIRT);

	__cns3xxx_timer_init(IRQ_CNS3XXX_TIMER0);
}

struct sys_timer cns3xxx_timer = {
	.init = cns3xxx_timer_init,
};
