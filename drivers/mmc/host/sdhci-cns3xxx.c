/*******************************************************************************
 *
 *  drivers/mmc/host/sdhci-cns3xxx.c
 *
 *  SDHCI support for the CNS3XXX SOCs
 *
 *  Author: Scott Shu
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
 *
 ******************************************************************************/

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/mmc/host.h>

#include <asm/scatterlist.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#include <mach/sdhci.h>

#include "sdhci.h"

#define MAX_BUS_CLK	(4)

static unsigned use_dma = 0;

struct sdhci_cns3xxx {
	struct sdhci_host	*host;
	struct platform_device	*pdev;
	struct resource		*ioarea;
	struct clk		*clk_io;
	struct clk		*clk_bus[MAX_BUS_CLK];
};

static unsigned int sdhci_cns3xxx_get_max_clk(struct sdhci_host *host)
{
	int clk = 50000000;

	return clk;
}

static unsigned int sdhci_cns3xxx_get_timeout_clk(struct sdhci_host *host)
{
	return sdhci_cns3xxx_get_max_clk(host) / 100000;
}

/*
 * sdhci_cns3xxx_set_clock - callback on clock change
 *
 * When the card's clock is going to be changed, look at the new frequency
 * and find the best clock source to go with it.
 */
static void sdhci_cns3xxx_set_clock(struct sdhci_host *host, unsigned int clock)
{
	u16 clk;
	unsigned long timeout;
	int max_speed, div = 1;
	int hclk = cns3xxx_cpu_clock()/4;

	if (clock == host->clock)
		return;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;

   /* 
	* SD frequency divider should be determined by HCLK
	* the maximum SD clock is 25MHz (normal speed mode) or 50MHz (high speed mode)
	* formula => SD clock = (HCLK / divisor)
	*/
	div = 1;
	hclk = cns3xxx_cpu_clock()/4;

	if (0x4 & sdhci_readw(host, 0x28)) {
		max_speed = 50;
	} else {
		max_speed = 25;
	}

	while (max_speed < (hclk/div)) { div++; }
		
	switch (div) {
	case 1:
		clk = 0x00 << SDHCI_DIVIDER_SHIFT;	/* */
		break;
	case 2:
		clk = 0x01 << SDHCI_DIVIDER_SHIFT;	/* base clock divided by 2 */
		break;
	case 3:
		clk = 0x03 << SDHCI_DIVIDER_SHIFT;	/* base clock divided by 3 */
		break;
	case 4:
		clk = 0x02 << SDHCI_DIVIDER_SHIFT;	/* base clock divided by 4 */
		break;
	case 5: case 6: case 7: case 8:
		clk = 0x04 << SDHCI_DIVIDER_SHIFT;	/* base clock divided by 8 */
		break;
	default:
		clk = 0x08 << SDHCI_DIVIDER_SHIFT;	/* base clock divided by 16 */
	}

	if (clock == 400000)  //for eMMC init
	{
	    clk = 0x80 << 8;	
        }	

	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	timeout = 10;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	host->timeout_clk = sdhci_cns3xxx_get_timeout_clk(host);
out:
	host->clock = clock;

	sdhci_writew(host, 0x0007, SDHCI_TIMEOUT_CONTROL);
}

static struct sdhci_ops sdhci_cns3xxx_ops = {
	.get_max_clock		= sdhci_cns3xxx_get_max_clk,
	.get_timeout_clock	= sdhci_cns3xxx_get_timeout_clk,
	.set_clock		= sdhci_cns3xxx_set_clock,
};

static int sdhci_cns3xxx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host;
	struct sdhci_cns3xxx *sc;
	struct resource *res;
	int ret, irq;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "no irq specified\n");
		return irq;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory specified\n");
		return -ENOENT;
	}

	host = sdhci_alloc_host(dev, sizeof(*sc));
	if (IS_ERR(host)) {
		dev_err(dev, "sdhci_alloc_host() failed\n");
		return PTR_ERR(host);
	}

	sc = sdhci_priv(host);

	sc->host = host;
	sc->pdev = pdev;

	platform_set_drvdata(pdev, host);

	sc->ioarea = request_mem_region(res->start, resource_size(res), mmc_hostname(host->mmc));
	if (!sc->ioarea) {
		dev_err(dev, "failed to reserve register area\n");
		ret = -ENXIO;
		goto err_req_regs;
	}

	host->ioaddr = ioremap_nocache(res->start, resource_size(res));
	if (!host->ioaddr) {
		dev_err(dev, "failed to map registers\n");
		ret = -ENXIO;
		goto err_req_regs;
	}

	host->hw_name = "cns3xxx";
	host->ops = &sdhci_cns3xxx_ops;
	host->quirks = 0;
	host->irq = irq;

	if (use_dma != 1) {
		host->quirks |= SDHCI_QUIRK_BROKEN_DMA;
		host->quirks |= SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK;
	} else {
		host->quirks |= SDHCI_QUIRK_FORCE_DMA;
		host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;
		host->quirks |= (SDHCI_QUIRK_32BIT_DMA_ADDR | SDHCI_QUIRK_32BIT_DMA_SIZE);
		host->quirks |= SDHCI_QUIRK_NO_BUSY_IRQ;
		//host->quirks |= SDHCI_QUIRK_FORCE_BLK_SZ_2048;
		//host->quirks |= SDHCI_QUIRK_NO_MULTIBLOCK;
		host->quirks |= SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK;
//		host->quirks |= SDHCI_QUIRK_AUTO_CMD12;
//		host->quirks |= SDHCI_QUIRK_READ_WAIT_CTRL;
	}

	host->quirks |= SDHCI_QUIRK_INVERTED_WRITE_PROTECT;
	host->quirks |= SDHCI_QUIRK_NONSTANDARD_CLOCK;
	host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;   // workaround for XO1 hardware

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(dev, "sdhci_add_host() failed (%d)\n", ret);
		goto err_add_host;
	}

	return 0;

err_add_host:
	free_irq(host->irq, host);
	iounmap(host->ioaddr);
	release_resource(sc->ioarea);
//	kfree(sc->ioarea);

err_req_regs:
	sdhci_free_host(host);

	return ret;
}

static int sdhci_cns3xxx_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct resource *res;

	pr_debug("%s: remove=%p\n", __func__, pdev);

	sdhci_remove_host(host, 0);
	sdhci_free_host(host);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	return 0;
}

#ifdef CONFIG_PM

static int sdhci_cns3xxx_suspend(struct platform_device *dev, pm_message_t state)
{

	return 0;
}

static int sdhci_cns3xxx_resume(struct platform_device *dev)
{

	return 0;
}

#else
#define sdhci_cns3xxx_suspend	NULL
#define sdhci_cns3xxx_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver sdhci_cns3xxx_driver = {
	.probe		= sdhci_cns3xxx_probe,
	.remove		= __devexit_p(sdhci_cns3xxx_remove),
	.suspend	= sdhci_cns3xxx_suspend,
	.resume		= sdhci_cns3xxx_resume,
	.driver		= {
		.name	= "sdhci-cns3xxx",
		.owner	= THIS_MODULE,
	},
};

static char banner[] __initdata = KERN_INFO "sdhci-cns3xxx, (c) 2009 Cavium Networks\n";

static int __init sdhci_cns3xxx_init(void)
{
	unsigned long iocdb = 0;
	unsigned long gpioapin = __raw_readl(__io(CNS3XXX_MISC_BASE_VIRT + 0x0014));

	printk(banner);


	/* MMC/SD mode select */
	//__raw_writel(0x2, __io(CNS3XXX_MISC_BASE_VIRT + 0x0700));
       // printk("\n Putting CNS34XX as MMC mode (0x%08x) \n",__raw_readl(__io(CNS3XXX_MISC_BASE_VIRT + 0x0700)));

	/* MMC/SD pins share with GPIOA */
	__raw_writel(gpioapin | (0x1f0f0004), __io(CNS3XXX_MISC_BASE_VIRT + 0x0014));
	cns3xxx_pwr_clk_en(CNS3XXX_PWR_CLK_EN(SDIO));
	cns3xxx_pwr_soft_rst(CNS3XXX_PWR_SOFTWARE_RST(SDIO));

	// set SDIO drive strength to 2 (15.7 mA)
	iocdb = __raw_readl(__io(CNS3XXX_MISC_BASE_VIRT + 0x0020));;
	iocdb &= (~(3 << 10));
	iocdb |= (2 << 10);
	__raw_writel(iocdb, __io(CNS3XXX_MISC_BASE_VIRT + 0x0020));

	return platform_driver_register(&sdhci_cns3xxx_driver);
}

static void __exit sdhci_cns3xxx_exit(void)
{
	platform_driver_unregister(&sdhci_cns3xxx_driver);
}

module_init(sdhci_cns3xxx_init);
module_exit(sdhci_cns3xxx_exit);

module_param(use_dma, uint, 0);

MODULE_AUTHOR("Scott Shu");
MODULE_DESCRIPTION("Cavium Networks CNS3XXX SDHCI glue");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sdhci-cns3xxx");

MODULE_PARM_DESC(use_dma, "Whether to use DMA or not. Default = 0");
