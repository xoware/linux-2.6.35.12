/*******************************************************************************
 *
 *  drivers/gpio/cns3xxx-gpio.c
 *
 *  GPIO driver for the CNS3XXX SOCs
 *
 *  Copyright (c) 2009 Cavium Networks 
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/irq.h>
//#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>

#include <mach/pm.h>
//#include <mach/gpio.h>

#include <mach/cns3xxx.h>
#include <linux/gpio.h>
#include <mach/misc.h>


#define GPIOA_PIN_NO	32
#define GPIOB_PIN_NO	32
#define MAX_GPIO_NO	(GPIOA_PIN_NO + GPIOB_PIN_NO)
#define CNS3XXX_GPIO_MAX MAX_GPIO_NO

#if 0
#define gpio_get_value        __gpio_get_value
#define gpio_set_value        __gpio_set_value
#define gpio_cansleep         __gpio_cansleep
#endif

#ifdef CONFIG_GPIO_DEBUG 
#define __pr_debug(fmt,args...)		printk( KERN_ERR fmt,##args )
#else
#define __pr_debug(fmt,args...)
#endif 

//#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT
#define PIN_INPUT 0
#define PIN_OUTPUT 1

#define PIN_TRIG_EDGE 0
#define PIN_TRIG_LEVEL 1

#define PIN_TRIG_SINGLE 0
#define PIN_TRIG_BOTH 1

#define PIN_TRIG_RISING 0
#define PIN_TRIG_FALLING 1

#define PIN_TRIG_HIGH 0
#define PIN_TRIG_LOW 1
void (*gpio_a_isr[GPIOA_PIN_NO])(int i);
void (*gpio_b_isr[GPIOB_PIN_NO])(int i);
int intr_a_count=0;
int intr_b_count=0;
//#endif	// CONFIG_CNS3XXX_GPIO_INTERRUPT

extern void gic_mask_irq(unsigned int irq);
extern void gic_unmask_irq(unsigned int irq);
extern void gic_ack_irq(unsigned int irq);
void cns3xxx_gpio_set_levelintr(void (*funcptr)(int),int trig_level, int gpio_pin);
void cns3xxx_gpio_set_edgeintr(void (*funcptr)(int), int trig_both, int trig_rising, int gpio_pin);
#define CONFIG_CNS3XXX_GPIO_INTERRUPT
#define CONFIG_CNS3XXX_GPIO_INTERRUPT_TEST

/***********************************************************************
 * The CNS3XXX has GPIOA(32) and GPIOB(32) total 64 GPIO. For the
 * generic GPIO interface, the GPIO pin number count from GPIOA to GPIOB.
 * For example:
 *      0 -> GPIOA[0]
 *      1 -> GPIOA[1]
 *     ......
 *     31 -> GPIOA[31]
 *     32 -> GPIOB[0]
 *     33 -> GPIOB[1]
 *     ......
 *     63 -> GPIOB[31]
 **********************************************************************/

/*
 * Configure the GPIO line as an input.
 */
int cns3xxx_set_gpio_direction_input(unsigned int pin)
{
	volatile __u32 reg;
	unsigned long flags;

	if (pin >= MAX_GPIO_NO)
		return -EINVAL;

	local_irq_save(flags);


	/* Clear register bit to set as input pin. */
	if (pin < GPIOA_PIN_NO) { /* GPIOA */
		reg = __raw_readl(CNS3XXX_GPIOA_BASE_VIRT + GPIO_DIR);
		reg &= ~(1 << pin);
		__raw_writel(reg, CNS3XXX_GPIOA_BASE_VIRT + GPIO_DIR);
	} else {		  /* GPIOB */
		reg = __raw_readl(CNS3XXX_GPIOB_BASE_VIRT + GPIO_DIR);
		reg &= ~(1 << (pin - GPIOA_PIN_NO));
		__raw_writel(reg, CNS3XXX_GPIOB_BASE_VIRT + GPIO_DIR);
	}

	local_irq_restore(flags);

	return 0;
}

static int cns3xxx_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return (offset < CNS3XXX_GPIO_MAX)
		? cns3xxx_set_gpio_direction_input(offset)
		: -EINVAL;
}


/*
 * Configure the GPIO line as an output, with default state.
 */
int __init_or_module cns3xxx_set_gpio_direction_output(unsigned int pin, unsigned int state)
{
	volatile __u32 reg;
	u32 base, gpio_index;
	unsigned long flags;
	if (pin < 0 || pin >= MAX_GPIO_NO)
		return -EINVAL;

	local_irq_save(flags);

	if (pin < GPIOA_PIN_NO) { /* GPIOA */
		base = CNS3XXX_GPIOA_BASE_VIRT;
		gpio_index = 1 << pin;
	} else {		  /* GPIOB */
		base = CNS3XXX_GPIOB_BASE_VIRT;
		gpio_index = 1 << (pin - GPIOA_PIN_NO);
	}

	/* Set register bit to set as output pin. */
	reg = __raw_readl(base + GPIO_DIR);
	reg |= gpio_index;
	__raw_writel(reg, base + GPIO_DIR);

	if (state)
		__raw_writel(gpio_index, base + GPIO_BIT_SET);
	else
		__raw_writel(gpio_index, base + GPIO_BIT_CLEAR);

	local_irq_restore(flags);

	return 0;
}

int __init_or_module cns3xxx_set_gpio_direction_output_only(unsigned int pin)
{
       volatile __u32 reg;
       u32 base, gpio_index;
       unsigned long flags;
       if (pin < 0 || pin >= MAX_GPIO_NO)
               return -EINVAL;

       local_irq_save(flags);

       if (pin < GPIOA_PIN_NO) { /* GPIOA */
               base = CNS3XXX_GPIOA_BASE_VIRT;
               gpio_index = 1 << pin;
       } else {                  /* GPIOB */
               base = CNS3XXX_GPIOB_BASE_VIRT;
               gpio_index = 1 << (pin - GPIOA_PIN_NO);
       }

       /* Set register bit to set as output pin. */
       reg = __raw_readl(base + GPIO_DIR);
       reg |= gpio_index;
       __raw_writel(reg, base + GPIO_DIR);

       /*if (state)
               __raw_writel(gpio_index, base + GPIO_BIT_SET);
       else
               __raw_writel(gpio_index, base + GPIO_BIT_CLEAR);*/

       local_irq_restore(flags);

       return 0;
}

static int cns3xxx_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	return (offset < CNS3XXX_GPIO_MAX)
		? cns3xxx_set_gpio_direction_output(offset, value)
		: -EINVAL;
}

/*
 * Set the state of an output GPIO line.
 */
void cns3xxx_gpio_set_value(unsigned int pin, unsigned int state)
{
	if (pin < 0 || pin >= MAX_GPIO_NO)
		return;

	if (pin < GPIOA_PIN_NO)	{ /* GPIOA */
		if (state)
			__raw_writel(1 << pin, CNS3XXX_GPIOA_BASE_VIRT + GPIO_BIT_SET);
		else
			__raw_writel(1 << pin, CNS3XXX_GPIOA_BASE_VIRT + GPIO_BIT_CLEAR);
	} else {		  /* GPIOB */
		if (state)
			__raw_writel(1 << (pin - GPIOA_PIN_NO), CNS3XXX_GPIOB_BASE_VIRT + GPIO_BIT_SET);
		else
			__raw_writel(1 << (pin - GPIOA_PIN_NO), CNS3XXX_GPIOB_BASE_VIRT + GPIO_BIT_CLEAR);
	}
}

static void cns3xxx_set(struct gpio_chip *chip, unsigned offset, int value)
{
	cns3xxx_gpio_set_value(offset, value);
}

/*
 * Read the state of a GPIO line.
 */
int cns3xxx_gpio_get_value(unsigned int pin)
{
	volatile __u32 reg;
	bool bret = 0;

	if (pin < 0 || pin >= MAX_GPIO_NO)
		return -EINVAL;

	if (pin < GPIOA_PIN_NO) { /* GPIOA */
		reg = __raw_readl(CNS3XXX_GPIOA_BASE_VIRT + GPIO_INPUT);
		bret = (reg & (1 << pin)) != 0;
	} else {		  /* GPIOB */
		reg = __raw_readl(CNS3XXX_GPIOB_BASE_VIRT + GPIO_INPUT);
		bret = (reg & (1 << (pin - GPIOA_PIN_NO))) != 0;
	}
	
	return bret;
}

static int cns3xxx_get(struct gpio_chip *chip, unsigned offset)
{
	return cns3xxx_gpio_get_value(offset);
}

#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT_TEST
static irqreturn_t cns3xxx_gpio_irq_handler(int irq, void *dev_id)
{
	unsigned int volatile    status;
	int i;

	// Clean System irq status
	/* FIXME if needed
	HAL_INTC_CLEAR_EDGE_TRIGGER_INTERRUPT(INTC_GPIO_EXTERNAL_INT_BIT_INDEX);
	*/ 

	/* FIXME need to check if GPIOB has different interrupt source */
	disable_irq_nosync(IRQ_CNS3XXX_GPIOA);

	status = __raw_readl(CNS3XXX_GPIOA_BASE_VIRT + GPIO_INTERRUPT_MASKED_STATUS);

	for (i = 0; i < GPIOA_PIN_NO; i++) {   
		if (status & (1 << i))  {   		/* interrupt is detected and not masked */
			if (gpio_a_isr[i] != NULL) {
				gpio_a_isr[i](i);
			}
		}    
	}  
	__raw_writel(status, CNS3XXX_GPIOA_BASE_VIRT + GPIO_INTERRUPT_CLEAR);

	status = __raw_readl(CNS3XXX_GPIOB_BASE_VIRT + GPIO_INTERRUPT_MASKED_STATUS);
	for (i = 0; i < GPIOB_PIN_NO; i++) {   
		if (status & (1 << i)) {			/* interrupt is detected and not masked */
			if (gpio_b_isr[i] != NULL) {
				gpio_b_isr[i](i);
			}
		}    
	}   
	__raw_writel(status, CNS3XXX_GPIOB_BASE_VIRT + GPIO_INTERRUPT_CLEAR);

	/* Unmask Intc Interrupt Status */
	enable_irq(IRQ_CNS3XXX_GPIOA);
	//enable_irq(IRQ_CNS3XXX_GPIOB);
	return IRQ_HANDLED;
}
#endif

//#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT
static int cns3xxx_set_interrupt_method(int method, int gpio_pin)
{
	volatile __u32 reg;
	u32 base, gpio_index;

	if (gpio_pin >= 0 && gpio_pin < GPIOA_PIN_NO) {	/* GPIOA */
		base = CNS3XXX_GPIOA_BASE_VIRT;
		gpio_index = 0x1 << gpio_pin;
	} else if (gpio_pin < MAX_GPIO_NO) {		/* GPIOB */
		base = CNS3XXX_GPIOB_BASE_VIRT;
		gpio_index = 0x1 << (gpio_pin - GPIOA_PIN_NO);
	} else {
		printk("GPIO pin number error.\n");
		return -EINVAL;
	}

	reg = __raw_readl(base + GPIO_INTERRUPT_TRIGGER_METHOD);

	if (method)			/* level trigger */
		reg |= gpio_index;
	else 				/* edge trigger */
		reg &= ~gpio_index;

	__raw_writel(reg, base + GPIO_INTERRUPT_TRIGGER_METHOD);

	return 0;
}

static int cns3xxx_set_interrupt_both_edges(int both_edge, int gpio_pin)
{
	volatile __u32 reg;
	u32 base, gpio_index;

	if (gpio_pin >= 0 && gpio_pin < GPIOA_PIN_NO) {	/* GPIOA */
		base = CNS3XXX_GPIOA_BASE_VIRT;
		gpio_index = 0x1 << gpio_pin;
	} else if (gpio_pin < MAX_GPIO_NO) {		/* GPIOB */
		base = CNS3XXX_GPIOB_BASE_VIRT;
		gpio_index = 0x1 << (gpio_pin - GPIOA_PIN_NO);
	} else {
		printk("GPIO pin number error.\n");
		return -EINVAL;
	}

	reg = __raw_readl(base + GPIO_INTERRUPT_TRIGGER_BOTH_EDGES);

	if (both_edge)			/* both edge */
		reg |= gpio_index;
	else				/* single edge */
		reg &= ~gpio_index;

	__raw_writel(reg, base + GPIO_INTERRUPT_TRIGGER_BOTH_EDGES);

	return 0;
}

static int cns3xxx_set_interrupt_type(int type, int gpio_pin)
{
	volatile __u32 reg;
	u32 base, gpio_index;

	if (gpio_pin >= 0 && gpio_pin < GPIOA_PIN_NO) {	/* GPIOA */
		base = CNS3XXX_GPIOA_BASE_VIRT;
		gpio_index = 0x1 << gpio_pin;
	} else if (gpio_pin < MAX_GPIO_NO) {		/* GPIOB */
		base = CNS3XXX_GPIOB_BASE_VIRT;
		gpio_index = 0x1 << (gpio_pin - GPIOA_PIN_NO);
	} else {
		printk("GPIO pin number error.\n");
		return -EINVAL;
	}

	reg = __raw_readl(base + GPIO_INTERRUPT_TRIGGER_TYPE);

	if (type)			/* falling edge or low level*/
		reg |= gpio_index;
	else				/* rising edge or high level */
		reg &= ~gpio_index;

	__raw_writel(reg, base + GPIO_INTERRUPT_TRIGGER_TYPE);

	return 0;
}

/*  
 * Setup GPIO for Edge Trigger Interrupt mode 
 */
void cns3xxx_gpio_set_edgeintr(void (*funcptr)(int), int trig_both, int trig_rising, int gpio_pin)
{
	u32 base, gpio_index;

	if (gpio_pin >= 0 && gpio_pin < GPIOA_PIN_NO) {
		base = CNS3XXX_GPIOA_BASE_VIRT;
		gpio_a_isr[gpio_pin] = funcptr;
		gpio_index = 1 << gpio_pin;
	} else if (gpio_pin < MAX_GPIO_NO) {
		base = CNS3XXX_GPIOB_BASE_VIRT;
		gpio_b_isr[gpio_pin - GPIOA_PIN_NO] = funcptr;
		gpio_index = 1 << (gpio_pin - GPIOA_PIN_NO);
	} else {
		printk("GPIO pin number error.\n");
		return;
	}

	cns3xxx_set_gpio_direction_input(gpio_pin);

	if (trig_both == PIN_TRIG_BOTH) {
		/* Set Trigger Both */
		cns3xxx_set_interrupt_method(GPIO_INTERRUPT_TRIGGER_METHOD_EDGE, gpio_pin);
		cns3xxx_set_interrupt_both_edges(GPIO_INTERRUPT_TRIGGER_EDGE_BOTH, gpio_pin);
	} else if (trig_both == PIN_TRIG_SINGLE) {
		/* Set Single Rising/Falling Edge Trigger */
		if (trig_rising == PIN_TRIG_RISING) {
			cns3xxx_set_interrupt_method(GPIO_INTERRUPT_TRIGGER_METHOD_EDGE, gpio_pin);
			cns3xxx_set_interrupt_both_edges(GPIO_INTERRUPT_TRIGGER_EDGE_SINGLE, gpio_pin);
			cns3xxx_set_interrupt_type(GPIO_INTERRUPT_TRIGGER_TYPE_RISING, gpio_pin);
		} else if (trig_rising == PIN_TRIG_FALLING) {
			cns3xxx_set_interrupt_method(GPIO_INTERRUPT_TRIGGER_METHOD_EDGE, gpio_pin);
			cns3xxx_set_interrupt_both_edges(GPIO_INTERRUPT_TRIGGER_EDGE_SINGLE, gpio_pin);
			cns3xxx_set_interrupt_type(GPIO_INTERRUPT_TRIGGER_TYPE_FALLING, gpio_pin);
		} else {
			printk("Trigger rising/falling error.\n");
			return;
		}
	} else {
		printk("Trigger both/single error.\n");
		return;
	}

	/* Enable Interrupt */
	__raw_writel(gpio_index, base + GPIO_INTERRUPT_ENABLE);
}

/*  
 * Setup GPIO for LEVEL Trigger Interrupt mode 
 */
void cns3xxx_gpio_set_levelintr(void (*funcptr)(int),int trig_level, int gpio_pin)
{
	u32 base, gpio_index;

	if (gpio_pin >= 0 && gpio_pin < GPIOA_PIN_NO) {
		base = CNS3XXX_GPIOA_BASE_VIRT;
		gpio_a_isr[gpio_pin] = funcptr;
		gpio_index = 1 << gpio_pin;
	} else if (gpio_pin < MAX_GPIO_NO) {
		base = CNS3XXX_GPIOB_BASE_VIRT;
		gpio_b_isr[gpio_pin - GPIOA_PIN_NO] = funcptr;
		gpio_index = 1 << (gpio_pin - GPIOA_PIN_NO);
	} else {
		printk("GPIO pin number error.\n");
		return;
	}

	cns3xxx_set_gpio_direction_input(gpio_pin);

	/* Set Trigger High/Low */
	if (trig_level == PIN_TRIG_HIGH) {
		cns3xxx_set_interrupt_method(GPIO_INTERRUPT_TRIGGER_METHOD_LEVEL, gpio_pin);
		cns3xxx_set_interrupt_type(GPIO_INTERRUPT_TRIGGER_TYPE_HIGH, gpio_pin);
	} else if (trig_level == PIN_TRIG_LOW) {
		cns3xxx_set_interrupt_method(GPIO_INTERRUPT_TRIGGER_METHOD_LEVEL, gpio_pin);
		cns3xxx_set_interrupt_type(GPIO_INTERRUPT_TRIGGER_TYPE_LOW, gpio_pin);
	} else {
		printk("Trigger level error.\n");
		return;
	}

	/* Enable Interrupt */
	__raw_writel(gpio_index, base + GPIO_INTERRUPT_ENABLE);
}

/*  
 * Clear GPIOA Trigger Interrupt
 */
void cns3xxx_gpio_a_clear_intr(int gpio_pin)
{
	volatile __u32 reg;

	if (gpio_pin >= 0 && gpio_pin < GPIOA_PIN_NO) {
		/* Unregister isr of GPIOA[gpio_pin] */	
		gpio_a_isr[gpio_pin] = NULL;	

		/* Disable Interrupt */
		reg = __raw_readl(CNS3XXX_GPIOA_BASE_VIRT + GPIO_INTERRUPT_ENABLE);
		reg &= ~(1 << gpio_pin);
		__raw_writel(reg, CNS3XXX_GPIOA_BASE_VIRT + GPIO_INTERRUPT_ENABLE);
	}
}

/*  
 * Clear GPIOB Trigger Interrupt
 */
void cns3xxx_gpio_b_clear_intr(int gpio_pin)
{
	volatile __u32 reg;

	if (gpio_pin >= 0 && gpio_pin < GPIOB_PIN_NO) {
		/* Unregister isr of GPIOB[gpio_pin] */
		gpio_b_isr[gpio_pin] = NULL;		

		// Disable Interrupt 
		reg = __raw_readl(CNS3XXX_GPIOB_BASE_VIRT + GPIO_INTERRUPT_ENABLE);
		reg &= ~(1 << gpio_pin);
		__raw_writel(reg, CNS3XXX_GPIOB_BASE_VIRT + GPIO_INTERRUPT_ENABLE);
		
	}
}
EXPORT_SYMBOL(cns3xxx_gpio_set_edgeintr);
EXPORT_SYMBOL(cns3xxx_gpio_set_levelintr);
EXPORT_SYMBOL(cns3xxx_gpio_a_clear_intr);
EXPORT_SYMBOL(cns3xxx_gpio_b_clear_intr);

EXPORT_SYMBOL(cns3xxx_gpio_get_value);
EXPORT_SYMBOL(cns3xxx_gpio_set_value);
EXPORT_SYMBOL(cns3xxx_set_gpio_direction_input);
EXPORT_SYMBOL(cns3xxx_set_gpio_direction_output_only);
//#endif

#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT_TEST
static void cns3xxx_gpio_a_isr_test(int i)
{
	unsigned int volatile status = 0;

	disable_irq_nosync(IRQ_CNS3XXX_GPIOA);

	printk("gpio_a_isr_test, count:%d\n",intr_a_count+1);
	status = __raw_readl(CNS3XXX_GPIOA_BASE_VIRT + GPIO_INTERRUPT_MASKED_STATUS);
	__raw_writel(status, CNS3XXX_GPIOA_BASE_VIRT + GPIO_INTERRUPT_CLEAR);

	/* FIXME if needed
	HAL_INTC_CLEAR_EDGE_TRIGGER_INTERRUPT(INTC_GPIO_EXTERNAL_INT_BIT_INDEX);
	*/

	intr_a_count++;
	if (intr_a_count >= 4) {
		cns3xxx_gpio_a_clear_intr(i);
		intr_a_count = 0;
	}

	enable_irq(IRQ_CNS3XXX_GPIOA);
}

static void cns3xxx_gpio_b_isr_test(int i)
{
	unsigned int volatile status = 0;

	disable_irq_nosync(IRQ_CNS3XXX_GPIOA); /* FIXME */

	printk("gpio_b_isr_test, count:%d\n",intr_b_count+1);
	status = __raw_readl(CNS3XXX_GPIOB_BASE_VIRT + GPIO_INTERRUPT_MASKED_STATUS);
	__raw_writel(status, CNS3XXX_GPIOB_BASE_VIRT + GPIO_INTERRUPT_CLEAR);

	/* FIXME if needed
	HAL_INTC_CLEAR_EDGE_TRIGGER_INTERRUPT(INTC_GPIO_EXTERNAL_INT_BIT_INDEX);
	*/

	intr_b_count++;
	if (intr_b_count >= 4) {
		cns3xxx_gpio_b_clear_intr(i);
		intr_b_count = 0;
	}

	enable_irq(IRQ_CNS3XXX_GPIOA); /* FIXME */
}
#endif

/*
Read String into Buffer, Max String buffer is 100
*/
ssize_t readstring(char *buff, const char *buf, size_t count){
    	int i=0;
        if (count) {
                char c;

            for(i=0;i<count&&i<100;i++){
                if (get_user(c, buf+i))
                        return -EFAULT;
                    buff[i] = c;
            }
                buff[i]=0;
        }
        return count;
}

#define MISC_REG_VALUE(offset) (*((volatile unsigned int *)(CNS3XXX_MISC_BASE_VIRT+offset)))





static int cns3xxx_gpio_read_proc(char *page, char **start,  off_t off, int count, int *eof, void *data)
{
	int num = 0;

	num += sprintf(page+num, "********** GPIO Group A **********\n");
	num += sprintf(page+num, "GPIO A ENABLE PIN      : %08x \n",MISC_GPIOA_PIN_ENABLE_REG );
	num += sprintf(page+num, "GPIO OUT               : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_OUTPUT));
	num += sprintf(page+num, "GPIO IN                : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INPUT));
	num += sprintf(page+num, "GPIO Direction         : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_DIR));
#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT
	num += sprintf(page+num, "GPIO Interrupt Enable  : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INTERRUPT_ENABLE));
	num += sprintf(page+num, "GPIO Interrupt Raw     : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INTERRUPT_RAW_STATUS));
	num += sprintf(page+num, "GPIO Interrupt Trigger : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INTERRUPT_TRIGGER_METHOD));
	num += sprintf(page+num, "GPIO Interrupt Both    : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INTERRUPT_TRIGGER_BOTH_EDGES));
	num += sprintf(page+num, "GPIO Interrupt RiseNeg : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INTERRUPT_TRIGGER_TYPE));
	num += sprintf(page+num, "GPIO Interrupt MASKED  : %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INTERRUPT_MASK));
	num += sprintf(page+num, "GPIO Interrupt MASKEDST: %08x \n", CNS3XXX_GPIO_REG(0,GPIO_INTERRUPT_MASKED_STATUS));
#endif	

	num += sprintf(page+num, "********** GPIO Group B **********\n");
	num += sprintf(page+num, "GPIO B ENABLE PIN      : %08x \n",MISC_GPIOB_PIN_ENABLE_REG );
	num += sprintf(page+num, "GPIO OUT               : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_OUTPUT));
	num += sprintf(page+num, "GPIO IN                : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INPUT));
	num += sprintf(page+num, "GPIO Direction         : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_DIR));
#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT
	num += sprintf(page+num, "GPIO Interrupt Enable  : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INTERRUPT_ENABLE));
	num += sprintf(page+num, "GPIO Interrupt Raw     : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INTERRUPT_RAW_STATUS));
	num += sprintf(page+num, "GPIO Interrupt Trigger : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INTERRUPT_TRIGGER_METHOD));
	num += sprintf(page+num, "GPIO Interrupt Both    : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INTERRUPT_TRIGGER_BOTH_EDGES));
	num += sprintf(page+num, "GPIO Interrupt RiseNeg : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INTERRUPT_TRIGGER_TYPE));
	num += sprintf(page+num, "GPIO Interrupt MASKED  : %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INTERRUPT_MASK));
	num += sprintf(page+num, "GPIO Interrupt MASKEDST: %08x \n", CNS3XXX_GPIO_REG(1,GPIO_INTERRUPT_MASKED_STATUS));
#endif	
	num += sprintf(page+num, "MISC_REG 0x1c: %08x \n", MISC_REG_VALUE(0x1c));
	num += sprintf(page+num, "MISC_REG 0x20: %08x \n", MISC_REG_VALUE(0x20));
	num += sprintf(page+num, "MISC_REG 0x24: %08x \n", MISC_REG_VALUE(0x24));
	num += sprintf(page+num, "MISC_REG 0x28: %08x \n", MISC_REG_VALUE(0x28));
	num += sprintf(page+num, "MISC_REG 0x2c: %08x \n", MISC_REG_VALUE(0x2c));
	num += sprintf(page+num, "MISC_REG 0x30: %08x \n", MISC_REG_VALUE(0x30));
	num += sprintf(page+num, "MISC_REG 0x34: %08x \n", MISC_REG_VALUE(0x34));

	return num;
}
static int cns3xxx_gpio_set_irq_type(u32 gpio_pin, u32 type);

static int cns3xxx_gpio_write_proc(struct file *file, const char __user *buffer,
			   unsigned long count, void *data){
	int pin = 0,state =0;
	char read_buff[100],buf_cmd[100],buf_param1[100],buf_param2[100],buf_param3[100],buf_param4[100];
	readstring((char *)read_buff,(const char *)buffer,count);
	sscanf(read_buff,"%s %s %s %s %s\n",(char *)&buf_cmd,(char *)&buf_param1\
									   ,(char *)&buf_param2, (char *)&buf_param3, (char *)&buf_param4);

	if (strcmp(buf_cmd,"direct") == 0) {
		if (strcmp(buf_param1,"input") == 0){
			sscanf(buf_param2, "%d", &pin);
			printk("direction input pin=%d \n",pin);
			cns3xxx_set_gpio_direction_input(pin);
		} else if (strcmp(buf_param1,"output") == 0){
			sscanf(buf_param2, "%d", &pin);
			sscanf(buf_param3, "%d", &state);
			printk("direction output pin=%d state=%d  \n",pin,state);
			cns3xxx_set_gpio_direction_output(pin,state);
		} else 
			printk("syntax: direct input(output) pin (state)\n");
		return count;
	}

	if (strcmp(buf_cmd,"set") == 0) {
		if(strcmp(buf_param1,"value") == 0){
			sscanf(buf_param2, "%d", &pin);
			sscanf(buf_param3, "%d", &state);
			printk("set value pin=%d state=%d  \n",pin,state);
			cns3xxx_gpio_set_value(pin,state);
		}
		else
			printk("syntax: set value pin state\n");
		return count;
	}

	if (strcmp(buf_cmd,"get") == 0) {
		if (strcmp(buf_param1,"value") == 0){
			sscanf(buf_param2, "%d", &pin);
			state=cns3xxx_gpio_get_value(pin);
			printk("get value pin=%d state=%d \n",pin,state);
		}
		else
			printk("syntax: get value pin\n");
		return count;
	}

#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT_TEST
	if (strcmp(buf_cmd,"trig") == 0) {
		if (strcmp(buf_param1,"edge") == 0) {
			if(strcmp(buf_param2,"both") == 0) {
				sscanf(buf_param3, "%d", &pin);
				if (pin < GPIOA_PIN_NO)
					cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_a_isr_test, 1, 0, pin);
				else
					cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_b_isr_test, 1, 0, pin);
				return count;
			}
			if (strcmp(buf_param2,"single") == 0) {
				if (strcmp(buf_param3,"rising") == 0) {
					sscanf(buf_param4, "%d", &pin);
					if (pin < GPIOA_PIN_NO)
						cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_a_isr_test, 0, 0, pin);
					else
						cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_b_isr_test, 0, 0, pin);
					return count;
				}
				if (strcmp(buf_param3,"falling") == 0) {
					sscanf(buf_param4, "%d", &pin);
					if (pin < GPIOA_PIN_NO)
						cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_a_isr_test, 0, 1, pin);
					else
						cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_b_isr_test, 0, 1, pin);
					return count;
				}		
			}
		}
		if (strcmp(buf_param1,"level") == 0) {
			if (strcmp(buf_param2,"high") == 0) {
				sscanf(buf_param3, "%d", &pin);
				if (pin < GPIOA_PIN_NO)
					cns3xxx_gpio_set_levelintr(&cns3xxx_gpio_a_isr_test, 0, pin);
				else
					cns3xxx_gpio_set_levelintr(&cns3xxx_gpio_b_isr_test, 0, pin);
				return count;
			}
			if (strcmp(buf_param2,"low") == 0) {
				sscanf(buf_param3, "%d", &pin);
				if (pin < GPIOA_PIN_NO)
					cns3xxx_gpio_set_levelintr(&cns3xxx_gpio_a_isr_test, 1, pin);
				else
					cns3xxx_gpio_set_levelintr(&cns3xxx_gpio_b_isr_test, 1, pin);
				return count;
			}
		}
	}

	if (strcmp(buf_cmd,"test") == 0) {
		if (strcmp(buf_param1,"1") == 0){
			sscanf(buf_param2, "%d", &pin);
			printk("IRQ_TYPE_EDGE_RISING pin=%d \n",pin);
			cns3xxx_gpio_set_irq_type(pin,IRQ_TYPE_EDGE_RISING);
		} else if (strcmp(buf_param1,"2") == 0){
			sscanf(buf_param2, "%d", &pin);
			printk("IRQ_TYPE_EDGE_FALLING pin=%d \n",pin);
			cns3xxx_gpio_set_irq_type(pin,IRQ_TYPE_EDGE_FALLING);
		} else if (strcmp(buf_param1,"3") == 0){
			sscanf(buf_param2, "%d", &pin);
			printk("IRQ_TYPE_EDGE_BOTH pin=%d \n",pin);
			cns3xxx_gpio_set_irq_type(pin,IRQ_TYPE_EDGE_BOTH);
		} else if (strcmp(buf_param1,"4") == 0){
			sscanf(buf_param2, "%d", &pin);
			printk("IRQ_TYPE_LEVEL_HIGH pin=%d \n",pin);
			cns3xxx_gpio_set_irq_type(pin,IRQ_TYPE_LEVEL_HIGH);
		} else if (strcmp(buf_param1,"5") == 0){
	 		sscanf(buf_param2, "%d", &pin);
			printk("IRQ_TYPE_LEVEL_LOW pin=%d \n",pin);
			cns3xxx_gpio_set_irq_type(pin,IRQ_TYPE_LEVEL_LOW);
		} else 
			printk("syntax: direct input(output) pin (state)\n");
		return count;
	}
#endif
	printk("syntax:\n1. direct input(output) pin (state)\n2. get(set) value pin (state)\n");
#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT
	printk("3. trig edge both pin\n4. trig edge single rising(falling) pin\n5. trig level high(low) pin\n");
#endif

	return count;
}

static int cns3xxx_request(struct gpio_chip *chip, unsigned offset)
{
	if ( 0 <= offset && offset <=31) {
		if (((CNS3XXX_GPIOA_PIN_DIS >> offset) & 1) == 1)
			return -EINVAL;
		else
			return 0;
	}
	else if ( 32 <= offset && offset <= 63) {
		if ( ((CNS3XXX_GPIOB_PIN_DIS >> (offset-32)) & 1) == 1)
			return -EINVAL;
		else
			return 0;
	}
	else
		return -EINVAL;
}

static void cns3xxx_free(struct gpio_chip *chip, unsigned offset)
{
}

static int cns3xxx_to_irq(struct gpio_chip *chip, unsigned offset)
{
	printk("xxx gpio to irq\n");
	if ( 0 <= offset && offset <=31)
		return IRQ_CNS3XXX_GPIOA;
	else if ( 32 <= offset && offset <= 63)
		return IRQ_CNS3XXX_GPIOB;
	else
		return -EINVAL;
}

static struct gpio_chip cns3xxx_gpiochip = {
	.label			= "cns3xxx_gpio",
	.owner			= THIS_MODULE,
	.request		= cns3xxx_request,
	.free			= cns3xxx_free,
	.direction_input	= cns3xxx_direction_in,
	.get			= cns3xxx_get,
	.direction_output	= cns3xxx_direction_out,
	.set			= cns3xxx_set,
	.to_irq			= cns3xxx_to_irq,
	.can_sleep		= 0,
};

static int cns3xxx_gpio_set_irq_type(u32 gpio_pin, u32 type)
{
    //u32 irq = gpio_to_irq(gpio_pin);

	switch (type) {
		case IRQ_TYPE_EDGE_RISING:
			cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_a_isr_test, PIN_TRIG_SINGLE, PIN_TRIG_RISING, gpio_pin);
			break;
		case IRQ_TYPE_EDGE_FALLING:
			cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_a_isr_test, PIN_TRIG_SINGLE, PIN_TRIG_FALLING, gpio_pin);
			break;
		case IRQ_TYPE_EDGE_BOTH:
			cns3xxx_gpio_set_edgeintr(&cns3xxx_gpio_a_isr_test, PIN_TRIG_BOTH, 0, gpio_pin);
			break;
		case IRQ_TYPE_LEVEL_LOW:
			cns3xxx_gpio_set_levelintr(&cns3xxx_gpio_a_isr_test, PIN_TRIG_HIGH, gpio_pin);
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			cns3xxx_gpio_set_levelintr(&cns3xxx_gpio_a_isr_test, PIN_TRIG_LOW, gpio_pin);
			break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void cns3xxx_gpio_ack(unsigned int irq)
{
	gic_ack_irq(irq);
}
static void cns3xxx_gpio_mask(unsigned int irq)
{
	gic_mask_irq(irq);
}
static void cns3xxx_gpio_unmask(unsigned int irq)
{
	gic_unmask_irq(irq);
}

static struct irq_chip cns3xxx_gpio_irq_chip = {
	.name = "CNS3XXXGPIO", 
        .ack = cns3xxx_gpio_ack,
        .mask = cns3xxx_gpio_mask,
        .unmask = cns3xxx_gpio_unmask,
        .set_type = cns3xxx_gpio_set_irq_type,
};

extern struct proc_dir_entry *cns3xxx_proc_dir;
static struct proc_dir_entry *proc_cns3xxx_gpio;

int __init gpio_init(void)
{
	int irq=0;
#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT
	u32 i, ret;
#endif
	printk("cns3xxx gpio initialize.");
	cns3xxx_pwr_clk_en(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);
	//cns3xxx_pwr_power_up(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);
	cns3xxx_pwr_soft_rst(0x1 << PM_CLK_GATE_REG_OFFSET_GPIO);

	proc_cns3xxx_gpio = create_proc_entry("gpio", S_IFREG | S_IRUGO, cns3xxx_proc_dir) ;
	proc_cns3xxx_gpio->read_proc = cns3xxx_gpio_read_proc;
	proc_cns3xxx_gpio->write_proc = cns3xxx_gpio_write_proc;

#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT
	for (i = 0; i < GPIOA_PIN_NO; i++) {
		gpio_a_isr[i] = NULL;
	}
	for (i = 0; i < GPIOB_PIN_NO; i++) {
		gpio_b_isr[i] = NULL;
	}

	/* Clear All Interrupt Status */
	__raw_writel(0xFFFFFFFF, CNS3XXX_GPIOA_BASE_VIRT + GPIO_INTERRUPT_CLEAR);
	__raw_writel(0xFFFFFFFF, CNS3XXX_GPIOB_BASE_VIRT + GPIO_INTERRUPT_CLEAR);


#ifdef CONFIG_CNS3XXX_GPIO_INTERRUPT_TEST

	ret = request_irq(IRQ_CNS3XXX_GPIOA, cns3xxx_gpio_irq_handler, 0, "cns3xxx_gpio_a", 0);
	if (ret < 0) {
		printk("request_irq fail : %d \n", ret);
		return 0;
	} else
		printk("GPIO interrupt handler install ok. \n");

	irq=IRQ_CNS3XXX_GPIOA;
	set_irq_chip(irq, &cns3xxx_gpio_irq_chip);
	set_irq_handler(irq, handle_level_irq);
	set_irq_flags(irq, IRQF_VALID | IRQF_PROBE);
#endif
#endif

	return 0;
}

static int __devinit gpio_cns3xxx_probe(struct platform_device *pdev)
{
	int ret;

	// ref include/asm-generic/gpio.h, 
	// @base: identifies the first GPIO number handled by this chip; or, if
	// negative during registration, requests dynamic ID allocation.
	cns3xxx_gpiochip.label="cns3xxx_gpio_chip";
	cns3xxx_gpiochip.base=-1; 
	cns3xxx_gpiochip.dev = &pdev->dev;
	cns3xxx_gpiochip.ngpio = CNS3XXX_GPIO_MAX;

	ret = gpiochip_add(&cns3xxx_gpiochip);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"could not register gpiochip, %d\n",
				ret);
	}
	return gpio_init();
}

#ifdef CONFIG_PM

static int gpio_cns3xxx_suspend(struct platform_device *dev, pm_message_t state)
{
	__pr_debug("%s,%s,%d\n", __FILE__, __FUNCTION__, __LINE__);

	return 0;
}

static int gpio_cns3xxx_resume(struct platform_device *dev)
{
	__pr_debug("%s,%s,%d\n", __FILE__, __FUNCTION__, __LINE__);

	return 0;
}

#else
#define gpio_cns3xxx_suspend	NULL
#define gpio_cns3xxx_resume	NULL
#endif /* CONFIG_PM */

static struct platform_driver gpio_cns3xxx_driver = {
	.probe		= gpio_cns3xxx_probe,

#ifdef CONFIG_PM
	.suspend	= gpio_cns3xxx_suspend,
	.resume		= gpio_cns3xxx_resume,
#endif /* CONFIG_PM */
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "cns3xxx-gpio",
	},
};

int __init cns3xxx_gpio_init(void)
{
	return platform_driver_probe(&gpio_cns3xxx_driver, gpio_cns3xxx_probe);
}	

void __exit cns3xxx_gpio_exit(void)
{
	free_irq(IRQ_CNS3XXX_GPIOA, 0);
}

module_init(cns3xxx_gpio_init);
module_exit(cns3xxx_gpio_exit);

MODULE_LICENSE("GPL");
