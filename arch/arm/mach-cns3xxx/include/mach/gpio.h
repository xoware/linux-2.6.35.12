/*******************************************************************************
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

#ifndef	_CNS3XXX_GPIO_H_
#define	_CNS3XXX_GPIO_H_

#include <mach/cns3xxx.h>

// because asm-generic/gpio.h define ARCH_NR_GPIOS, 
// so define ARCH_NR_GPIOS must before asm-generic/gpio.h
#define ARCH_NR_GPIOS   (64)

#include <asm-generic/gpio.h>


#define GPIO_OUTPUT					0x00
#define GPIO_INPUT					0x04
#define GPIO_DIR					0x08
#define GPIO_BIT_SET					0x10
#define GPIO_BIT_CLEAR					0x14
#define GPIO_INTERRUPT_ENABLE				0x20
#define GPIO_INTERRUPT_RAW_STATUS			0x24
#define GPIO_INTERRUPT_MASKED_STATUS			0x28
#define GPIO_INTERRUPT_MASK				0x2C
#define GPIO_INTERRUPT_CLEAR				0x30
#define GPIO_INTERRUPT_TRIGGER_METHOD			0x34
#define GPIO_INTERRUPT_TRIGGER_BOTH_EDGES		0x38
#define GPIO_INTERRUPT_TRIGGER_TYPE			0x3C

#define GPIO_INTERRUPT_TRIGGER_METHOD_EDGE		0
#define GPIO_INTERRUPT_TRIGGER_METHOD_LEVEL		1
#define GPIO_INTERRUPT_TRIGGER_EDGE_SINGLE		0
#define GPIO_INTERRUPT_TRIGGER_EDGE_BOTH		1
#define GPIO_INTERRUPT_TRIGGER_TYPE_RISING		0
#define GPIO_INTERRUPT_TRIGGER_TYPE_FALLING		1
#define GPIO_INTERRUPT_TRIGGER_TYPE_HIGH		0
#define GPIO_INTERRUPT_TRIGGER_TYPE_LOW			1

#define MISC_REG_VALUE(offset) (*((volatile unsigned int *)(CNS3XXX_MISC_BASE_VIRT+offset)))
#define CNS3XXX_GPIOA_PIN_DIS MISC_REG_VALUE(0x14)
#define CNS3XXX_GPIOB_PIN_DIS MISC_REG_VALUE(0x18)


#define CNS3XXX_GPIO_REG(group,offset) \
	(__raw_readl( CNS3XXX_GPIOA_BASE_VIRT + ((CNS3XXX_GPIOB_BASE_VIRT-CNS3XXX_GPIOA_BASE_VIRT) * group) + offset))

//#define ARCH_NR_GPIOS   (256 + CONFIG_S3C24XX_GPIO_EXTRA) // ref: arch/arm/mach-s3c2410/include/mach/gpio.h
#define gpio_get_value        __gpio_get_value
#define gpio_set_value        __gpio_set_value
#define gpio_cansleep         __gpio_cansleep
#define gpio_to_irq           __gpio_to_irq         



#if 0
int gpio_get_value(unsigned int pin);
void gpio_set_value(unsigned int pin, unsigned int state);
int gpio_to_irq(unsigned int pin);
int irq_to_gpio(unsigned int irq);
int __init_or_module gpio_direction_input(unsigned int pin);
int __init_or_module gpio_direction_output(unsigned int pin, unsigned int state);

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}
#endif

#endif
