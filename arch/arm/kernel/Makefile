#
# Makefile for the linux kernel.
#

CPPFLAGS_vmlinux.lds := -DTEXT_OFFSET=$(TEXT_OFFSET)
AFLAGS_head.o        := -DTEXT_OFFSET=$(TEXT_OFFSET)

ifdef CONFIG_DYNAMIC_FTRACE
CFLAGS_REMOVE_ftrace.o = -pg
endif

CFLAGS_REMOVE_return_address.o = -pg

# Object file lists.

obj-y		:= compat.o elf.o entry-armv.o entry-common.o irq.o \
		   process.o ptrace.o return_address.o setup.o signal.o \
		   sys_arm.o stacktrace.o time.o traps.o

obj-$(CONFIG_LEDS)		+= leds.o
obj-$(CONFIG_OC_ETM)		+= etm.o

obj-$(CONFIG_ISA_DMA_API)	+= dma.o
obj-$(CONFIG_ARCH_ACORN)	+= ecard.o 
obj-$(CONFIG_FIQ)		+= fiq.o
obj-$(CONFIG_MODULES)		+= armksyms.o module.o
obj-$(CONFIG_ARTHUR)		+= arthur.o
obj-$(CONFIG_ISA_DMA)		+= dma-isa.o
obj-$(CONFIG_PCI)		+= bios32.o isa.o
obj-$(CONFIG_SMP)		+= smp.o
obj-$(CONFIG_AMP)		+= amp.o
obj-$(CONFIG_HAVE_ARM_SCU)	+= smp_scu.o
obj-$(CONFIG_HAVE_ARM_TWD)	+= smp_twd.o
obj-$(CONFIG_DYNAMIC_FTRACE)	+= ftrace.o
obj-$(CONFIG_KEXEC)		+= machine_kexec.o relocate_kernel.o
obj-$(CONFIG_KPROBES)		+= kprobes.o kprobes-decode.o
obj-$(CONFIG_ATAGS_PROC)	+= atags.o
obj-$(CONFIG_OABI_COMPAT)	+= sys_oabi-compat.o
obj-$(CONFIG_ARM_THUMBEE)	+= thumbee.o
obj-$(CONFIG_KGDB)		+= kgdb.o
obj-$(CONFIG_ARM_UNWIND)	+= unwind.o
obj-$(CONFIG_HAVE_TCM)		+= tcm.o

obj-$(CONFIG_CRUNCH)		+= crunch.o crunch-bits.o
AFLAGS_crunch-bits.o		:= -Wa,-mcpu=ep9312

obj-$(CONFIG_CPU_XSCALE)	+= xscale-cp0.o
obj-$(CONFIG_CPU_XSC3)		+= xscale-cp0.o
obj-$(CONFIG_CPU_MOHAWK)	+= xscale-cp0.o
obj-$(CONFIG_IWMMXT)		+= iwmmxt.o
obj-$(CONFIG_CPU_HAS_PMU)	+= pmu.o
obj-$(CONFIG_HW_PERF_EVENTS)	+= perf_event.o
AFLAGS_iwmmxt.o			:= -Wa,-mcpu=iwmmxt

ifneq ($(CONFIG_ARCH_EBSA110),y)
  obj-y		+= io.o
endif

head-y			:= head$(MMUEXT).o
obj-$(CONFIG_DEBUG_LL)	+= debug.o
obj-$(CONFIG_EARLY_PRINTK)	+= early_printk.o

extra-y := $(head-y) init_task.o vmlinux.lds
