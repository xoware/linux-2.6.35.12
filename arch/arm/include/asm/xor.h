/*
 *  arch/arm/include/asm/xor.h
 *
 *  Copyright (C) 2001 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <asm-generic/xor.h>

#define __XOR(a1, a2) a1 ^= a2

#define GET_BLOCK_2(dst) \
	__asm__("ldmia	%0, {%1, %2}" \
		: "=r" (dst), "=r" (a1), "=r" (a2) \
		: "0" (dst))

#define GET_BLOCK_4(dst) \
	__asm__("ldmia	%0, {%1, %2, %3, %4}" \
		: "=r" (dst), "=r" (a1), "=r" (a2), "=r" (a3), "=r" (a4) \
		: "0" (dst))

#define XOR_BLOCK_2(src) \
	__asm__("ldmia	%0!, {%1, %2}" \
		: "=r" (src), "=r" (b1), "=r" (b2) \
		: "0" (src)); \
	__XOR(a1, b1); __XOR(a2, b2);

#define XOR_BLOCK_4(src) \
	__asm__("ldmia	%0!, {%1, %2, %3, %4}" \
		: "=r" (src), "=r" (b1), "=r" (b2), "=r" (b3), "=r" (b4) \
		: "0" (src)); \
	__XOR(a1, b1); __XOR(a2, b2); __XOR(a3, b3); __XOR(a4, b4)

#define PUT_BLOCK_2(dst) \
	__asm__ __volatile__("stmia	%0!, {%2, %3}" \
		: "=r" (dst) \
		: "0" (dst), "r" (a1), "r" (a2))

#define PUT_BLOCK_4(dst) \
	__asm__ __volatile__("stmia	%0!, {%2, %3, %4, %5}" \
		: "=r" (dst) \
		: "0" (dst), "r" (a1), "r" (a2), "r" (a3), "r" (a4))

#ifdef AMP_RAID5
void xor_sw(int src_cnt, unsigned long bytes, void **srcs, void *dest)
{
	int i;
	int align;
	void *src[32];
	void *dst;
	int max_align = 0;

	for(i = 0; i < src_cnt; i++) {
		align = (unsigned long)srcs[i] % 4;
		if(align) {
                	src[i] = srcs[i] + (4 - align);
			if(align > max_align)
				max_align = align;
		}
		else
			src[i] = srcs[i];
	}
	align = (unsigned long)dst % 4;
	if(align) {
		dst = dest + (4 - align);
		if(align > max_align)
			max_align = align;
	}
	else
		dst = dest;
	bytes -= max_align;

	unsigned int lines = bytes / sizeof(unsigned long) / 4;
        register unsigned int a1 __asm__("r4");
        register unsigned int a2 __asm__("r5");
        register unsigned int a3 __asm__("r6");
        register unsigned int a4 __asm__("r7");
        register unsigned int b1 __asm__("r8");
        register unsigned int b2 __asm__("r9");
        register unsigned int b3 __asm__("ip");
        register unsigned int b4 __asm__("lr");

	do {
                GET_BLOCK_4((unsigned long)dest);
		for(i = 0; i < src_cnt; i++) 
                	XOR_BLOCK_4((unsigned long)src[i]);
                PUT_BLOCK_4(dst);
        } while (--lines);
}
EXPORT_SYMBOL(xor_sw);
#endif /* AMP_RAID5 */


static void
xor_arm4regs_2(unsigned long bytes, unsigned long *p1, unsigned long *p2)
{
	unsigned int lines = bytes / sizeof(unsigned long) / 4;
	register unsigned int a1 __asm__("r4");
	register unsigned int a2 __asm__("r5");
	register unsigned int a3 __asm__("r6");
	register unsigned int a4 __asm__("r7");
	register unsigned int b1 __asm__("r8");
	register unsigned int b2 __asm__("r9");
	register unsigned int b3 __asm__("ip");
	register unsigned int b4 __asm__("lr");

	do {
		GET_BLOCK_4(p1);
		XOR_BLOCK_4(p2);
		PUT_BLOCK_4(p1);
	} while (--lines);
}

static void
xor_arm4regs_3(unsigned long bytes, unsigned long *p1, unsigned long *p2,
		unsigned long *p3)
{
	unsigned int lines = bytes / sizeof(unsigned long) / 4;
	register unsigned int a1 __asm__("r4");
	register unsigned int a2 __asm__("r5");
	register unsigned int a3 __asm__("r6");
	register unsigned int a4 __asm__("r7");
	register unsigned int b1 __asm__("r8");
	register unsigned int b2 __asm__("r9");
	register unsigned int b3 __asm__("ip");
	register unsigned int b4 __asm__("lr");

	do {
		GET_BLOCK_4(p1);
		XOR_BLOCK_4(p2);
		XOR_BLOCK_4(p3);
		PUT_BLOCK_4(p1);
	} while (--lines);
}

static void
xor_arm4regs_4(unsigned long bytes, unsigned long *p1, unsigned long *p2,
		unsigned long *p3, unsigned long *p4)
{
	unsigned int lines = bytes / sizeof(unsigned long) / 2;
	register unsigned int a1 __asm__("r8");
	register unsigned int a2 __asm__("r9");
	register unsigned int b1 __asm__("ip");
	register unsigned int b2 __asm__("lr");

	do {
		GET_BLOCK_2(p1);
		XOR_BLOCK_2(p2);
		XOR_BLOCK_2(p3);
		XOR_BLOCK_2(p4);
		PUT_BLOCK_2(p1);
	} while (--lines);
}

static void
xor_arm4regs_5(unsigned long bytes, unsigned long *p1, unsigned long *p2,
		unsigned long *p3, unsigned long *p4, unsigned long *p5)
{
	unsigned int lines = bytes / sizeof(unsigned long) / 2;
	register unsigned int a1 __asm__("r8");
	register unsigned int a2 __asm__("r9");
	register unsigned int b1 __asm__("ip");
	register unsigned int b2 __asm__("lr");

	do {
		GET_BLOCK_2(p1);
		XOR_BLOCK_2(p2);
		XOR_BLOCK_2(p3);
		XOR_BLOCK_2(p4);
		XOR_BLOCK_2(p5);
		PUT_BLOCK_2(p1);
	} while (--lines);
}

static struct xor_block_template xor_block_arm4regs = {
	.name	= "arm4regs",
	.do_2	= xor_arm4regs_2,
	.do_3	= xor_arm4regs_3,
	.do_4	= xor_arm4regs_4,
	.do_5	= xor_arm4regs_5,
};

#ifdef CONFIG_CNS3XXX_RAID
extern void do_cns_rdma_xorgen(unsigned int src_no, unsigned int bytes,
			       void **bh_ptr, void *dst_ptr);
/*
 * We create these funcs/template just for benchmark reference.
 */
static void xor_cns_raid_2(unsigned long bytes, unsigned long *p1,
			   unsigned long *p2)
{
	void *src[2];

	src[0] = p2;
	src[1] = p1;
	do_cns_rdma_xorgen(2, bytes, src, (void *)p2);
}

static struct xor_block_template xor_block_cnsraid = {
	.name = "CNS-RAID",
#ifndef CONFIG_AMP
	.do_2 = xor_cns_raid_2,
#else
	.do_2   = xor_arm4regs_2,
	.do_3   = xor_arm4regs_3,
#endif
};
#endif /* CONFIG_CNS3XXX_RAID */

#undef XOR_TRY_TEMPLATES

#ifdef CONFIG_CNS3XXX_RAID
#define XOR_TRY_TEMPLATES                      \
       do {                                    \
               xor_speed(&xor_block_arm4regs); \
               xor_speed(&xor_block_8regs);    \
               xor_speed(&xor_block_32regs);   \
               xor_speed(&xor_block_cnsraid);  \
       } while (0)
#else
#define XOR_TRY_TEMPLATES			\
	do {					\
		xor_speed(&xor_block_arm4regs);	\
		xor_speed(&xor_block_8regs);	\
		xor_speed(&xor_block_32regs);	\
	} while (0)
#endif /* CONFIG_CNS3XXX_RAID */
