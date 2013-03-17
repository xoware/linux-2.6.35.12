#ifndef  VSC7385_H
#define  VSC7385_H

int vsc7385_reg_read(u32 block, u32 subblock, u32 addr, u32 *value);
int vsc7385_reg_write(u32 block, u32 subblock, u32 addr, u32 value);

/* VSC7385 register addressing */
#define BLOCK_SYSTEM       7
#define SUBBLOCK_SYSTEM    0
#define ADDRESS_GMIIDELAY  5
#define ADDRESS_ICPU_CTRL  0x10
#define ADDRESS_ICPU_ADDR  0x11
#define ADDRESS_ICPU_DATA  0x12
#define ADDRESS_GLORESET   0x14

#define BLOCK_MEMINIT      3
#define SUBBLOCK_MEMINIT   2
#define ADDRESS_MEMINIT    0

#define BLOCK_FRAME_ANALYZER    2
#define SUBBLOCK_FRAME_ANALYZER 0
#define ADDRESS_RECVMASK        0x10
#define ADDRESS_MACACCESS       0xb0
#define ADDRESS_VLANACCESS      0xe0

#define BLOCK_MAC          1
#define ADDRESS_MAC_CFG    0
#define ADDRESS_ADVPORTM   0x19

/* ethernet modes */
enum { LinkDown, Link1000Full, Link100Full, Link10Full, Link100Half, Link10Half };

#define MAC_RESET     0x20000030
#define MAC_1000_FULL 0x10070180
#define MAC_100_FULL  0x10050440
#define MAC_10_FULL   0x10050440
#define MAC_100_HALF  0x90010440
#define MAC_10_HALF   0x90010440

#define MAC_EXT_CLK_1000 0x1
#define MAC_EXT_CLK_100  0x2
#define MAC_EXT_CLK_10   0x3
#define MAC_INT_CLK      0x4

#endif  // VSC7385_H
