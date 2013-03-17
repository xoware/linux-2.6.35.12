/*
 * Copyright 2008 Cavium Networks
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, Version 2, as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_BOARD_CNS3XXXH_SUM
#define __MACH_BOARD_CNS3XXXH_SUM

#ifdef CONFIG_PAGE_SIZE_64K
#include <mach/cns3xxx-64k.h>
#else
#include <mach/cns3xxx-4k.h>
#endif

#endif

//KH: 20110628 add for the SATA stagger support
//    
// -------------------------------------------------
#define STAGGER_SATA_PORT_DISKS 5
#define STAGGER_DELAY   2000
// -------------------------------------------------
