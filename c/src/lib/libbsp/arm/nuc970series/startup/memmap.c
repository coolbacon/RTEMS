/*
at91sam9x25

baconxu@gmail.com

*/

#include <rtems.h>
#include <libcpu/mmu.h>

#include <bspopts.h>

/* Remember, the ARM920 has 64 TLBs. If you have more 1MB sections than
 * that, you'll have TLB lookups, which could hurt performance.
 */
mmu_sect_map_t mem_map[] = {
/*  <phys addr>  <virt addr> <size> <flags> */
    {0x00000000, 0x00000000,  64,    MMU_CACHE_WTHROUGH}, 	/* SDRAM */
    {0x20000000, 0x20000000, 256,    MMU_CACHE_NONE},     	/* Internal regs */
    {0x3C000000, 0x3C000000,   1,    MMU_CACHE_NONE},     	/* Internal SRAM */
    {0xA0000000, 0xA0000000, 256,    MMU_CACHE_NONE},     	/* device */
    {0xB0000000, 0xB0000000, 256,    MMU_CACHE_NONE},     	/* Internal regs */
	{0xF0000000, 0xF0000000, 256,    MMU_CACHE_NONE},     	/* Internal Boot ROM */
    {0x00000000, 0x00000000,   0,    0}                   	/* The end */
};

