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
    {0x00300000, 0x00000000,   1,    MMU_CACHE_NONE},     /* SRAM */
    {0x00300000, 0x00300000, 256,    MMU_CACHE_NONE},     /* SRAM */
    {0x20000000, 0x20000000, 128,    MMU_CACHE_WTHROUGH}, /* SDRAM */
    {0x40000000, 0x40000000,   1,    MMU_CACHE_NONE},     /* Expansion CS0 */
    {0x50000000, 0x50000000,   1,    MMU_CACHE_NONE},     /* CF CE 1 */
    {0x60000000, 0x60000000,   1,    MMU_CACHE_NONE},     /* CF CE 1 */
    {0xf0000000, 0xf0000000, 256,    MMU_CACHE_NONE},     /* Internal regs */
    {0x00000000, 0x00000000,   0,    0}                   /* The end */
};

