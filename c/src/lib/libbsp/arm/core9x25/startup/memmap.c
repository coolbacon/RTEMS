/*
    core9x25 for RTEMS.CN' at91sam9x25 board or at91sam9g25.
    baconxu@gmail.com
*/

#include <rtems.h>
#include <libcpu/mmu.h>

#include <bspopts.h>


mmu_sect_map_t mem_map[] = {
/*  <phys addr>  <virt addr> <size> <flags> */
    {0x20000000, 0x00000000,   1,    MMU_CACHE_NONE},     /* SRAM */
    {0x00100000, 0x00100000,   1,    MMU_CACHE_NONE},     /* ROM  */
    {0x00300000, 0x00300000,   1,    MMU_CACHE_NONE},     /* SRAM */
    {0x10000000, 0x10000000,   1,    MMU_CACHE_NONE},     /* ebi cs0*/
    {0x20000000, 0x20000000, 128,    MMU_CACHE_WTHROUGH}, /* SDRAM */
    {0x30000000, 0x30000000,   1,    MMU_CACHE_NONE},     /* ebi cs2*/
    {0x40000000, 0x40000000,   1,    MMU_CACHE_NONE},     /* ebi cs3*/
    {0x50000000, 0x50000000,   1,    MMU_CACHE_NONE},     /* ebi cs4*/
    {0x60000000, 0x60000000,   1,    MMU_CACHE_NONE},     /* ebi cs5*/
    {0xf0000000, 0xf0000000, 256,    MMU_CACHE_NONE},     /*internal register*/
    {0x00000000, 0x00000000,   0,    0}                   /* The end */
};

