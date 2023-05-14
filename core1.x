INCLUDE link.x

_cpu2_stack_start = ORIGIN(SRAM2) + LENGTH(SRAM2);

SECTIONS
{
    .itcmram : ALIGN(4) {
        *(.itcmram .itcmram.*);
        . = ALIGN(4);
    } > ITCMRAM

    .dtcmram : ALIGN(4) {
        *(.dtcmram .dtcmram.*);
        . = ALIGN(4);
    } > DTCMRAM

    .shared : ALIGN(8) {
        KEEP(microamp-data.o(.shared));
        . = ALIGN(8);
    } > AXISRAM

    /* The SRAM1 and SRAM2 section are commonly used as the stack and
       heap for the CM4 core in dualcore versions and should thus not
       be used in examples */

    .sram1 (NOLOAD) : ALIGN(4) {
        *(.sram1 .sram1.*);
        . = ALIGN(4);
    } > SRAM1

    .sram2 (NOLOAD) : ALIGN(4) {
        *(.sram2 .sram2.*);
        . = ALIGN(4);
    } > SRAM2

    .sram3 (NOLOAD) : ALIGN(4) {
        *(.sram3 .sram3.*);
        . = ALIGN(4);
    } > SRAM3

    .sram4 (NOLOAD) : ALIGN(4) {
        *(.sram4 .sram4.*);
        . = ALIGN(4);
    } > SRAM4

    .flash2 : ALIGN(4) {
        LONG(_cpu2_stack_start);
        KEEP(*(.flash2.reset_vector));
        KEEP(*(.flash2.vector_table));
        *(.flash2 .flash2.*);
        . = ALIGN(4);
    } > FLASH2
};
