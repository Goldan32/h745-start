INCLUDE link.x;


/* The location of the stack can be overridden using the
   `_stack_start` symbol.  Place the stack at the end of RAM */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

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

    .sram3 (NOLOAD) : ALIGN(4) {
        *(.sram3 .sram3.*);
        . = ALIGN(4);
    } > SRAM3

    .sram4 (NOLOAD) : ALIGN(4) {
        *(.sram4 .sram4.*);
        . = ALIGN(4);
    } > SRAM4

    .flash1 : ALIGN(4) {
        LONG(_stack_start);
        KEEP(*(.flash1.reset_vector));
        KEEP(*(.flash1.vector_table));
        *(.flash1 .flash1.*);
        . = ALIGN(4);
    } > FLASH1
};
