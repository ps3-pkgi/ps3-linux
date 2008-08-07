OUTPUT_ARCH(powerpc:common)
ENTRY(_zimage_start)
EXTERN(_zimage_start)
SECTIONS
{
  .kernel:vmlinux.bin :
  {
    _vmlinux_start =  .;
    *(.kernel:vmlinux.bin)
    _vmlinux_end =  .;
  }

  . = ALIGN(4096);
  .kernel:initrd :
  {
    _initrd_start =  .;
    *(.kernel:initrd)
    _initrd_end =  .;
  }

  . = ALIGN(4096);
  .text      :
  {
    _start = .;
    *(.text)
    *(.fixup)
    _etext = .;
  }

  . = ALIGN(4096);
  .data    :
  {
    *(.rodata*)
    *(.data*)
    *(.sdata*)
    *(.got2)
  }

  /DISCARD/ : { *(.dynstr .dynsym .hash .interp) }

  . = ALIGN(4);
  .dynamic :
  {
    __dynamic_start = .;
    *(.dynamic)
  }
  .rela.dyn : { *(.rela*) }

  . = ALIGN(8);
  .kernel:dtb :
  {
    _dtb_start = .;
    *(.kernel:dtb)
    _dtb_end = .;
  }

  . = ALIGN(4096);
  .bss       :
  {
    _edata  =  .;
    __bss_start = .;
    *(.sbss)
    *(.bss)
    _end = . ;
  }
}
