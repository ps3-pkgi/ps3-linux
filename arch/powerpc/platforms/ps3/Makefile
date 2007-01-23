obj-y += setup.o mm.o time.o hvcall.o htab.o repository.o
obj-y += interrupt.o exports.o os-area.o

obj-$(CONFIG_SMP) += smp.o
obj-$(CONFIG_SPU_BASE) += spu.o

obj-$(CONFIG_PPC_PS3_ROMBOOT) += dt.o

quiet_cmd_dtc = DTC     $@
      cmd_dtc = dtc -b 0 -O asm $< > $@.tmp && mv $@.tmp $@

$(obj)/dt.o: $(obj)/dt.S

$(obj)/dt.S: $(srctree)/arch/powerpc/boot/dts/ps3.dts
	$(call cmd,dtc)

clean-files += $(obj)/dt.S $(obj)/dt.S.tmp
