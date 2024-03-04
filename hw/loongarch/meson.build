loongarch_ss = ss.source_set()
loongarch_ss.add(files(
    'fw_cfg.c',
))
loongarch_ss.add(when: 'CONFIG_LOONGARCH_VIRT', if_true: [files('virt.c'), fdt])
loongarch_ss.add(when: 'CONFIG_ACPI', if_true: files('acpi-build.c'))
loongarch_ss.add(when: 'CONFIG_LOONGARCH_2k1000', if_true: [files('2k1000.c')])

hw_arch += {'loongarch': loongarch_ss}
