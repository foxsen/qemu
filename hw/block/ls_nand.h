#ifndef __LS_NAND_H_
#define __LS_NAND_H_

void ls_nand_set_dmaaddr(uint64_t val);
static DeviceState *nand_init(BlockBackend *blk, int manf_id, int chip_id);
static void nand_setpins(DeviceState *dev, uint8_t cle, uint8_t ale,
                         uint8_t ce, uint8_t wp, uint8_t gnd);
#define nand_getpins ls_nand_getpins
#define nand_setio ls_nand_setio
#define nand_getio ls_nand_getio
#define nand_getbuswidth ls_nand_getbuswidth
void nand_getpins(DeviceState *dev, int *rb);
void nand_setio(DeviceState *dev, uint32_t value);
uint32_t nand_getio(DeviceState *dev);
uint32_t nand_getbuswidth(DeviceState *dev);

#endif
