#ifndef __LS1GPA_MMCI_H__
#define __LS1GPA_MMCI_H__

struct LS_MMCIState;
typedef struct LS_MMCIState LS_MMCIState;

void *ls_mmci_init(MemoryRegion *sysmem,
                       hwaddr base,
                       BlockBackend *blk, qemu_irq irq);

void ls_mmci_handlers(LS_MMCIState *s, qemu_irq readonly,
                          qemu_irq coverswitch);

void ls_sdio_set_dmaaddr(uint32_t val);
void ls_sdio_set_dmaaddr(int i, uint64_t val);

#endif
