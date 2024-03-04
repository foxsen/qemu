#define LOONGARCH_CSR_CRMD		0x0	/* Current mode info */
#define LOONGARCH_CSR_KS4		0x34
#define PERCPU_BASE_KS			LOONGARCH_CSR_KS4
#define LOONGARCH_CSR_DMWIN0		0x180	/* 64 direct map win0: MEM & IF */
#define LOONGARCH_CSR_DMWIN1		0x181	/* 64 direct map win1: MEM & IF */
#define LOONGARCH_CSR_DMWIN2		0x182	/* 64 direct map win2: MEM */
#define LOONGARCH_CSR_DMWIN3		0x183	/* 64 direct map win3: MEM */

#define DMW_PABITS	48
/* Direct map window 0/1 */
#define CSR_DMW0_PLV0		(1 << 0)
#define CSR_DMW0_VSEG		(0x8000)
#define CSR_DMW0_BASE		(CSR_DMW0_VSEG << DMW_PABITS)
#define CSR_DMW0_INIT		(CSR_DMW0_BASE | CSR_DMW0_PLV0)

#define CSR_DMW1_PLV0		(1 << 0)
#define CSR_DMW1_MAT		(1 << 4)
#define CSR_DMW1_VSEG		(0x9000)
#define CSR_DMW1_BASE		(CSR_DMW1_VSEG << DMW_PABITS)
#define CSR_DMW1_INIT		(CSR_DMW1_BASE | CSR_DMW1_MAT | CSR_DMW1_PLV0)

.macro PCI_CONFIG_WRITE_DEV dev func mem int
li.d $t0, 0x800000fe00000000|(\dev<<11)|(\func<<8);
li.w $t1, \mem;
st.w $t1, $t0, 0x10;
li.w $t1, \int;
st.h $t1, $t0, 0x3c;
li.w $t1, 7;
st.w $t1, $t0, 0x4;
.endm


.macro PCI_CONFIG_WRITE_BRG dev func memstart memend iostart ioend int pri sec sub
li.d $t0, 0x800000fe00000000|(\dev<<11)|(\func<<8);
li.w $t1, ((\memstart>>16)&0xfff0)|(\memend&0xfff00000);
st.w $t1, $t0, 0x20;
st.w $t1, $t0, 0x24;
li.w $t1, ((\iostart>>16)&0xff)|(\ioend&0xff0000);
st.w $t1, $t0, 0x30;
li.w $t1, ((\iostart>>8)&0xf0)|(\ioend&0xf0)|0x101;
st.w $t1, $t0, 0x1c;
li.w $t1, \pri |(\sec << 8)|(\sub << 16);
st.w $t1, $t0, 0x18;
li.w $t1, 7;
st.w $t1, $t0, 0x4;
.endm


.text
.global _start;
_start:
bl	1f
nop
 .dword 0
 .dword 0
1:
	# Config direct window and set PG
	# 0x8000 xxxx xxxx xxxx
	li.d	$t0, CSR_DMW0_INIT	# UC, PLV0
	csrwr	$t0, LOONGARCH_CSR_DMWIN0
	# 0x9000 xxxx xxxx xxxx
	li.d	$t0, CSR_DMW1_INIT	# CA, PLV0
	csrwr	$t0, LOONGARCH_CSR_DMWIN1
	li.d    $t0, CSR_DMW1_BASE
	move    $t1, $ra
	bl      1f
1:
	or $ra, $ra, $t0
	jirl $zero, $ra, 8
	or    $ra, $t1, $t0
	# Enable PG
	li.w	$t0, 0xb0	# PLV=0, IE=0, PG
	csrwr	$t0, LOONGARCH_CSR_CRMD

	csrwr	$zero, PERCPU_BASE_KS

       csrrd   $t2, 0x20
       andi    $t2, $t2, 0x3ff
	ld.b $t1, $ra, 12

 beq $t1, $t2, core0_start
########
#define FN_OFF 0x020
#define SP_OFF 0x028
#define GP_OFF 0x030
#define A1_OFF 0x038

 slli.d     $t2, $t2, 8
 li.d    $t0, 0x800000001fe01000

 or      $t0, $t0, $t2


 bl waitforinit

waitforinit:

 ld.w      $t1, $t0, FN_OFF
 bnez    $t1, 1f
 jr $ra
 nop
1:
 ld.d      $ra, $t0, FN_OFF
 ld.d      $sp, $t0, SP_OFF
 ld.d      $r2, $t0, GP_OFF
 ld.d      $a1, $t0, A1_OFF
 jirl    $ra, $ra, 0

####
 1:  b   1b

core0_start:

PCI_CONFIG_WRITE_DEV 2,0,0x1fe20000,0
PCI_CONFIG_WRITE_DEV 3,0,0x40040000,20
PCI_CONFIG_WRITE_DEV 3,1,0x40050000,22
PCI_CONFIG_WRITE_DEV 4,0,0x40000000,57
PCI_CONFIG_WRITE_DEV 4,1,0x40060000,58
PCI_CONFIG_WRITE_DEV 4,2,0x40070000,59
PCI_CONFIG_WRITE_DEV 5,0,0x40080000,37
PCI_CONFIG_WRITE_DEV 6,0,0x400c0000,36
PCI_CONFIG_WRITE_DEV 7,0,0x400d0000,12
PCI_CONFIG_WRITE_DEV 8,0,0x400e0000,27
PCI_CONFIG_WRITE_BRG 9,0,0x40100000,0x4fffffff,0x18000000,0x180fffff,40,0,1,1
PCI_CONFIG_WRITE_BRG 10,0,0x50000000,0x53ffffff,0x18100000,0x181fffff,41,0,4,4
PCI_CONFIG_WRITE_BRG 11,0,0x54000000,0x57ffffff,0x18200000,0x182fffff,42,0,8,8
PCI_CONFIG_WRITE_BRG 12,0,0x58000000,0x5fffffff,0x18300000,0x183fffff,43,0,12,12
PCI_CONFIG_WRITE_BRG 13,0,0x60000000,0x77ffffff,0x18400000,0x184fffff,44,0,16,16
PCI_CONFIG_WRITE_BRG 14,0,0x78000000,0x7fffffff,0x18500000,0x185fffff,45,0,16,16
ld.d $ra, $ra, 4
jirl $zero, $ra, 0
nop

