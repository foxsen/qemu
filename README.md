支持龙芯主板模拟的QEMU
QEMU with added loongson platforms support.

目前支持龙芯2H的开发板。关于2H开发板的更多信息，可以从下面的链接获取。
Now it supports Loongson-2H SoC development board. More information on this board: http://www.loongson.cn/product/system/62.html and http://wiki.loongnix.org/index.php/2HSoc%E5%BC%80%E5%8F%91%E6%9D%BF (with pmon/kernel binary and sources).

目前状态:
Current Status:
  模拟了大部分设备，能够启动官方提供的pmon。模拟并不详尽，还需要更多工作以支持其它配置的pmon以及启动操作系统。将提供技术文档进行更详细的说明。
  The hardware is emulated up to the point of booting the official pmon binary. more testing and detail emulation needed to boot the os.

用法：
  1, 下载编译。使用mips64el-softmmu配置。编译依赖的环境同官方qemu。
  git clone https://github.com/foxsen/qemu.git
  cd qemu.git
  git checkout loongson
  mkdir build
  cd build
  ../configure --enable-debug --enable-trace-backends=simple --target-list=mips64el-softmmu
  (--enable-debug和--enable-trace-backends=simple并非必须，只是当前阶段便于调试)

   2, 准备pmon和系统。
   2.1 pmon
    wget http://ftp.loongnix.org/firmware/pmon/release/2HSoc/bin/gzrom2hsoc20141016v0.6.bin
    dd if=/dev/zero of=flash.img bs=32k count=32
    dd if=./gzrom2hsoc20141016v0.6.bin of=flash.img conv=ontrunc

## 选择ls2h机器，指定上述做好的bios映像文件，启动
    ./build/mips64el-softmmu/qemu-system-mips64el -machine ls2h -pflash ./flash.img

    如果在linux图形界面下，而且qemu编译了gtk等支持，则会出现一个模拟器窗口。可以在view菜单下选择输出界面（ls2h-dc,compatmonitor0,seriali0等)。ls2h-dc是被模拟的龙芯机器的图形显示输出，compatmonitor0是qemu的监控界面，可以用来查询被模拟机器的各种信息，以及配置动态的设备，比如usb等），serial0是串口输出。启动初期显卡初始化没完成时可以先看串口输出。
    图形界面下鼠标可能被模拟器窗口捕获，需要按ctrl+alt+G释放。

   2.2 系统
     TODO

TODO:
    * finish interrupt emulation.
    * fix usb bug. when ohci is enabled, pmon driver will stuck in hc_interrupt(after keyboard initialized) with tlb exception.
    * test sata emulation.
    * implement cpu address window registers. That is, writes to CPU window registers will really work.
    * implement PCI-E.
    * test network emulation.
    * boot the linux & VxWorks OS.
