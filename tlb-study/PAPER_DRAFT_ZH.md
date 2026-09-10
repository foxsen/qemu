# 现代全系统模拟器中访存通路的开销分析与保守优化

## ——以 QEMU 8.2 SoftMMU 为例

> 中文论文草稿 v0.1，2026-09-10
>
> 作者与单位：待补
>
> 状态说明：本文使用当前仓库中已经完成并通过现有校验的数据。应用负载的优化
> 对比为 3 次重复（stress-ng 因双峰补到 5 次），宿主机尚未离线 SMT sibling，
> 因此文中将其解释为阶段性趋势而非统计显著结论。用户所写的
> `hpca2027-paper64.pdf` 不在本机目录；实际存在且与 TLB 相关的是 14 页的 HPCA
> 2027 #54 *Bifrost* 匿名稿，第 7 节已按其表 III 核验扩展 workload。

## 摘要

全系统动态二进制翻译器需要在每条客户机访存指令上完成客户机虚拟地址到宿主机
地址的转换。软件转换后备缓冲器（SoftTLB）将常见转换内联到动态生成代码中，但
主表冲突、页表失效和大页覆盖不足仍会把部分访问送入代价较高的慢路径。已有工作
在早期 QEMU 上报告，访存模拟平均占总执行时间的 38.1%，针对 SoftTLB 查询与填充
的组合优化可获得 24.4% 的平均性能提升。然而，现代 QEMU 已引入可动态伸缩的直接
映射主表和全相联 victim TLB，十年前的瓶颈及收益不能直接外推。

本文以 QEMU 8.2.9 的 x86-64 system mode 为对象，建立一套将无插桩时间采样、慢路径
事件计数和客户机访存精确分母相互分离的测量方法，并在微基准、sysbench、
stress-ng、DaCapo、嵌套 QEMU、SPEC CPU2006 `429.mcf` 和 GAPBS PageRank 上刻画
访存通路。结果表明，六类应用负载中的 data refill 仅占客户机访存次数的
0.0246%--0.4948%，但可命名 SoftMMU 慢路径仍占宿主采样周期的 1.17%--16.84%。
固定主表为 4096 项并关闭 victim 后，六个系统负载的可命名慢路径平均为 13.39%，
最高 `429.mcf` 为 24.18%，仍未接近早期论文的完整访存模拟平均 38.1%。由于当前
分类不含 JIT 内联 fast-hit lookup，这个差异不能解释为现代 QEMU 已消除全部开销。

基于该观察，本文实现两种不改动生成代码公共命中路径的保守优化：其一是在主表和
victim TLB 均未命中后查询 128 项大页转换缓存，并仍通过原有填充逻辑安装普通
4 KiB SoftTLB 项；其二是在 x86 页表遍历中缓存 L2--L4 非叶转换，使后续遍历可跳过
若干层级。在固定 4096 项主表、关闭 victim、P-Core 锁定 2.1 GHz 且去除 timing
路径原子计数后，大页 cache 在 THP 随机微基准达到 1.399 倍，PTW cache 在 4 KiB
随机微基准达到 1.066 倍。sysbench 和 GAPBS 基本无变化；`429.mcf` 的 LP/both
出现约 6%--7% 正信号，nested 热点窗口达到约 2.2 倍但端到端仅约 1.05 倍。
阶段性结果表明，面向现代全系统模拟器的优化不应只追求单次 refill 更快，而应同时
解决软件查询成本、workload 选择、自适应启停和公共 fast path 成本四个问题。

**关键词：** 全系统模拟；动态二进制翻译；QEMU；SoftMMU；软件 TLB；页表遍历；
大页；性能分析

## 1 引言

全系统模拟器能够在没有目标硬件的条件下运行完整操作系统，因而广泛用于体系结构
研究、操作系统开发、设备模型验证和跨指令集软件支持。与仅模拟用户态 ABI 的进程
级动态二进制翻译不同，system mode 必须忠实处理客户机页表、特权状态、MMIO、脏页、
观察点和 TLB shootdown。访存不仅出现频率高，而且横跨动态生成代码、目标架构 MMU
和 QEMU 地址空间三个层次，是影响模拟速度和正确性的关键通路。

QEMU 的 TCG 后端将常见 SoftTLB 查询直接生成为宿主代码：地址命中时，只需完成索引、
tag 比较和 addend 加法；查询失败时则离开生成代码，依次检查 victim TLB、调用目标
架构的 TLB fill、遍历客户机页表，并经由 QEMU MemoryRegion 完成客户机物理地址到
宿主虚拟地址或 MMIO 路径的解析。因此，访存优化存在一个基本矛盾：公共命中路径
覆盖几乎所有访问，但增加一条指令也可能造成广泛退化；慢路径单次代价高且更容易
安全扩展，但它在现代 QEMU 中可能已经十分少见。

Tong 等人在 QEMU 1.7.0 上测得访存模拟平均占总时间的 38.1%，并通过 SoftTLB
容量、victim、关联度、大页和 refill 等组合优化获得 24.4% 的平均模拟器加速 [1]。
此后 QEMU 主线已采用动态伸缩的直接映射主表，并配置 8 项全相联 victim TLB。
由此产生本文的核心问题：在现代实现中过滤掉大量 miss 后，剩余慢路径是否仍值得
优化；如果值得，怎样在不增加每次访存公共成本的前提下利用大页和页表层级局部性；
以及微基准上的机制收益能否转化为应用级端到端加速。

本文围绕以下研究问题展开：

- **RQ1：** 现代 QEMU system mode 中，SoftTLB miss、victim 命中、refill 和页表
  遍历的事件频率与时间占比如何？
- **RQ2：** 在不修改生成代码 fast path 的约束下，大页转换复用和非叶页表转换
  缓存能否减少慢路径工作量？
- **RQ3：** 定向微基准上的收益能否在真实应用和嵌套虚拟化场景中转化为端到端收益？
- **RQ4：** 当应用级收益不明显时，瓶颈来自机制无效、覆盖率不足，还是实验负载与
  测量设计不足？

本文目前形成四点阶段性贡献：

1. 建立三口径分离的可复现实验框架。无插桩构建用于墙钟和 `perf` 时间归因，profile
   构建用于 miss/refill/PTW 事件计数，TCG plugin 用于获得测量窗口内客户机访存的
   精确分母，避免用带原子计数器的运行时间评价性能。
2. 给出现代 QEMU 8.2 SoftMMU 的量化画像。应用级 data refill 比例低于 0.5%，但
   可命名慢路径最高仍占 16.84% 的宿主周期；victim recovery 在不同负载间从
   8.95% 到 88.00%，表明冲突和局部性具有显著 workload 依赖性。
3. 实现两个 miss-only 原型，并明确一个容易被忽略的正确性边界：用于失效覆盖的
   页大小与可线性重建物理地址的转换范围并不总是相同，尤其在嵌套翻译中必须分别
   建模。
4. 报告并分析真实负载中的弱结果。两种方案可显著加速定向微基准，却尚未带来稳定
   应用级收益。这一负结果为后续 workload 筛选、自适应缓存和 fast-path 优化提供了
   更具体的设计约束。

## 2 背景与动机

### 2.1 system mode 访存通路

图 1 给出本文关注的 QEMU TCG system-mode 访存路径。实线表示一次客户机访存的正常
控制流，虚线表示优化原型新增的 miss-path 旁路。

```text
客户机 load/store/fetch
          |
          v
  生成代码中的主 SoftTLB 查询 ----命中----> tag/addend/权限检查 ---> RAM
          |
         miss
          v
      8-entry victim TLB --------命中----> 回填主表 -----------> RAM
          |
         miss
          +............... 大页转换缓存 ...............+
          |                                             |
          v                                             v
   target TLB fill / x86 页表遍历 <.... 非叶 PTW 缓存   |
          |                                             |
          v                                             |
  guest PA -> MemoryRegion -> host VA / MMIO / ROMD     |
          |                                             |
          +---------- tlb_set_page_full() <-------------+
                         |
                  安装普通 4 KiB 项
```

**图 1  QEMU system-mode 访存路径与本文优化位置。**

主表命中路径由 TCG 生成代码直接执行，布局和指令数都高度敏感。`CPUTLBEntry` 保存
fast path 需要的比较字段和 addend，而权限、MemoryRegion、物理地址及目标架构附加
状态位于不进入公共生成代码布局的 `CPUTLBEntryFull` 中。主表 miss 后，QEMU 首先
检查全相联 victim TLB；只有二者都 miss，才调用目标架构 `tlb_fill`。对 x86 客户机，
该过程通常触发多级基数页表遍历；随后 `tlb_set_page_full()` 解析 RAM、ROMD、MMIO、
脏页和 watchpoint 语义，并向主表安装一项转换。

### 2.2 现代 QEMU SoftTLB 的容量与冲突

QEMU 8.2 的每个 MMU mode 主表初始为 256 项，最小 64 项；在本实验的 64 位 x86
构建中，编译期上限为 $2^{22}$ 项。主表保持直接映射，并且只在 TLB flush 时根据
过去窗口的占用率伸缩：占用超过 70% 时扩大一倍，低于 30% 且 100 ms 窗口到期时
缩小。该策略避免了运行中重哈希，却意味着编译期容量上限并不等同于任意时刻的实际
容量。过大的表还会增加 flush 延迟并降低宿主 cache 局部性。

本实验六类应用负载结束时，每个 MMU mode 的最大实际表容量为 1,024--16,384 项，
远低于编译期上限。与此同时，DaCapo 和嵌套 QEMU 的 victim recovery 均超过 83%。
这一结果说明，现代动态容量确实显著过滤了 miss，但直接映射冲突仍然存在，且不能
只根据最大容量推断慢路径已经消失。

### 2.3 大页覆盖与页表遍历

QEMU 8.2 的主 SoftTLB 项不直接覆盖一个完整客户机大页。即使目标 MMU 返回 2 MiB
或 1 GiB 页，正常填充仍只安装一个 `TARGET_PAGE_SIZE` 区间；较大的页大小主要用于
保证失效覆盖的保守性。因此，同一个 2 MiB 映射中的不同 4 KiB 子页仍可能反复进入
refill。另一方面，大页叶节点位于更高页表层级，本身可以少访问一级或多级页表。
“减少每次 walk 的层数”和“减少 refill 次数”是两个不同的效应，必须分别测量。

硬件 MMU cache 已证明缓存部分转换可以在 TLB miss 后跳过页表层级 [3]。但软件
实现的查找、key 比较和一致性维护也消耗宿主指令，且 QEMU 慢路径事件远少于硬件
访存事件，因此硬件结论不能直接证明软件 PTW cache 有利可图。

### 2.4 优化边界：失效范围不等于线性转换范围

大页缓存若要从已知子页转换合成相邻子页的物理地址，需要满足该区间内
$p = p_{base} + (v-v_{base})$。原 QEMU 的 `lg_page_size` 同时参与大页失效处理；
在 x86 嵌套翻译中，为保证任一阶段失效都能覆盖已安装项，该值可能取两阶段页大小
的较大者。然而，两阶段组合后可线性复用的区间只能取较小者。若直接把前者用于地址
重建，就可能把相邻客户机虚拟页映射到错误物理页。

本文因而新增 `lg_translation_size`，令

$$
S_{invalidate}=\max(S_{stage1},S_{stage2}),\qquad
S_{linear}=\min(S_{stage1},S_{stage2}).
$$

前者继续服务于保守失效，后者单独限定大页缓存能够重建地址的范围。这一分离既是
原型正确性的关键，也是一条可推广的设计原则：缓存覆盖范围必须由转换语义决定，
不能由失效实现中的保守近似代替。

## 3 测量方法

### 3.1 三类证据相互分离

本文将性能、机制和分母证据分为三种独立运行：

1. **timing/perf 运行。** 使用不含 `QEMU_TLB_PROFILE` 的 QEMU 构建测量墙钟时间、
   workload 自报吞吐和宿主 `perf` 周期。`perf` 中的 named SoftMMU 由 refill、PTW、
   victim、原子访存、TLB 维护及相关支持函数构成。
2. **profile 运行。** 使用带 `QEMU_TLB_PROFILE` 的构建记录主表 miss、victim hit、
   fill call、安装页大小、调用来源、访问类型、primary/NPT walk 及各页表层级访问。
   这些原子计数会扰动执行时间，因此不把该运行的时间与 baseline 比较。
3. **plugin-window 运行。** TCG plugin 在客户机 marker 标记的开始/结束 PC 之间统计
   全部动态访存操作，提供 `miss / guest memory operation` 的精确分母。plugin 每次
   访存都增加宿主侧计数，也不用于时间对比。

这种设计避免了两个常见误区。第一，客户机微基准显式执行的 load 数不是系统内全部
访存的分母，因为页表遍历、指令取值和操作系统活动也会产生访问。第二，`perf` 能
识别慢路径函数，却不能把内联在 JIT 代码中的 fast-hit 指令从客户机指令主体中完全
分离。因此，本文的 named SoftMMU 是“可归因慢路径占比”，不是全部访存模拟时间。

### 3.2 事件定义与一致性检查

本文采用以下定义：

- `L1 miss`：直接映射主 SoftTLB 查询失败；
- `victim hit`：主表 miss 后由 8 项 victim TLB 恢复；
- `refill`：主表和 victim 均 miss 后调用 target MMU fill；
- `PTW`：x86 `mmu_translate()` 执行的一次 primary 或 nested page walk；
- `level visit`：一次 walk 对某一级页表项的访问。

除异常原子访问外，应近似满足

$$
N_{refill}=N_{L1\ miss}-N_{victim\ hit}.
$$

当前结果集中 134 个 origin/access 组合的 balance residual 均为 0。分析脚本还检查
workload 远端返回值、marker 闭合、QEMU 版本、`perf` 样本来源、输入与二进制哈希，
防止把失败运行、不同原型或插桩时间误合并到同一张表。

### 3.3 工作负载

现有工作负载覆盖五类行为：

| 类别 | 工作负载 | 主要目的 |
|---|---|---|
| 定向微基准 | 128 MiB random/dense，4 KiB 或 2 MiB THP | 控制页覆盖、访问局部性和 miss 强度 |
| 内存与失效压力 | sysbench random 4 KiB write；stress-ng `tlb-shootdown` | 连续访存与频繁映射失效 |
| 托管运行时 | DaCapo 9.12 `avrora` | JVM、代码与数据混合行为 |
| 应用与图计算 | SPEC CPU2006 `429.mcf` train；GAPBS PR scale-20，8 trials | 大工作集、指针访问和热身效应 |
| 二阶段翻译 | 外层 QEMU/TCG 运行内层 QEMU/KVM random-page | primary walk、NPT 和端到端稀释 |

此外，一次从全新源码树开始的 Linux 3.12.9 `make -j1` 已成功完成：墙钟
14,674.95 s、QEMU CPU 14,646.13 s、named SoftMMU 7.89%。该结果补充了长时间
真实系统负载的开销画像，但当前优化 A/B 按既定实验范围没有纳入内核编译，因此
本文不报告其优化收益。

## 4 访存慢路径开销画像

### 4.1 应用级事件频率与时间占比

表 1 汇总六类应用负载。`data L1 miss / mem` 和 `data refill / mem` 的分母来自独立
plugin-window 运行；named SoftMMU 来自无 profile 计数器的 `perf` 运行。不同口径
只在 workload 和配置一致时关联，不比较 profile 与 timing 运行的墙钟时间。

**表 1  现代 QEMU SoftMMU 慢路径开销。**

| workload | named SoftMMU | data L1 miss / mem | data refill / mem | victim / L1 miss | refill / L1 miss | PTW / mem | levels / PTW |
|---|---:|---:|---:|---:|---:|---:|---:|
| sysbench random 4 KiB write | 5.21% | 0.106% | 0.0587% | 44.46% | 55.54% | — | — |
| stress-ng `tlb-shootdown` | 16.84% | 0.612% | 0.4948% | 19.14% | 80.86% | — | — |
| DaCapo 9.12 `avrora` | 6.61% | 0.221% | 0.0372% | 83.15% | 16.85% | — | — |
| nested QEMU/KVM random | 13.87% | 0.858% | 0.1276% | 85.13% | 14.87% | — | — |
| SPEC2006 `429.mcf` train | 15.22% | 0.291% | 0.2649% | 8.95% | 91.05% | 0.2660% | 4.19 |
| GAPBS PageRank scale-20 ×8 | 1.17% | 0.205% | 0.0246% | 88.00% | 12.00% | 0.0256% | 4.17 |

最直接的发现是：事件频率与时间占比并不成比例。六个负载的 data refill 都不到客户机
访存的 0.5%，但 named SoftMMU 可达到 16.84%。`429.mcf` 中 91.05% 的 data L1
miss 最终进入 refill，PTW 事件约为 data access 的 0.2660%，平均每次 walk 访问
4.193 层；它是当前最明确的应用级 miss-path 压力样本。相反，GAPBS PageRank 中
88.00% 的 L1 miss 被 victim 挽回，PTW 仅为每次访存的 0.0256%。同一个“大工作集”
标签下，具体访问次序和热身行为仍会产生一个数量级的 refill 差异。

GAPBS 还显示了热身口径的重要性。单次冷 trial 曾得到 10.83% named SoftMMU，八次
连续 trial 合并后下降到 1.17%。这并非计数错误，而是 SoftTLB 和页表结构热身后，
初始化开销被更长稳定区间摊薄。正式结果采用八次合并窗口，后续研究也应明确区分冷
启动、稳态和重新映射阶段。

### 4.2 慢路径内部归因

对前四个 pilot workload 的 `perf` 周期进一步分解如表 2。表中函数列为独占周期，
不能相加得到 inclusive 路径时间；`core` 与 `support` 则是分析器定义的互斥分类。

**表 2  可命名 SoftMMU 周期的函数级分解。**

| workload | core | support | `probe_access_internal` | `mmu_translate` + `ptw_translate` | `victim_tlb_hit` | `tlb_set_page_full` |
|---|---:|---:|---:|---:|---:|---:|
| sysbench | 4.69% | 0.52% | 0.95% | 1.02% | 0.49% | 0.46% |
| stress-ng | 14.87% | 1.97% | 2.85% | 2.59% | 0.97% | 1.62% |
| DaCapo | 6.17% | 0.44% | 1.59% | 0.52% | 0.44% | 0.24% |
| nested QEMU | 12.40% | 1.46% | 1.91% | 1.91% | 1.58% | 1.49% |

这些数据表明不存在一个对所有 workload 都占主导的单一慢路径函数。stress-ng 同时
提高 probe、PTW 和安装成本；DaCapo 的 victim recovery 较高，页表遍历独占比例
反而较低；nested QEMU 则在 victim、PTW 和安装上均有明显开销。因此，机制优化应
按事件结构而不是仅按总 SoftMMU 百分比选择。

### 4.3 大页减少 walk 层数，但不减少 4 KiB refill

在 128 MiB、4 KiB 随机页访问微基准中，几乎每次 load 都发生 refill，named
SoftMMU 约占 95%。开启客户机透明 2 MiB 大页后，约 98.4% 的 fill 返回 21-bit
page size，但 QEMU 仍约为每个 4 KiB 子页安装一次 SoftTLB 项。随机访问的早期
对照平均加速约 8.71%，其来源不是 refill 次数下降，而是大页叶节点减少了 walk
层级。

分层 PTW 计数进一步验证了这一解释。在 16 MiB、16,384 次随机页访问的 profile
smoke 中，4 KiB 映射产生 16,502 次 helper-load refill 和 82,873 次页表层访问，
平均 4.992 层/walk；THP 对照产生 16,507 次 refill，数量几乎不变，但其中 14,473
次 fill 返回 2 MiB 页，页表层访问下降到 68,622 次，减少 17.20%，平均降至
4.129 层/walk。该结果给出了大页 translation cache 的直接动机：既然目标 MMU 已经
识别出大页映射，就应尝试在后续 4 KiB 子页 miss 时复用该转换。

### 4.4 嵌套翻译的额外层级

嵌套实验确认内层客户机启用 AMD NPT，并完成 524,288 次随机页访问。测量窗口内，
外层 QEMU 记录 4,248,853 次 primary walk、19,149,367 次层访问，即 4.507
层/walk；同时记录 2,079,998 次 nested walk、8,325,190 次层访问，即 4.002
层/walk。nested walk 占全部 walk 的 32.87%，说明二阶段翻译不是可忽略的小项。
但这些是带插桩机制计数，不用于评价端到端时间。

## 5 保守的 miss-path 优化

### 5.1 设计原则

本文遵循三条原则。第一，原型不改变 `CPUTLBEntry` 的布局，不在每次客户机访存都
执行的生成代码 fast path 中加入比较或分支。第二，命中旁路后仍回到
`tlb_set_page_full()`，避免绕过 RAM/MMIO、ROMD、dirty、watchpoint 和 host
MemoryRegion 处理。第三，初始版本优先采用保守 generation 失效；在正确性覆盖完成
前，不以更精细但难以验证的 range-aware invalidation 换取性能。

两种缓存均支持 `off`、`on` 和 `probe` 三种模式。`probe` 执行查找和匹配但强制
走原路径，用于估计查询自身的成本与潜在命中覆盖率。

### 5.2 大页转换缓存

大页缓存按 MMU mode 分配，共 32 set、4 way，即 128 项。缓存项保存客户机虚拟基址、
物理基址、`CPUTLBEntryFull`、线性转换范围和 generation。插入发生在 target MMU
已经返回转换、但 QEMU 尚未把物理地址解析到具体 MemoryRegion 之前；只有
`lg_translation_size > TARGET_PAGE_BITS` 的转换才可插入。

查询只发生在主表和 victim TLB 均 miss 后。若地址、MMU mode、generation、访问权限
与转换范围匹配，则按大页内偏移合成目标物理地址：

$$
p = p_{base} \mathbin{|} (v \mathbin{\&} (2^{S_{linear}}-1)).
$$

随后用保存的完整转换元数据调用原有 `tlb_set_page_full()`，只安装当前 4 KiB 子页。
写访问不会复用带 `PAGE_WRITE_INV` 的项，权限不足也不命中。full/page/range TLB
invalidation 均推进 generation，使旧项以 $O(1)$ 方式整体失效。

与直接让主表项覆盖大页相比，该设计没有改变公共索引和 tag 比较，也不会让 MMIO
或脏页处理走一条未经验证的新路径；代价是它只减少 target fill/PTW，不减少普通
主表 miss 和 4 KiB 项安装。

### 5.3 非叶 PTW cache

PTW cache 为每个 CPU 分配 64 set、4 way，保存 x86-64 L2--L4 非叶页表转换。key
包括 CR3、paging mode、MMU index、PTW index、level 和该层对应的虚拟地址前缀；
value 保存下一层页表物理地址以及沿途累计权限。命中 L2、L3 或 L4 后，walker 可直接
跳到更低层；叶页表项始终重新读取，以保留最终映射与 accessed/dirty 位语义。

该原型覆盖四级页表、LA57 前缀处理和 NPT 路径，并在所有 QEMU TLB full/page/range
invalidation 上推进统一 generation。与硬件 MMU cache 不同，这里每次查询都执行
软件哈希、比较和统计，因此只有被跳过页表访问的宿主成本高于查询成本时才可能获益。

### 5.4 正确性检查

当前正确性覆盖包括 4 KiB/THP 映射、随机与密集访问、读写权限、`mprotect`、失效、
嵌套翻译和校验和一致性。最终 `mprotect` smoke 完成 32,768 次访问，checksum 为
4,177,920；其间大页缓存命中 29,893 次并经历 27 次失效，PTW cache 经历 16 次
失效。nested 的 base、LP 和 PTW 运行均得到 checksum 66,846,720，64 MiB 映射中
63,488 KiB 经 `/proc/self/smaps` 确认为 THP。

开发过程中曾出现把嵌套失效页大小直接当作线性转换范围的错误；该错误能够通过基本
启动测试，却会为相邻子页合成错误物理地址。本文用独立 `lg_translation_size` 修复，
并保留故障目录供回归分析，但不将其纳入性能表。这也说明，对地址转换缓存而言，
“能启动”或单一 checksum 并不足以替代跨页大小、权限和失效的系统测试。

## 6 性能评估

### 6.1 实验设置

除特别说明外，实验配置见表 3。固定-TLB 消融与最终优化矩阵采用 P-Core、固定频率
请求、提高优先级和休眠抑制；SMT sibling 仍在线，3 次样本仍不足以支撑小幅差异的
发表级统计推断。

**表 3  主要实验环境。**

| 项目 | 配置 |
|---|---|
| 模拟器 | QEMU 8.2.9 (`v8.2.9-dirty`)，x86_64-softmmu |
| 加速器 | `-accel tcg,thread=single`，1 vCPU |
| 宿主机 | Intel Core i7-1260P，P-Core logical CPU 7，nice=-20 |
| 宿主系统 | x86-64 Linux 7.0.0-31-generic；CPU 6/7 `performance`，min=max=2.1 GHz |
| 客户机 | Debian 12，Linux 6.1.0-53-cloud-amd64 |
| 客户机内存 | cloud workload 1 GiB；微基准 512 MiB |
| 磁盘状态 | cloud workload 使用 qcow2 snapshot，丢弃运行期写入 |
| 大页状态 | 优化对比中 guest THP=`always`，并由 `smaps` 验证实际覆盖 |
| 重复次数 | 定向微基准 5 次；应用优化对比 3 次，stress-ng 5 次 |

timing build 不含 `QEMU_TLB_PROFILE`，两个实验缓存的 lookup/hit 原子计数也仅在
profile build 编译。timing、profile 与 perf 分开运行；满载 `turbostat` 复核 CPU 7
的 Bzy_MHz 为 2091 MHz，接近请求的 2.1 GHz。硬件热保护仍可能临时降频。

### 6.2 定向微基准

表 4 使用 128 MiB working set、128 passes、每组 5 次重复。性能取相同页模式的
`base median / candidate median`；括号内给出样本 CV。

**表 4  定向随机访存微基准。**

| guest page | base | LP | PTW | LP+PTW |
|---|---:|---:|---:|---:|
| 2 MiB THP | 306.698（0.48%） | 219.177（0.48%，1.399×） | 311.741（1.22%，0.984×） | 218.849（0.19%，1.401×） |
| 4 KiB | 314.947（0.28%） | 317.663（0.63%，0.991×） | 295.490（0.70%，1.066×） | 295.954（0.67%，1.064×） |

大页 cache 在预期压力场景中取得约 40% 收益；PTW cache 的净收益为 6.6%。独立
profile 显示，THP 下 LP 将约 419.5 万次 refill 降到约 6.6 万，4 KiB 下 PTW L2
约命中 419.5 万次。LP+PTW 没有超过适配页模式的单一机制，说明额外查询不叠加收益。

### 6.3 应用与系统负载

表 5 报告应用级中位数。主指标按 workload 定义：sysbench 用 MiB/s，DaCapo 用
guest 毫秒，nested 用内层 ns/access，`mcf`/GAPBS 用测量窗墙钟，stress-ng 用
real-time bogo ops/s；大于 1 表示更快。括号中依次为 timing 样本 CV 和独立 profile
的机制命中率，而不是主表命中率。

**表 5  应用级端到端性能与机制命中率。**

| workload | n | LP ratio（CV；命中） | PTW ratio（CV；命中） | LP+PTW ratio（CV；LP/PTW 命中） |
|---|---:|---:|---:|---:|
| sysbench | 3 | 1.000×（0.07%；2.35%） | 0.995×（0.66%；67.78%） | 0.998×（0.68%；32.58%/73.94%） |
| stress-ng | 5 | 1.026×（1.63%；17.96%） | 1.060×（3.46%；79.91%） | 1.089×（2.64%；19.22%/90.73%） |
| SPEC `429.mcf` | 3 | 1.060×（7.76%；68.55%） | 0.995×（2.03%；56.51%） | 1.069×（7.10%；67.96%/99.81%） |
| GAPBS PR | 3 | 0.997×（3.74%；81.35%） | 1.010×（5.62%；59.04%） | 1.006×（0.79%；78.65%/95.42%） |
| DaCapo | 3 | 1.022×（5.79%；33.00%） | 0.995×（1.03%；65.97%） | 1.068×（4.66%；26.09%/98.93%） |
| nested hotspot | 3 | 2.229×（7.97%；49.87%） | 1.106×（6.72%；52.49%） | 2.198×（3.64%；47.62%/58.15%） |

当前结果不支持“两个原型已经带来通用端到端加速”的结论。sysbench 和 GAPBS 基本
不变；DaCapo both 与 `mcf` LP/both 有约 6%--7% 正信号，但其 CV 也达到约
4.7%--7.8%。DaCapo 与 `mcf` 的 PTW 命中率虽为 65.97% 和 56.51%，性能仍约
-0.5%，直接说明软件 cache 命中率不能单独预测净收益。

stress baseline 的 real-time bogo/s 呈双峰，CV 为 38.2%，低簇只使用约一半 CPU
时间。按 CPU-time 中位速率，base、LP、PTW、both 分别为 327.88、336.57、347.80、
357.18 bogo/s；所有样本均保留，但本负载只作为提示，不作为稳定加速证据。六类
workload 的四种配置均来自同一无计数 QEMU binary，profile 只用于独立机制解释。

### 6.4 热点收益为何没有转化为端到端收益

嵌套负载给出了最直观的 Amdahl 定律例子。内层 random-page 热点中，大页 cache 将
中位延迟从 568.796 ns/access 降到 LP 的 255.158 ns/access 和 both 的
258.788 ns/access，达到 2.229 倍和 2.198 倍；但外层测量窗只从 17.206 s 降到
16.311 s 和 16.493 s，即 1.055 倍和 1.043 倍。机制命中且局部窗口显著变快，
并不意味着它在端到端时间中占有足够权重。

设原始执行时间中可优化部分占比为 $f$，该部分加速为 $s$，则理论端到端加速上限为

$$
Speedup_{total}=\frac{1}{(1-f)+f/s}.
$$

当 refill 只占客户机访存的千分之几，且 workload 还包含大量非访存、fast-hit、
设备和启动成本时，即使 $s$ 很高，$f$ 也可能过小。更高的 LP/PTW 命中率也不必然
带来收益：命中率的分母是已经通过主表和 victim 过滤的少数事件，缓存查询、哈希和
普通 4 KiB 项安装仍然存在。

### 6.5 固定主表并关闭 victim 后的时间归因

为检查动态扩容和 victim 是否掩盖了早期论文中的访存开销，本文新增固定容量消融：
每个 MMU mode 的主 SoftTLB 固定为 4096 项，关闭 8-entry victim TLB，并在
i7-1260P 的 P-Core logical CPU 7 上以 nice -20 运行；CPU 6/7 的 scaling
min/max 均请求为 2.1 GHz。perf 使用 P-Core 的 `cpu_core/cycles/u` 事件，LP/PTW
实验 cache 均关闭。

六个系统负载的 named SoftMMU 占比分别为 sysbench 0.47%、DaCapo 8.70%、
GAPBS 11.49%、nested QEMU 14.62%、stress-ng shootdown 20.85% 和 `429.mcf`
24.18%；等权平均 13.39%，中位数 13.05%。因此，在这个配置下可命名慢路径没有
接近 TACO 报告的 38.1%，最高的 mcf 也未达到 30%。相反，刻意制造持续 miss 的
128 MiB 随机页微基准达到约 95%，证明 workload 选择可以把相同实现从低占比推到
几乎完全受慢路径支配。

该消融仍不能复刻 TACO 的完整口径：JIT 中内联的 fast-hit lookup 被归入 guest
JIT，无法与客户机有效计算分离。故 13.39% 是当前可归因慢路径的下界，而不是全部
访存模拟开销。每个系统负载当前只有一次 perf 运行，虽有 6,834--100,696 条有效
样本并全部通过 provenance/配置校验，发表版仍需重复采样。完整数据和复现命令见
`FIXED_TLB_NOVICTIM_RESULTS_ZH.md`。

### 6.6 对研究问题的回答

- **RQ1：** 现代 SoftTLB 的 refill 确实稀少，但剩余事件代价高；“事件低频”不能
  推导出“慢路径时间可忽略”。
- **RQ2：** 两个原型都能减少目标工作量；大页 cache 和 PTW cache 在适配的定向
  微基准中分别获得 39.9% 和 6.6% 加速，保持 fast path 不变适合机制验证。
- **RQ3：** 当前证据不支持通用应用级加速，但 `mcf` LP/both 与 DaCapo both 出现
  约 6%--7% 的待确认信号。收益受 workload、热身、失效频率和端到端覆盖率影响。
- **RQ4：** 真实负载结果弱并非单一原因。现有数据同时显示机制有效、覆盖率不足和
  测量噪声较大；下一阶段应扩大代表性 workload 并提高统计强度，而不是仅继续调大
  缓存容量。

## 7 后续 workload 与实验计划

### 7.1 选择原则

后续 workload 不应只因为“常用于 TLB 论文”就加入，而应预先回答它覆盖哪一类
地址转换行为。建议至少记录并按以下维度分层：

1. working-set 大小、活跃 4 KiB/2 MiB/1 GiB 页数和实际 THP 覆盖率；
2. 主表 miss、victim recovery、refill 和 PTW / guest memory operation；
3. 冷启动与稳态、顺序/随机/图遍历局部性；
4. `mmap`、`munmap`、`mprotect`、上下文切换和 shootdown 造成的 invalidation/churn；
5. primary 与 nested PTW 比例，以及多 vCPU 下的并发与锁竞争；
6. named SoftMMU、guest JIT、TB lookup/chaining、设备/I/O 和其他时间分类。

只有同时满足“慢路径可观测”和“应用场景有代表性”的 workload，才适合检验本文方案；
仅有高硬件 dTLB MPKI 不保证在 QEMU 的动态 SoftTLB 上也有高 refill 率。

### 7.2 候选 workload 矩阵

本机 `hpca2027` 目录没有 #64，实际与 TLB 相关的是 14 页的 HPCA 2027 #54
*Bifrost: Improving Translation Reach across Cache Hierarchy with In-Place Table
Entry Coalescing*。其表 III 使用以下 workload，并筛选 L2 TLB MPKI>5 的任务，
每项模拟 500M 指令：

- GraphBIG：PR、GC、SSSP、TC、BFS、CC、BC，数据集 8 GB；
- XSBench particle simulation，9 GB；
- GUPS random access，10 GB；
- DLRM sparse-length sum，10.3 GB；
- GenomicsBench k-mer counting，33 GB。

这些是硬件地址转换论文的高 TLB-miss 候选，不保证在 QEMU SoftTLB 下同样高 miss；
下一轮应先用缩小输入完成正确性，再扩到论文规模并筛选 SoftTLB refill/PTW 强度。

| 类别 | 候选 | 预期覆盖 | 当前状态 |
|---|---|---|---|
| SPEC 内存密集型 | CPU2006 `429.mcf`、`471.omnetpp`、`483.xalancbmk`；视许可证扩展 CPU2017 对应负载 | 大工作集、指针追踪、冲突 miss | `429.mcf` train 已测，其余待测 |
| 图分析 | GraphBIG PR/GC/SSSP/TC/BFS/CC/BC（#54 为 8 GB） | 随机访问、图规模与热身敏感性 | GAPBS PR scale-20 已测 |
| KV/存储 | Redis、Memcached、RocksDB/YCSB | 长稳态、大页、系统调用与 I/O 混合 | 待准备 |
| 数据分析/HPC | XSBench 9 GB、GUPS 10 GB、DLRM 10.3 GB、GenomicsBench 33 GB | 大地址空间、不规则访存和稀疏访问 | 已从 #54 核验，待准备 |
| 托管运行时 | 更长 DaCapo workload 与多次迭代 | code/data 混合、GC 与稳定态 | `avrora` 短样本已测 |
| 虚拟化 | 更长 nested QEMU，4 KiB/THP、不同 NPT 组合 | 二阶段翻译与端到端稀释 | random-page 已测 |
| OS 压力 | kernel build、fork/exec、mmap/mprotect、shootdown | 失效、上下文切换和长时间系统行为 | baseline kernel build 已测 |

### 7.3 发表级实验协议

下一阶段建议采用以下协议：

- 将宿主 governor 设为 `performance`，固定频率或记录频率，隔离物理核并避开 SMT
  sibling，关闭会改变长运行墙钟的自动休眠；
- 每个 workload 预热后测量 30--60 s 的稳定窗口，或运行到确定的工作量完成；
- baseline、LP、PTW、probe 使用随机化 ABBA/Latin-square 顺序，每组至少 10 次；
- 报告中位数、均值、标准差、bootstrap 95% 置信区间和效应量，并预先定义离群规则；
- 使用同一最终提交、同一 QEMU binary hash、同一 snapshot backing image 和相同输入；
- timing、profile、plugin-window 继续分开；timing 保持不编译 cache 原子统计；
- 除端到端时间外，同时报告 LP/PTW 查询与命中、实际跳过的 fill/level visit、flush、
  eviction、主表最终容量和 THP 覆盖率；
- 先用小输入做结果正确性和数据集校验，再准备大数据；数据生成、解压和 cache warming
  不计入 workload 测量窗口，但必须在复现说明中记录。

### 7.4 从 miss path 扩展到公共 fast path

若更有代表性的 workload 仍不能让端到端收益稳定超过噪声，研究重点应转向覆盖每次
访存的公共路径。候选方向包括更紧凑的 tag/addend 布局、减少 compare 与分支、低冲突
索引、受控二路结构，以及减少宿主 I-cache/DTLB 压力。但任何 fast-path 方案都必须
同时报告 TCG 生成代码大小、宿主前端事件和无 miss 的 dense workload，避免“降低
miss 但让每次 hit 更贵”。

miss-path 原型本身也可继续自适应化：根据主表/victim miss、LP/PTW 命中、flush
churn 和实际节省的层访问动态启停；对 PTW cache 比较只查 L3、不同 set/way 与替换
策略；对大页 cache 在 target 明确提供线性 translation span 时才启用，并在正确性
充分后探索 range-aware invalidation。

## 8 局限性与有效性威胁

**测量环境。** 当前实验只有一台 i7-1260P 宿主。最终固定-TLB A/B 绑定 P-Core、
nice=-20、抑制休眠，并把 scaling min/max 请求为 2.1 GHz；满载 Bzy_MHz 为
2091 MHz。但同一物理核的 SMT sibling 仍在线，硬件仍可因温度或功耗保护降频。
应用优化对比仅 3 次（stress 为 5 次），固定 TLB 系统负载各 1 次 perf；短 workload
不能区分小信号与系统噪声。现有结果应视为筛选线索而非显著性结论。

**实现范围。** 原型目前聚焦 x86-64、单 vCPU、single-thread TCG。大页 cache 的
容器位于通用 SoftMMU 层，但可线性范围由 x86 target 显式提供；其他 target 尚未验证。
多 vCPU/MTTCG 下的缓存共享、失效广播、锁竞争和内存序也未形成完整性能数据。

**时间归因。** named SoftMMU 不包含内联在客户机 JIT 代码中的 fast-hit 指令，因而
低估全部访存模拟成本；反过来，把所有 guest JIT 周期都归因于访存又会严重高估。
本文只把可命名慢路径用于机制定位，不声称复刻早期论文的 38.1% 总访存比例。

**插桩效应。** profile 和 plugin 都会扰动执行，因此与 timing 分开。最终 timing
build 已不编译 cache lookup/hit 原子计数；命中率只取自独立 profile。profile 的
时间不参与加速比，且单次 profile 的 ASLR/直接映射冲突状态仍可能改变命中率。

**workload 代表性。** 目前只有 `429.mcf` 的一个 SPEC train 输入、GAPBS 的一个
算法/图规模和 DaCapo 的一个 benchmark。已有结果足以证明 workload 依赖性，却不足
以代表服务器、图分析、托管运行时或全部 SPEC。HPCA 2027 #54 的 workload 已从
原 PDF 表 III 核验，但当前尚未运行其 8--33 GB 数据集，不能把第 7 节表述成复现。

**原型 lineage。** 最终表中的六类 A/B 数据来自同一无统计 QEMU binary，旧原型
数据不再进入主表。profile 计数来自同一源码的独立插桩构建，不能与 timing 比绝对
时间；发表版仍应增加重复并冻结最终提交与二进制哈希。

## 9 相关工作

Tong 等系统分析了早期 QEMU 的访存模拟开销，并比较 SoftTLB 容量调整、victim、
关联结构、大页与 refill 优化，报告平均 38.1% 的访存模拟时间和 24.4% 的组合优化
收益 [1]。本文复查的是十年后已具备动态表和 victim TLB 的 QEMU 8.2，结论从
“扩大和增强主 SoftTLB”转向“现代实现中剩余 miss 的覆盖率是否足以抵消软件查询
成本”。

Hong 等进一步研究 full-system DBT 的控制转移与内存虚拟化，在 QEMU 2.2.0 上强调
动态 SoftTLB 容量、large-page partial flush 与 per-process 行为的共同影响 [2]。
其结果说明扩大容量必须同时考虑 flush；本文实际观测到的 1,024--16,384 项运行期
容量和 workload 间 victim recovery 差异支持这一观点，但本文没有复现其跨 ISA 与
Android 实验。

Barr、Cox 和 Rixner 系统比较硬件 MMU cache，指出保存部分转换并跳过页表层级通常
优于只缓存页表项 [3]。本文的 L2--L4 PTW cache 借鉴该机制，但把研究问题放在软件
页表遍历：软件哈希、权限累计和失效成本使其收益边界不同于硬件。

Bellard 介绍了 QEMU 基于动态翻译的可移植模拟器设计 [4]。DaCapo、SPEC CPU 和
GAPBS 分别提供托管运行时、传统 CPU 应用与图分析 workload [5--7]。本文当前只使用
这些套件的子集，不把子集结果外推为整套 benchmark 的平均性能。

## 10 结论

本文对 QEMU 8.2.9 x86-64 system mode 的访存慢路径进行了分层测量。现代动态主
SoftTLB 和 victim TLB 已把 data refill 压低到客户机访存的 0.0246%--0.4948%，但
可命名慢路径仍可占 1.17%--16.84% 的宿主周期，呈现明显的低频高代价特征。基于该
特征实现的大页转换缓存和非叶 PTW cache 不改变生成代码 fast path。固定 4096 项、
关闭 victim 后，可命名慢路径平均为 13.39%、最高为 24.18%；两种 cache 分别在
适配的定向微基准中达到 1.399 倍和 1.066 倍加速。

然而，sysbench 和 GAPBS 仍基本无变化；`429.mcf` LP/both 与 DaCapo both 只有约
6%--7% 的未确认正信号，nested 约 2.2 倍热点收益也仅转化为约 1.05 倍外层收益。
这一结果缩小了问题空间：单纯让 refill 更快不足以构成通用优化，后续必须采用 #54
一类高 TLB-miss 大数据负载、延长稳定窗口并增加重复；若覆盖率仍低，则应转向每次
访存都执行的公共 fast path，同时严格约束代码大小和宿主 cache 代价。

## 参考文献

[1] X. Tong, T. Koju, M. Kawahito, and A. Moshovos. Optimizing Memory
Translation Emulation in Full System Emulators. *ACM Transactions on
Architecture and Code Optimization*, 11(4), Article 60, 2015.
<https://doi.org/10.1145/2686034>

[2] D.-Y. Hong, C.-C. Hsu, C.-Y. Chou, W.-C. Hsu, P. Liu, and J.-J. Wu.
Optimizing Control Transfer and Memory Virtualization in Full System
Emulators. *ACM Transactions on Architecture and Code Optimization*, 12(4),
2015. <https://doi.org/10.1145/2837027>

[3] T. W. Barr, A. L. Cox, and S. Rixner. Translation Caching: Skip, Don't
Walk (the Page Table). In *Proceedings of ISCA*, 2010.
<https://www.cs.rice.edu/CS/Architecture/docs/barr-isca10.pdf>

[4] F. Bellard. QEMU, a Fast and Portable Dynamic Translator. In *Proceedings
of the USENIX Annual Technical Conference*, 2005.

[5] S. M. Blackburn et al. The DaCapo Benchmarks: Java Benchmarking
Development and Analysis. In *Proceedings of OOPSLA*, 2006.

[6] J. L. Henning. SPEC CPU2006 Benchmark Descriptions. *ACM SIGARCH Computer
Architecture News*, 34(4), 2006.

[7] S. Beamer, K. Asanović, and D. Patterson. The GAP Benchmark Suite. In
*Proceedings of IISWC*, 2015.

## 附录 A：当前证据与复现入口

本文数据解释与详细 caveat 见：

- [`REPORT_ZH.md`](REPORT_ZH.md)：开销画像、三口径定义与长 Linux build；
- [`OPTIMIZATION_RESULTS_ZH.md`](OPTIMIZATION_RESULTS_ZH.md)：两个原型及 A/B 结果；
- [`PILOT_RESULTS.md`](PILOT_RESULTS.md)：早期运行、失败与数据集来源；
- [`canonical-results.json`](canonical-results.json)：六类开销画像的固定结果清单；
- [`README.md`](README.md)：构建、运行、校验与 provenance 说明。

主要复现命令如下：

```sh
./tlb-study/run-tests.sh
./tlb-study/summarize-workloads.py \
  --manifest tlb-study/canonical-results.json
python3 ./tlb-study/summarize-optimizations.py \
  tlb-study/results/opt-*-fixed4k-novictim-pcore7-fixed2100-\
nostats-v1-{base,lp,ptw,both}-r0[1-3]
./tlb-study/validate-results.py RESULTS... \
  --qemu-version 8.2.9 --require-provenance
```

## 附录 B：投稿前待补清单

- [ ] 补作者、单位、基金、匿名化与投稿格式；
- [x] 核验本机实际的 HPCA 2027 #54 PDF、页数、workload、输入和筛选指标；
- [x] 将 #54 workload 与第 7 节候选矩阵对齐；
- [x] 在统一无 cache 原子统计 binary 上重跑 baseline/LP/PTW/LP+PTW；
- [ ] 在隔离核、performance governor 和禁用休眠条件下至少重复 10 次；
- [ ] 完成已接入的 `471.omnetpp`、`483.xalancbmk` 四配置测量，并增加更多
  GAPBS 算法及大规模输入；
- [ ] GraphBIG v3.2 的 PR/GC/SSSP/TC/BFS/CC/BC runner 与小图正确性门槛已
  准备；正式结果仍需确定并记录可复现的大图来源，不能用相同字节数替代 #54
  未披露的 8 GB 图；
- [ ] 增加至少一类长稳态 server/KV workload 和一类多 vCPU workload；
- [ ] 对 Linux kernel build 补齐 baseline/LP/PTW 对比，或解释为何排除；
- [ ] 补齐 read/write/execute、unmap/remap、MMIO、dirty/watchpoint 和多核 shootdown
  的自动化正确性回归；
- [ ] 报告 bootstrap 区间、效应量、随机化顺序和预注册离群规则；
- [ ] 生成正式图：访存路径、事件漏斗、微基准结果、应用结果和命中率—收益散点图；
- [ ] 复核参考文献的卷期、页码、DOI，并按目标会议模板排版。
