# QEMU 8.2 SoftMMU TLB 实验阶段报告

## 研究问题与口径

本实验在 QEMU 8.2.9 x86-64 system mode、单 vCPU TCG 上复查 TACO 论文的
核心问题：动态 SoftTLB 容量显著增大以后，TLB miss 和 refill 是否已低到不值得
优化。时间数据来自不含计数器的 baseline QEMU；miss/refill 来自独立的
`QEMU_TLB_PROFILE` 构建；精确分母由 TCG plugin 在 workload barrier 内统计。
三类数据不混用运行时间。
两个 build tree 的 `compile_commands.json` 已交叉检查：
`QEMU_TLB_PROFILE` 仅存在于 profile 构建，在 baseline 构建中不存在。

原 TACO 工作在较早 QEMU 上报告平均 38.1% 时间用于内存模拟，组合 STLB
优化带来平均 24.4% 模拟器加速。本实验不直接套用该数值，而是检验 QEMU 8.2
已经具备动态主表和 victim TLB 后，慢路径是否仍构成可测瓶颈。

这里的 `L1 miss` 是 QEMU 直接索引主表 miss，`victim hit` 是随后被 8 项 victim
TLB 挽回的部分，`refill` 是仍需调用 target MMU fill 的部分。“SoftMMU 时间”仅为
`perf` 能识别的慢路径、页表遍历及维护函数，不包含内联在 guest JIT 代码中的
fast-hit 指令，因此是可归因慢路径占比，不是论文中全部内存模拟开销的严格复刻。

## 已测结果

| workload | named SoftMMU 时间 | data L1 miss / guest mem op | data refill / guest mem op | victim / L1 miss | refill / L1 miss |
|---|---:|---:|---:|---:|---:|
| sysbench 随机 4 KiB 写 | 5.21% | 0.106% | 0.0587% | 44.46% | 55.54% |
| stress-ng `tlb-shootdown` | 16.84% | 0.612% | 0.4948% | 19.14% | 80.86% |
| DaCapo 9.12 `avrora` | 6.61% | 0.221% | 0.0372% | 83.15% | 16.85% |
| nested QEMU/KVM 随机页访问 | 13.87% | 0.858% | 0.1276% | 85.13% | 14.87% |
| SPEC2006 `429.mcf` train | 15.22% | 0.291% | 0.2649% | 8.95% | 91.05% |
| GAPBS PageRank scale-20 x8 | 1.17% | 0.205% | 0.0246% | 88.00% | 12.00% |

前四项旧 pilot 的周期权重进一步分解如下。`core` 包含 refill、PTW、victim、原子访问和 TLB
维护函数，`support` 包含这些路径调用的通用支持函数。后几列是 `perf` 对函数
本身的独占归因，不是 inclusive 调用链时间，也不能相加得到完整 SoftMMU 时间。

| workload | core | support | `probe_access_internal` | `mmu_translate` + `ptw_translate` | `victim_tlb_hit` | `tlb_set_page_full` |
|---|---:|---:|---:|---:|---:|---:|
| sysbench | 4.69% | 0.52% | 0.95% | 1.02% | 0.49% | 0.46% |
| stress-ng | 14.87% | 1.97% | 2.85% | 2.59% | 0.97% | 1.62% |
| DaCapo | 6.17% | 0.44% | 1.59% | 0.52% | 0.44% | 0.24% |
| nested QEMU | 12.40% | 1.46% | 1.91% | 1.91% | 1.58% | 1.49% |

前四个精确窗口分别包含约 13.1 亿、5.86 亿、113.4 亿和 26.9 亿次 guest
memory operation；新增 `429.mcf` 与 GAPBS 窗口分别为 80.43 亿和 63.20 亿次。
两组新增 baseline perf 数据分别包含 4,047 和 3,261 条带 cycle-period 权重的
有效记录，并通过完整 provenance 校验。

`429.mcf` 是当前最明确的应用级 miss-path 压力样本：91.05% 的 data L1 miss
最终进入 refill，PTW/event 达 guest data access 的 0.2660%，平均 4.193 层/walk。
GAPBS PageRank 的八次 trial 则由 victim TLB 挽回 88.00% 的 data L1 miss，PTW
仅为 0.0256%/access。其单次冷 trial 曾测得 10.83% named SoftMMU，而八次合并后
为 1.17%，显示页表与 SoftTLB 热身会显著改变可见瓶颈；正式表采用覆盖更充分的
八次样本。

微基准进一步给出了机制边界。128 MiB、4 KiB 随机页访问几乎每次 load 都发生
refill，named SoftMMU 占比约 95%；切换为透明 2 MiB 大页后，约 98.4% 的 fill
返回 21-bit page size，但 QEMU 仍约每个 4 KiB 子页安装一次 SoftTLB 项。随机
访问平均加速约 8.71%，主要可能来自每次 refill 的页表遍历层数减少，而不是
refill 次数减少。密集 64 B stride 的结果噪声较大，尚不能证明大页收益。

新增 PTW 分层计数给出了直接机制证据。在一组 16 MiB、16,384 次随机页访问的
profile smoke 中，4 KiB 映射产生 16,502 次 helper-load refill 和 82,873 次
页表层访问，平均每次 walk 为 4.992 层。THP 对照产生 16,507 次 refill，几乎
不变；但 14,473 次 fill 返回 21-bit page size，层访问降至 68,622 次，减少
17.20%，平均为 4.129 层。这说明大页收益来自跳过叶级访问，而 QEMU 仍按 4 KiB
子页 refill。该数据来自带计数器的小规模机制实验，其时间不能替代 baseline
性能结果。

嵌套 QEMU/KVM 对照确认客体 `kvm_amd` 的 NPT 已开启，内层 VM 完成 524,288
次随机页访问。修正 `nested_pg_mode` 不含 `PG_MODE_PG` 所导致的统计门控缺口后，
外层 QEMU 在测量区间记录到 4,248,853 次 primary walk、19,149,367 次层访问
（4.507 层/walk），以及 2,079,998 次 nested walk、8,325,190 次层访问（4.002
层/walk）。nested walk 占全部 walk 的 32.87%，说明二阶段翻译在该虚拟化场景中
不是可忽略的小项；该结论仍是插桩计数证据，不用于比较运行时间。

## 对“大容量会消除 miss”的判断

这个担心只成立了一部分。真实 workload 的 refill 频率确实低，六组数据均低于
每次 guest memory operation 的 0.5%；但剩余慢路径占 1.17%--16.84% 的采样
周期，且 `429.mcf` 达 15.22%，说明“事件少”等价于“时间不重要”的推论不成立。

QEMU 的主表虽然编译期上限为 2^22 项，六个应用 workload 结束时每个 MMU mode
最大的表实际为 1,024--16,384 项。表仍为直接索引，且仅在 flush 时按窗口
占用率调整大小；扩大容量还会增加 flush 和 host-cache 代价。DaCapo 与嵌套
QEMU 中 victim recovery 超过 83%，也直接表明冲突淘汰仍频繁存在。因此优化
机会没有消失，但收益会高度依赖 workload 和 invalidation 行为。

## 当前优先级

1. 在主表和 victim 均 miss 后查询小型大页 translation cache；命中后仍安装
   普通 4 KiB 项，先避免修改生成代码的 fast-hit 路径。
2. 用新增的 x86 PTW 分级计数测量 primary/NPT walk、各层访问次数和完整重启，
   再决定是否缓存非叶节点；缓存键至少包含 CR3/PCID、paging mode、level 和
   虚拟地址前缀，失效必须覆盖 CR3、INVLPG、全局页、权限变化及嵌套翻译。
3. 针对直接索引冲突测试第二哈希或更强 victim 结构，并用 dense workload 检查
   公共命中路径是否因额外指令而倒退。
4. 若这些方案不能改善 kernel build、DaCapo、SPEC、GAPBS 和 nested QEMU，
   再把范围扩展到 TB lookup/chaining、生成代码质量或 MTTCG 扩展性。

## 证据边界与后续复现

- Linux 3.12.9 clean `make -j1` 已在无插桩 baseline QEMU 中完整成功：wall
  14,674.95 秒、QEMU CPU 14,646.13 秒、1,451,310 条有效采样记录，named
  SoftMMU 为 7.89%（core 7.13%、support 0.75%），guest JIT 为 46.12%。运行
  从全新解压树开始，完成 `vmlinux` 和 `bzImage`，远端返回 0，并通过
  provenance、QEMU 版本和 Linux-build 专项校验。这补齐了 TACO 风格 kernel
  build 时间口径；作为无插桩时间运行，它有意不同时提供 miss 分母。
- PTW 分级计数已完成双构建树重编译，并通过真实 4 KiB/THP、stress-ng、
  `429.mcf`、GAPBS 和 nested QEMU/NPT 实验；这部分机制覆盖已完成。
- canonical 表的前四项生成于新版 provenance 记录之前，虽然事件、类型和 QEMU
  版本检查均通过，但缺少完整哈希和结构化环境记录；新增 `429.mcf` 与 GAPBS
  三口径结果以及 Linux clean perf 已通过新版 provenance 校验。最终发表数据
  仍应重跑前四项，并对关键无插桩时间实验做多次交错 A/B 重复。
- SPEC CPU2006 `429.mcf` train 与 GAPBS PageRank scale-20 x8 已完成
  perf/profile/window 三组运行并通过校验；`471.omnetpp`、`483.xalancbmk` 可作为
  后续扩展，但不再是当前两类压力场景结论的必要前置条件。

因此当前结论是“存在可测且可能有价值的优化空间”，而不是已经证明某个具体
方案具备端到端加速。最终评价必须采用独立的无插桩 A/B 时间运行和重复顺序实验。

## 相关工作

- Xin Tong 等，[*Optimizing Memory Translation Emulation in Full System
  Emulators*](https://doi.org/10.1145/2686034)，TACO 11(4)，2015：本实验的
  原始对照，覆盖 STLB lookup 与 refill 优化。
- Ding-Yong Hong 等，[*Optimizing Control Transfer and Memory Virtualization
  in Full System Emulators*](https://doi.org/10.1145/2837027)，TACO 12(4)，
  2015：强调动态 SoftTLB 需要同时权衡命中率和 flush 成本。
- Thomas W. Barr、Alan L. Cox、Scott Rixner，[*Translation Caching: Skip,
  Don't Walk (the Page Table)*](https://www.cs.rice.edu/CS/Architecture/docs/barr-isca10.pdf)，
  ISCA 2010：硬件 partial-translation cache 是本实验 PTW 非叶缓存的机制参考，
  但不直接证明软件实现一定获益。
