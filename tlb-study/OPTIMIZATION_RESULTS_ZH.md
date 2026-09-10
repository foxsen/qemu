# QEMU 8.2 SoftMMU Miss 路径优化结果

## 结论摘要

在 QEMU 8.2.9 x86-64 system mode、单 vCPU、single-thread TCG 上，大页
miss-cache 和非叶页表缓存都能显著加速刻意制造 miss 的微基准，但尚未证明它们是
通用的整机加速方案。在固定 4096 项主表、关闭 victim、P-Core 锁定 2.1 GHz 且
timing build 去除 cache 原子计数后，大页缓存在 THP 随机微基准达到 **1.399x**，
PTW cache 在 4 KiB 随机微基准达到 **1.066x**。真实负载中，sysbench 和 GAPBS
仍基本无变化；`429.mcf` 的 LP 与 LP+PTW 分别出现 1.060x 和 1.069x 趋势，nested
热点窗口达到 2.229x 和 2.198x，但端到端只有约 1.05x。

因此，“动态 SoftTLB 和 victim TLB 会过滤掉大部分 miss，使 miss 路径优化难以在
应用级体现”这个担心是成立的。但机会并未消失：剩余 miss 很贵，且在特定随机访存、
二阶段翻译和频繁页表遍历场景中仍有明确的机制收益。更合理的研究目标是识别并自适应
覆盖这些场景，而不是永久增加所有访存的 fast path 成本。

## 实现

### 大页 translation cache

- 每个 MMU mode 使用 32 set、4 way，共 128 项；只在动态主表和 8-entry victim
  TLB 都 miss 后查询，不改变生成代码中的 fast-hit 路径。
- 从一次大页 fill 保存线性 `vaddr -> paddr` 区间；命中后重建目标 4 KiB 子页并安装
  普通 SoftTLB 项。`off`、`on`、`probe` 由 `QEMU_SOFTMMU_LP_CACHE` 控制。
- full/page/range invalidation 使用 generation 作 O(1) 失效；store 不复用
  `PAGE_WRITE_INV` 项，权限不足时不命中。
- 新增 `lg_translation_size`，将“可线性重用范围”与原有 `lg_page_size` 分开。x86
  nested translation 为保证失效正确会取两阶段页大小的最大值，但线性映射只能取
  两阶段范围的最小值。混淆二者会为相邻子页合成错误物理地址，这是本轮发现并修复的
  关键正确性边界。

### L2--L4 页表缓存

- 每 CPU 使用 64 set、4 way，缓存 x86-64 L2--L4 非叶 walk 结果，叶节点始终读取。
- key 包含 CR3、paging mode、MMU/PTW index、level 和虚拟地址前缀，并保存累计
  权限；支持 4-level、LA57 和 NPT 路径。
- 所有 QEMU TLB full/page/range invalidation 都推进 generation。环境变量
  `QEMU_X86_PTW_CACHE=off|on|probe` 控制行为。

## 实验口径与正确性

Linux 内核编译按本轮要求排除。其余测试使用 QEMU 8.2.9、`-accel
tcg,thread=single`、1 vCPU，主 SoftTLB 固定 4096 项并关闭 victim。QEMU 绑定到
i7-1260P 的 P-Core logical CPU 7、nice=-20；SMT sibling CPU 6/7 的
`intel_pstate` min/max 均设为 2.1 GHz，`turbostat` 满载实测 Bzy_MHz 为 2091 MHz。
cloud workload 使用 snapshot，guest THP 设为 `always`，并用 systemd inhibitor
阻止休眠。timing build 不含 `QEMU_TLB_PROFILE`，cache lookup/hit 原子计数也只在
profile build 编译。系统 timing 随机化执行顺序并重复 3 次；stress-ng 因双峰补到
5 次。SMT sibling 未离线，当前结果仍是趋势而非发表级显著性结论。

最终 `mprotect` smoke 完成 32,768 次访问，checksum 为 4,177,920；期间大页缓存
命中 29,893 次并经历 27 次失效，PTW cache 经历 16 次失效，结果通过 PTW/TLB
一致性校验。nested 的全部 base/LP/PTW 运行都得到相同 checksum 66,846,720，且
64 MiB 映射中 63,488 KiB 确认为 THP。故障注入式的旧 nested 目录被保留用于说明
逐项清空和页大小语义错误，不纳入性能表。

## 固定 TLB、无统计开销微基准

每组为 128 MiB、128 passes、5 次重复；表内为中位数，括号内为样本 CV。加速比
统一取同页模式 base 除以候选值。

| guest page | base | LP | PTW | LP+PTW |
|---|---:|---:|---:|---:|
| 2 MiB THP | 306.698（0.48%） | 219.177（0.48%，1.399x） | 311.741（1.22%，0.984x） | 218.849（0.19%，1.401x） |
| 4 KiB | 314.947（0.28%） | 317.663（0.63%，0.991x） | 295.490（0.70%，1.066x） | 295.954（0.67%，1.064x） |

独立 profile 运行显示：THP 下 LP 命中约 412.9 万次，将约 419.5 万次 refill 降到
约 6.6 万；4 KiB 下 PTW L2 约命中 419.5 万次。LP+PTW 没有超过适配该页模式的
单一机制：THP 相差 +0.15%，4 KiB 相差 -0.16%，说明额外查询没有形成叠加收益。

## 非内核真实负载

主指标按 workload 定义：sysbench 用 MiB/s，DaCapo 用 guest 自报毫秒，nested 用
内层 ns/access，`mcf`/GAPBS 用测量窗墙钟，stress-ng 用 real-time bogo ops/s。
括号内为 timing 样本 CV；命中率来自独立的一次 profile 运行，不与 timing 合并。

| workload | n | LP ratio（CV；命中） | PTW ratio（CV；命中） | LP+PTW ratio（CV；LP/PTW 命中） |
|---|---:|---:|---:|---:|
| sysbench random memory | 3 | 1.000x（0.07%；2.35%） | 0.995x（0.66%；67.78%） | 0.998x（0.68%；32.58%/73.94%） |
| stress-ng `tlb-shootdown` | 5 | 1.026x（1.63%；17.96%） | 1.060x（3.46%；79.91%） | 1.089x（2.64%；19.22%/90.73%） |
| SPEC CPU2006 `429.mcf` train | 3 | 1.060x（7.76%；68.55%） | 0.995x（2.03%；56.51%） | 1.069x（7.10%；67.96%/99.81%） |
| GAPBS PageRank scale-20 | 3 | 0.997x（3.74%；81.35%） | 1.010x（5.62%；59.04%） | 1.006x（0.79%；78.65%/95.42%） |
| DaCapo `avrora` | 3 | 1.022x（5.79%；33.00%） | 0.995x（1.03%；65.97%） | 1.068x（4.66%；26.09%/98.93%） |
| nested QEMU/KVM hotspot | 3 | 2.229x（7.97%；49.87%） | 1.106x（6.72%；52.49%） | 2.198x（3.64%；47.62%/58.15%） |

六个 workload 的四种配置均来自同一无统计开销 QEMU binary。sysbench 和 GAPBS
没有可区分于噪声的端到端收益；DaCapo both、`mcf` LP/both 有约 6%--7% 正信号，
但 CV 也在约 4.7%--7.8%，需要更多重复。PTW 在 DaCapo 和 `mcf` 虽分别有 65.97%
与 56.51% 命中，timing 仍约 -0.5%，说明命中不等于净收益。

stress baseline 的 real-time 速率出现两个约 150 bogo/s 和三个约 326--335 bogo/s
的簇，CV 达 38.2%；低簇只使用约一半 CPU 时间。按 CPU-time 速率取中位数后，base、
LP、PTW、both 分别为 327.88、336.57、347.80、357.18 bogo/s，对应约 1.027x、
1.061x、1.089x，但仍不能把该 workload 当成稳定证据。所有样本均保留。

nested 内层随机访存的中位数从 568.796 降至 LP 的 255.158 ns/access 和 both 的
258.788 ns/access；外层测量窗中位数只从 17.206 s 降至 16.311 s 和 16.493 s，
即约 1.055x 和 1.043x。PTW 内层为 514.245 ns/access（1.106x），外层为 1.024x。
热点收益被启动、I/O、内层 QEMU 其他工作和 Amdahl 定律显著稀释。

## 后续研究优先级

1. **先优化公共 fast path。** 统计生成代码中的 compare、分支、host-TLB/cache miss
   和 helper 边界，探索更紧凑的 tag/addend 布局、低冲突索引和受控的二路结构。
   公共路径每次访存都执行，潜在覆盖面远大于 refill-only 方案，但必须同时约束 TCG
   code size 和 host I-cache。
2. **把大页 cache 做成自适应旁路。** 根据主表/victim miss、LP 命中、flush/churn
   和实际节省的 fill 动态启停；加入 range-aware invalidation、ASID/PCID tag，并只在
   target 明确提供线性 translation span 时启用。
3. **按层自适应 PTW cache。** mcf/GAPBS 的 L2 命中较低而 L3 接近饱和，应比较仅查
   L3、不同 set/way、替换策略和非原子生产计数；用更长 mcf 与 shootdown 序列验证约
   2%--5% 的信号是否稳定。
4. **扩大到 system-mode 其他热点。** 若目标是通用端到端性能，优先 profile TB
   lookup/chaining、间接跳转、interrupt/timer、MMIO/device model 与 I/O batching；
   nested 场景还应拆分外层 SoftMMU、内层 KVM 和设备/启动时间。
5. **提升实验强度。** 当前已使用 P-Core、2.1 GHz 固定请求、nice=-20、禁用休眠、
   随机化顺序和无计数 timing binary；下一步应离线 SMT sibling、预热 guest、使用
   30--60 秒稳定窗口并至少重复 10 次，继续把正确性、机制计数和 timing 分开运行。

## 复现

```sh
./tlb-study/run-tests.sh
./tlb-study/run-optimization-suite.py \
  --workload sysbench --workload stress-tlb --workload dacapo \
  --workload mcf --workload gapbs --workload nested \
  --variant base --variant lp --variant ptw --variant both \
  --repetitions 3 --cpu 7 --nice -20 \
  --name-tag fixed4k-novictim-pcore7-fixed2100-nostats-v1 \
  --tlb-entries 4096 --victim-tlb off --shuffle-seed 20260910
./tlb-study/summarize-optimizations.py \
  tlb-study/results/opt-*-fixed4k-novictim-pcore7-fixed2100-\
nostats-v1-{base,lp,ptw,both}-r0[1-3]
./tlb-study/validate-results.py RESULTS... \
  --qemu-version 8.2.9 --require-provenance
```

最终微基准目录包含 `fixed2100-nostats-v1`，系统机制计数目录包含
`fixed2100-profile-v1`。比较前应核对 `provenance.qemu_binary.sha256`、运行选项和
TLB 配置，不要仅按目录名合并 timing 与 profile。
