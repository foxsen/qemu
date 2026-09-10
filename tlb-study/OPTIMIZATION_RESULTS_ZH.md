# QEMU 8.2 SoftMMU Miss 路径优化结果

## 结论摘要

在 QEMU 8.2.9 x86-64 system mode、单 vCPU、single-thread TCG 上，大页
miss-cache 和非叶页表缓存都能显著加速刻意制造 miss 的微基准，但尚未证明它们是
通用的整机加速方案。最终微基准中，大页缓存达到 **1.254x**，PTW cache 达到
**1.195x**；真实负载的中位数变化则大多落在 -3.6% 到 +4.6%，且短负载噪声较大。

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
tcg,thread=single`、1 vCPU，并固定到 host CPU 2；cloud workload 使用 snapshot，
guest THP 设为 `always`。timing build 不含 `QEMU_TLB_PROFILE`，但实验 cache 的原子
统计仍开启，因此测得的 `on` 成本是保守值。宿主为 i7-1260P、`powersave`
governor，未做整机隔离，当前数字不是发表级置信区间。

最终 `mprotect` smoke 完成 32,768 次访问，checksum 为 4,177,920；期间大页缓存
命中 29,893 次并经历 27 次失效，PTW cache 经历 16 次失效，结果通过 PTW/TLB
一致性校验。nested 的全部 base/LP/PTW 运行都得到相同 checksum 66,846,720，且
64 MiB 映射中 63,488 KiB 确认为 THP。故障注入式的旧 nested 目录被保留用于说明
逐项清空和页大小语义错误，不纳入性能表。

## 最终微基准

每组为 128 MiB、128 passes、5 次重复；表内为中位数，括号内为
`mean +/- 95%` 正态近似描述区间。

| 场景 | off (ns/access) | on (ns/access) | 加速 |
|---|---:|---:|---:|
| 随机访问、guest 2 MiB THP、大页 cache | 268.814（273.496 +/- 8.855） | 214.335（215.167 +/- 2.275） | 1.254x |
| 随机访问、guest 4 KiB 页、PTW cache | 320.890（321.955 +/- 4.974） | 268.622（276.611 +/- 15.119） | 1.195x |

PTW profile smoke 进一步显示，4 KiB 随机访问启用缓存后大部分 walk 可从 L2
cache 直接跳到叶节点，典型 walk 从约 5 次层访问降到约 2 次。这证明收益来自实际
减少 guest page-table memory access，而不是单纯的计数器现象。

## 非内核真实负载

`performance ratio` 对普通负载取 `base wall median / variant wall median`；固定时长
stress-ng 取 bogo ops/s 中位数之比。LP/PTW 列是命中/查询比例。

| workload | n | LP ratio（命中率） | PTW ratio（命中率） |
|---|---:|---:|---:|
| sysbench random memory | 3 | 1.028x（30.02%） | 1.035x（70.66%） |
| stress-ng `tlb-shootdown` | 3 | 1.009x（13.84%） | 1.046x（80.48%） |
| SPEC CPU2006 `429.mcf` train | 3 | 0.964x（53.80%） | 1.020x（59.83%） |
| GAPBS PageRank scale-20 | 3 | 1.029x（88.32%） | 1.031x（51.80%） |
| DaCapo `avrora` | 3 | 0.994x（36.51%） | 0.985x（73.43%） |
| nested QEMU/KVM | 3 | 1.000x（47.27%） | 1.006x（52.07%） |

sysbench、mcf、GAPBS 来自同一较早 prototype binary；它们内部 A/B 可比，但早于
generation 与 nested 线性范围修复，不能与最终 binary 混合做统计。stress-ng、
DaCapo 和 nested 来自最终 binary。短至 5 秒的 sysbench/GAPBS 方向并不稳定；
DaCapo 的 LP/PTW 均无收益；mcf 的 PTW 三次均快于各自 baseline，是值得扩大样本的
弱信号。

stress-ng 三次 PTW 吞吐为 452.61、440.34、133.68 bogo ops/s；最后一个伴随明显
较低的 QEMU CPU 时间，是宿主干扰离群点，中位数仍由两个一致样本决定。一次 DaCapo
PTW 运行经历宿主休眠，guest 自报时间包含 suspend，已用 clean r04 替换；污染目录
保留但未进入表格。

nested 内层随机访存的中位数从 522.265 降至 243.520 ns/access，大页 cache 在热点
窗口内达到 2.145x；外层整个 workload 墙钟却是 12.755 对 12.758 秒，完全被启动、
I/O、内层 QEMU 其他工作和 Amdahl 定律淹没。PTW cache 的内层结果为 527.005
ns/access，也没有收益。

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
5. **提升实验强度。** 使用 performance governor、隔离物理核、预热 guest、30--60
   秒测量窗口、随机化或 ABBA 顺序和至少 10 次重复；在最终无计数 binary 上重新做
   `perf`，并把正确性、机制计数和 timing 分开运行。

## 复现

```sh
./tlb-study/run-tests.sh
./tlb-study/run-optimization-suite.py \
  --workload mcf --workload gapbs --workload nested \
  --variant base --variant lp --variant ptw --repetitions 3
./tlb-study/summarize-optimizations.py \
  tlb-study/results/opt-{mcf,gapbs,nested}-{base,lp,ptw}-r0[1-3]
./tlb-study/validate-results.py RESULTS... \
  --qemu-version 8.2.9 --require-provenance
```

最终微基准目录以 `final-timing-` 开头；最终 cloud 目录使用 `linear` tag。比较前应核对
`provenance.qemu_binary.sha256` 与 `provenance.git.tracked_diff_sha256`，不要仅按目录名
合并不同 prototype。
