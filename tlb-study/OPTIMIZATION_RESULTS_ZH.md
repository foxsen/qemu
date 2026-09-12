# QEMU 8.2 SoftMMU Miss 路径优化结果

## 结论摘要

在 QEMU 8.2.9 x86-64 system mode、单 vCPU、single-thread TCG 上，大页
miss-cache 和非叶页表缓存都能显著加速刻意制造 miss 的微基准，但尚未证明它们是
通用的整机加速方案。在固定 4096 项主表、关闭 victim、P-Core 锁定 2.1 GHz 且
timing build 去除 cache 原子计数后，大页缓存在 THP 随机微基准达到 **1.399x**，
PTW cache 在 4 KiB 随机微基准达到 **1.066x**。八负载、四配置、每配置三次的
无 perfmap 筛选中，`429.mcf` 的 LP 与 LP+PTW 分别达到 1.093x 和 1.156x；nested
热点达到 2.244x 和 2.269x，但端到端只有 1.064x 和 1.059x。`471.omnetpp` 的
LP 为 1.066x，但 base CV 为 4.66%；`483.xalancbmk` 基本持平。

加入自适应门控后的独立十轮重测进一步限定了结论：固定 4096 项、关闭 victim 时，
自适应组合的配对中位加速为 1.100x[1.053, 1.143]；默认动态 SoftTLB 加 victim
时为 0.998x[0.963, 1.037]，没有可辨收益。因此，“动态 SoftTLB 和 victim TLB
会过滤掉大部分 miss，使慢路径优化难以在应用级体现”这个判断成立。但机会并未
消失：剩余 miss 很贵，且在特定随机访存、二阶段翻译和频繁页表遍历场景中仍有
明确机制收益。研究目标应是识别并自适应覆盖这些场景，而不是增加所有访存的
fast-path 成本。

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
抑制休眠。timing build 不含 `QEMU_TLB_PROFILE`，cache lookup/hit 原子计数也只在
profile build 编译。系统 timing 显式关闭 perfmap，随机化执行顺序，八个 workload
的四种配置均重复 3 次。主矩阵中一次合盖 suspend 未被 inhibitor 拦截；受影响的
GAPBS 样本被保留为诊断目录并用干净样本替换。SMT sibling 未离线，当前结果仍是
趋势而非发表级显著性结论。

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
| sysbench random memory | 3 | 1.002x（0.15%；2.35%） | 1.003x（0.59%；67.78%） | 1.001x（0.08%；32.58%/73.94%） |
| stress-ng `tlb-shootdown` | 3 | 1.025x（1.47%；17.96%） | 1.039x（1.34%；79.91%） | 1.070x（0.99%；19.22%/90.73%） |
| SPEC CPU2006 `429.mcf` train | 3 | 1.093x（0.60%；68.55%） | 1.026x（1.62%；56.51%） | 1.156x（2.81%；67.96%/99.81%） |
| GAPBS PageRank scale-20 | 3 | 0.997x（0.74%；81.35%） | 0.977x（3.09%；59.04%） | 0.995x（8.80%；78.65%/95.42%） |
| DaCapo `avrora` | 3 | 0.946x（3.08%；33.00%） | 0.955x（10.99%；65.97%） | 0.985x（5.37%；26.09%/98.93%） |
| nested QEMU/KVM hotspot | 3 | 2.244x（4.11%；49.87%） | 0.970x（2.64%；52.49%） | 2.269x（3.80%；47.62%/58.15%） |
| SPEC CPU2006 `471.omnetpp` train | 3 | 1.066x（0.58%；—） | 1.041x（2.78%；—） | 1.026x（3.10%；—） |
| SPEC CPU2006 `483.xalancbmk` train | 3 | 1.002x（0.38%；—） | 0.993x（2.59%；—） | 0.985x（2.38%；—） |

八个 workload 的四种配置均来自同一无统计开销、无 perfmap QEMU binary；命中率
仍来自独立 profile，新增的 omnetpp/xalancbmk 尚无 profile，故不填命中率。
sysbench、GAPBS 和 xalancbmk 没有可区分于噪声的端到端收益；DaCapo 三种候选均
慢于 base，PTW 的 CV 达 10.99%。`mcf` LP/both 的 1.093x/1.156x 是当前最明确的
应用级正信号。omnetpp LP 为 1.066x，但 base 墙钟 CV 为 4.66%，仍需更多重复。

stress-ng 的主指标 bogo/s 在 LP、PTW、both 下分别为 1.025x、1.039x 和 1.070x，
但固定时长的外层墙钟分别只有 1.001x、0.997x 和 1.001x。它说明单位时间完成工作量
可能变化，但不能用固定 timeout 的墙钟证明端到端缩短。

nested 内层随机访存的中位数从 526.092 降至 LP 的 234.495 ns/access 和 both 的
231.883 ns/access，达到 2.244x 和 2.269x；外层测量窗中位数只从 16.173 s 降至
15.201 s 和 15.279 s，即 1.064x 和 1.059x。PTW 内层为 542.484 ns/access
（0.970x），外层为 1.017x。热点收益被启动、I/O、内层 QEMU 其他工作和 Amdahl
定律显著稀释。

一次 suspend 后立即开始的 GAPBS both 样本为 13.265 s；该目录已保留并由
6.304 s 的干净运行替换。替换后三次 both 的中位数为 5.555 s、CV 为 8.80%。
完整异常说明、主指标和统一墙钟表见 `ALL8_NOPERFMAP_RESULTS_ZH.md`。

## 自适应门控重测

门控为 LP 和 PTW 分别维护采样、启用和旁路阶段。采样窗为 2048 次候选事件，启用
和旁路窗均为 16384 次；LP 以每 32 次查询至少一次命中为阈值，PTW 以每 8 次查询
至少跳过一层为阈值。查询和插入均可旁路，公共 SoftTLB 命中路径保持不变。

| SoftTLB | 方案 | n | 墙钟中位数（CV） | 配对中位加速[95%区间] | 胜出 |
|---|---|---:|---:|---:|---:|
| 固定 4096、无 victim | base | 10 | 53.697 s（5.97%） | — | — |
| 固定 4096、无 victim | LP+PTW | 10 | 49.196 s（3.49%） | 1.101x[1.065, 1.159] | 10/10 |
| 固定 4096、无 victim | 自适应组合 | 10 | 48.463 s（22.91%） | 1.100x[1.053, 1.143] | 9/10 |
| 动态、8 项 victim | base | 10 | 40.590 s（3.35%） | — | — |
| 动态、8 项 victim | LP+PTW | 10 | 40.647 s（2.80%） | 0.986x[0.971, 1.027] | 3/10 |
| 动态、8 项 victim | 自适应组合 | 10 | 40.620 s（11.29%） | 0.998x[0.963, 1.037] | 5/10 |

固定配置的自适应样本有一轮 86.929 s，动态 PTW 有一轮 91.707 s；系统日志没有
对应的 suspend、热保护、内存不足或 NVMe 错误，因此保留长尾，不能把地址布局、
直接映射冲突或未观测调度扰动中的任一项写成已证实原因。4 KiB 负对照各 30 轮，
base、LP、自适应 LP 的中位延迟分别为 336.509、337.153 和 336.972 ns/access；
自适应 profile 旁路 738314/861993（85.7%）候选事件。门控降低明显无复用区间的
查询量和方差，但不保证消除端到端回退。

## 后续研究优先级

1. **先优化公共 fast path。** 统计生成代码中的 compare、分支、host-TLB/cache miss
   和 helper 边界，探索更紧凑的 tag/addend 布局、低冲突索引和受控的二路结构。
   公共路径每次访存都执行，潜在覆盖面远大于 refill-only 方案，但必须同时约束 TCG
   code size 和 host I-cache。
2. **细化已有自适应门控。** 当前只按 CPU 聚合机制事件；应比较按 MMU mode、地址
   空间和执行阶段维护状态，在线调节窗口与阈值，并加入 range-aware invalidation
   和 ASID/PCID tag。
3. **按层选择 PTW cache。** mcf/GAPBS 的 L2 命中较低而 L3 接近饱和，应比较仅查
   L3、不同 set/way 与替换策略，并在默认动态 TLB、更多负载和多宿主上确认是否能
   避免当前 PTW 负收益。
4. **扩大到 system-mode 其他热点。** 若目标是通用端到端性能，优先 profile TB
   lookup/chaining、间接跳转、interrupt/timer、MMIO/device model 与 I/O batching；
   nested 场景还应拆分外层 SoftMMU、内层 KVM 和设备/启动时间。
5. **提升实验强度。** 当前已使用 P-Core、2.1 GHz 固定请求、nice=-20、休眠
   inhibitor、随机化顺序和无计数 timing binary；但 inhibitor 未完全拦截合盖
   suspend。下一步应系统级禁用休眠、离线 SMT sibling、预热 guest、使用 30--60
   秒稳定窗口并至少重复 10 次，继续把正确性、机制计数和 timing 分开运行。

## 复现

```sh
./tlb-study/run-tests.sh
./tlb-study/run-optimization-suite.py \
  --workload sysbench --workload stress-tlb --workload dacapo \
  --workload mcf --workload gapbs --workload nested \
  --workload omnetpp --workload xalancbmk \
  --variant base --variant lp --variant ptw --variant both \
  --repetitions 3 --cpu 7 --nice -20 \
  --name-tag all8-fixed4096-novictim-noperfmap \
  --tlb-entries 4096 --victim-tlb off --shuffle-seed 20260910 \
  --no-perfmap
./tlb-study/summarize-optimizations.py \
  tlb-study/results/opt-*-all8-fixed4096-novictim-noperfmap-\
{base,lp,ptw,both}-r0[1-3]
./tlb-study/validate-results.py RESULTS... \
  --qemu-version 8.2.9 --require-provenance
```

最终八负载 timing 目录包含 `all8-fixed4096-novictim-noperfmap`，系统机制计数目录
包含 `fixed2100-profile-v1`。比较前应核对 `provenance.qemu_binary.sha256`、运行
选项和 TLB 配置，不要仅按目录名合并 timing 与 profile。
