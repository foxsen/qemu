# 固定 4096 项 SoftTLB、关闭 victim 的时间归因

## 结论

在 QEMU 8.2.9 x86-64 system mode、单 vCPU、single-thread TCG 下，将每个
MMU mode 的主 SoftTLB 固定为 4096 项并关闭 8-entry victim TLB 后，六个系统
负载中可由函数名归因的 SoftMMU 慢路径平均占 **13.39%**，中位数为 **13.05%**，
范围为 **0.47%--24.18%**。最高的 SPEC CPU2006 `429.mcf` train 仍未达到 30%。

这个结果不等价于 TACO 论文 38.1% 的完整访存模拟时间。当前 `perf` 分类包含
SoftTLB miss、refill、页表遍历、失效和相关支持函数，但不包含动态生成代码中
内联的 fast-hit lookup。因而可以得出的结论是：**固定 4096 项且关闭 victim 后，
可命名慢路径的平均占比没有接近 TACO 的 38.1%；完整访存模拟占比仍待对内联
fast path 做指令级归因。**

## 实验控制

- QEMU：8.2.9，`-accel tcg,thread=single`，单 vCPU；
- SoftTLB：`QEMU_SOFTMMU_TLB_ENTRIES=4096`，
  `QEMU_SOFTMMU_VICTIM_TLB=off`；
- 实验 cache：large-page cache 与 PTW cache 均关闭；
- 宿主：Intel Core i7-1260P，QEMU 绑定 P-Core logical CPU 7，SMT sibling 为
  CPU 6，QEMU nice 为 -20；
- CPU 6/7 使用 `intel_pstate`、`performance` governor，实验期间请求的
  scaling min/max 均为 2,100,000 kHz；硬件热保护和功耗限制仍可能降频；
- perf：`cpu_core/cycles/u`，997 Hz，直接附着 QEMU PID；
- cloud workload 使用 snapshot，guest THP 设为 `always`；准备、复制、编译和
  数据生成都在测量 barrier 之前完成；
- 六个目录均通过 QEMU 版本、provenance、perf 覆盖与分类一致性校验。

## 系统负载结果

每个负载当前为一次独立 perf 运行。百分比由采样周期权重计算；`core` 包含
SoftTLB/refill/PTW/失效等核心函数，`support` 包含这些路径使用的通用支持函数。
样本数足以做本轮瓶颈筛选，但发表版仍需重复运行并报告置信区间。

| workload | 测量窗 (s) | 有效样本 | core | support | named SoftMMU |
|---|---:|---:|---:|---:|---:|
| sysbench random 4 KiB write | 7.529 | 6,834 | 0.39% | 0.09% | **0.47%** |
| DaCapo 9.12 `avrora` | 100.359 | 100,696 | 7.75% | 0.95% | **8.70%** |
| GAPBS PageRank scale-20 | 18.371 | 16,187 | 10.12% | 1.37% | **11.49%** |
| nested QEMU/KVM random page | 19.173 | 15,874 | 12.45% | 2.17% | **14.62%** |
| stress-ng `tlb-shootdown` | 11.996 | 11,135 | 18.04% | 2.82% | **20.85%** |
| SPEC CPU2006 `429.mcf` train | 63.176 | 60,502 | 19.15% | 5.03% | **24.18%** |

六项等权算术平均为 13.39%，中位数为 13.05%。按样本数合并会得到 14.17%，
但该数字会让长负载获得更大权重，不作为跨 workload 的主结果。TACO 的 38.1%
比本轮等权均值高 24.71 个百分点；两者的 QEMU 版本、benchmark 集合和归因口径
均不同，不能将差异全部解释为固定 TLB 或关闭 victim 的效果。

## 压力微基准上界

128 MiB 随机页访问会刻意让 4096-entry 直接索引表持续 miss。有效 perf 轮次中，
guest THP 场景的 named SoftMMU 为 95.73% 和 94.88%，中位数 95.31%；4 KiB 页
场景为 95.01%、93.53% 和 95.08%，中位数 95.01%。首个 THP 轮次的测量窗口太短，
没有产生样本，已排除。

该结果说明当前分类在持续 miss 时确实能捕获主要开销，也给出了机制压力上界；
它不应与应用平均值混合。两个场景中最大的单一热点都是 `helper_ldub_mmu`，约占
44%--48%，随后是 `tlb_set_page_full`、`probe_access_internal`、
`mmu_translate` 和 `find_next_bit`。

### 固定宿主条件下的 cache 对照

同一环境下另做了不附着 perf 的 5 次 timing，对 LP、PTW 和二者同时启用进行
直接比较。表中为中位数；加速比为 base 除以候选值。

| guest page | base | LP | PTW | LP+PTW |
|---|---:|---:|---:|---:|
| THP | 333.775 ns（1.000x） | 251.789 ns（1.326x） | 348.535 ns（0.958x） | 253.236 ns（1.318x） |
| 4 KiB | 355.010 ns（1.000x） | 359.754 ns（0.987x） | 335.175 ns（1.059x） | 335.991 ns（1.057x） |

THP 下 LP 的命中/查询中位数为 98.40%，LP+PTW 的结果基本由 LP 决定；4 KiB
下 LP 命中率仅约 0.014%，而 PTW 有效查询几乎全部命中，组合结果基本由 PTW
决定。二者同时启用没有超过单独启用匹配该页大小的 cache：THP 慢约 0.57%，
4 KiB 慢约 0.24%，符合额外无效查询带来少量成本的解释。

除 4 KiB base 的一次 390.814 ns/access 离群值外，各组标准差/均值为
0.34%--1.34%；base THP 为 0.41%。中位数不受该离群点影响。相比先前未固定
宿主条件的数据，这组结果支持将较大波动主要归因于频率、调度和共享物理核干扰，
但未离线 SMT sibling，因此不能声称已经完全消除宿主噪声。

## 复现与后续

系统负载命令为：

```sh
python3 tlb-study/run-optimization-suite.py \
  --workload sysbench --workload stress-tlb --workload dacapo \
  --workload mcf --workload gapbs --workload nested \
  --variant base --repetitions 1 --cpu 7 --nice -20 \
  --name-tag fixed4k-novictim-pcore7-fixed2100-perfcore \
  --tlb-entries 4096 --victim-tlb off --perf \
  --perf-event cpu_core/cycles/u --no-perfmap
```

结果校验为：

```sh
python3 tlb-study/validate-results.py \
  tlb-study/results/opt-{sysbench,stress-tlb,dacapo,mcf,gapbs,nested}-\
fixed4k-novictim-pcore7-fixed2100-perfcore-base-r01 \
  --qemu-version 8.2.9 --require-provenance
```

若要严格回答是否复现 TACO 的 38.1%，下一步至少需要：给 JIT 内联 lookup 标记
精确地址范围，或构建语义等价的 outlined lookup 归因版本；使用与论文更接近的
Linux build、DaCapo、SPECint 和 multiprogrammed mix；并将固定/动态主表与
victim on/off 做成同一二进制、同一宿主控制下的析因实验。当前结果用于定位慢路径，
不用于声称完整访存模拟成本已经下降到 13.39%。
