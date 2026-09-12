# 固定 SoftTLB、关闭 victim 的无 perfmap 八负载结果

## 实验范围

本轮在 QEMU 8.2.9 x86-64 system mode、单 vCPU、single-thread TCG 上比较
`base`、`LP`、`PTW` 和 `LP+PTW`。主 SoftTLB 固定为 4096 项，victim TLB 关闭，
客户机 THP 设为 `always`。QEMU 绑定 i7-1260P 的 P-Core logical CPU 7，
nice 为 -20；同一物理核的 CPU 6/7 在运行期间将 scaling min/max 请求为
2.1 GHz。timing binary 不包含 profile 原子计数，全部运行显式使用
`--no-perfmap`。

八个 workload、四种配置各重复三次，共 96 个有效样本。运行顺序按 repetition
使用固定种子随机化。所有结果均通过远端返回码、provenance、QEMU 版本、固定 TLB
容量、victim 状态和 THP 检查。

## 主指标汇总

sysbench 使用 MiB/s，stress-ng 使用 real-time bogo/s，DaCapo 使用客户机自报
毫秒，nested 使用内层随机访存 ns/access；其余 workload 使用测量窗墙钟。性能比
大于 1 表示更快，括号内为候选配置主指标的样本 CV。

| workload | base 中位数 | LP | PTW | LP+PTW |
|---|---:|---:|---:|---:|
| sysbench random memory | 159.310 MiB/s | 1.002x（0.15%） | 1.003x（0.59%） | 1.001x（0.08%） |
| stress-ng `tlb-shootdown` | 346.470 bogo/s | 1.025x（1.47%） | 1.039x（1.34%） | 1.070x（0.99%） |
| SPEC CPU2006 `429.mcf` train | 51.926 s | 1.093x（0.60%） | 1.026x（1.62%） | 1.156x（2.81%） |
| GAPBS PageRank scale-20 | 5.526 s | 0.997x（0.74%） | 0.977x（3.09%） | 0.995x（8.80%） |
| DaCapo `avrora` | 73,699 ms | 0.946x（3.08%） | 0.955x（10.99%） | 0.985x（5.37%） |
| nested QEMU/KVM hotspot | 526.092 ns/access | 2.244x（4.11%） | 0.970x（2.64%） | 2.269x（3.80%） |
| SPEC CPU2006 `471.omnetpp` train | 671.088 s | 1.066x（0.58%） | 1.041x（2.78%） | 1.026x（3.10%） |
| SPEC CPU2006 `483.xalancbmk` train | 721.315 s | 1.002x（0.38%） | 0.993x（2.59%） | 0.985x（2.38%） |

## 端到端墙钟汇总

为避免把 workload 内部热点指标解释成整机加速，下面统一给出测量窗墙钟。表中
候选列仍为 `base median / candidate median`，括号内为候选墙钟 CV。

| workload | base wall | LP | PTW | LP+PTW |
|---|---:|---:|---:|---:|
| sysbench | 6.761 s | 1.006x（0.14%） | 1.006x（0.53%） | 1.006x（0.75%） |
| stress-ng | 11.364 s | 1.001x（0.19%） | 0.997x（0.40%） | 1.001x（1.88%） |
| `429.mcf` | 51.926 s | 1.093x（0.60%） | 1.026x（1.62%） | 1.156x（2.81%） |
| GAPBS PR | 5.526 s | 0.997x（0.74%） | 0.977x（3.09%） | 0.995x（8.80%） |
| DaCapo | 81.688 s | 0.947x（3.33%） | 0.964x（10.10%） | 0.984x（5.38%） |
| nested QEMU/KVM | 16.173 s | 1.064x（0.55%） | 1.017x（1.40%） | 1.059x（1.10%） |
| `471.omnetpp` | 671.088 s | 1.066x（0.58%） | 1.041x（2.78%） | 1.026x（3.10%） |
| `483.xalancbmk` | 721.315 s | 1.002x（0.38%） | 0.993x（2.59%） | 0.985x（2.38%） |

## 数据解释

- `429.mcf` 是当前最明确的应用级正信号：LP、PTW 和 LP+PTW 分别为 1.093x、
  1.026x 和 1.156x，且候选 CV 为 0.60%--2.81%。
- nested 的大页热点达到 2.244x--2.269x，但外层测量窗只有 1.059x--1.064x，
  再次体现 Amdahl 稀释。
- `471.omnetpp` 的 LP 为 1.066x，但 base 墙钟 CV 为 4.66%，三次重复不足以把它
  定性为稳定收益。`483.xalancbmk` 基本持平或略有回退。
- stress-ng 的 bogo/s 最多提高 7.0%，固定时长墙钟则基本不变；sysbench 和
  GAPBS 也没有可区分于噪声的端到端收益。
- DaCapo 三种候选均慢于 base，PTW 的 CV 达 10.99%。因此 cache 命中率不能单独
  预测净收益，也不能把定向微基准收益外推到所有真实负载。

本轮仍只有三次重复，SMT sibling 在线，尚未计算置信区间。上述结果适合作为负载
筛选和后续设计依据，不应表述成发表级统计显著性结论。

## 异常样本处理

主矩阵运行期间宿主在 2026-09-11 18:21:30--18:46:57 发生一次 suspend。
当时 `GAPBS/both/r02` 正处于准备阶段，正式测量在恢复后约 14 秒开始，得到
13.265 s，明显偏离另外两次的 5.555 s 和 5.344 s。原目录保留为：

```text
opt-gapbs-all8-fixed4096-novictim-noperfmap-both-r02-invalid-postsuspend-20260911T1847CST
```

2026-09-12 的干净替代样本为 6.304 s，返回码为 0，未生成 perfmap；替代运行期间
没有发生 suspend。最终汇总只使用替代样本，不通过隐藏异常值或取中位数掩盖环境
问题。

## 复现与校验

```sh
./tlb-study/run-optimization-suite.py \
  --workload sysbench --workload stress-tlb --workload dacapo \
  --workload mcf --workload gapbs --workload nested \
  --workload omnetpp --workload xalancbmk \
  --variant base --variant lp --variant ptw --variant both \
  --repetitions 3 --cpu 7 --nice -20 \
  --tlb-entries 4096 --victim-tlb off \
  --shuffle-seed 20260910 \
  --name-tag all8-fixed4096-novictim-noperfmap --no-perfmap

./tlb-study/summarize-optimizations.py \
  tlb-study/results/opt-*-all8-fixed4096-novictim-noperfmap-\
{base,lp,ptw,both}-r0[1-3]

./tlb-study/validate-results.py RESULTS... \
  --qemu-version 8.2.9 --require-provenance
```

校验器对最终 96 个目录全部报告 `OK [wall_timing]`。

## 后续自适应重测

上述八负载矩阵用于负载筛选，保持原始三次重复结果不变。加入自适应门控后的
`429.mcf` 正式重测采用新的统一 binary 和运行条件，不与本页旧批次拼接：固定
4096 项、关闭 victim 的后十轮中，LP+PTW 和自适应 LP+PTW 的配对中位加速分别为
1.101x[1.065, 1.159] 和 1.100x[1.053, 1.143]；QEMU 默认动态 SoftTLB 加 8 项
victim 时，LP、PTW、LP+PTW 和自适应组合分别为 0.995x、0.975x、0.986x 和
0.998x，四者均未给出正向证据。4 KiB、固定 64 项的 30 轮负对照中，自适应 LP
旁路 85.7% 候选查询，延迟中位数与基线不可区分，并把 CV 从始终启用 LP 的
12.70% 恢复到 6.02%（基线 5.90%）。完整方法、区间及异常说明见论文草稿第 5.3 节。
