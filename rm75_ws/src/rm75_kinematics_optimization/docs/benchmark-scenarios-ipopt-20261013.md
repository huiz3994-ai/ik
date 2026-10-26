# Scenario benchmark and constrained IK — 2026-10-13

This run supersedes the earlier undifferentiated random benchmark for algorithm conclusions.
Seed 7506 deterministically generates six scenario classes and stores every target joint
vector and independent seed in the raw CSV. C++ solvers ran 10,000 targets each; CasADi/Ipopt
ran the exact first 1,000 targets from that same CSV. Position-only success ignores rotation;
all other scenarios require <=2 mm and <=2 degrees plus legal joint limits.

## Overall

| Solver | Samples | Success | Mean/P99 time | Successful position error | Orientation-task error |
|---|---:|---:|---:|---:|---:|
| KDL NR/JL | 10,000 | 64.84% | 0.469 / 1.203 ms | ~0.0001 mm | ~0.00002° |
| fixed DLS | 10,000 | 79.06% | 0.109 / 0.266 ms | 0.917 mm | 0.208° |
| adaptive DLS | 10,000 | 81.42% | 0.147 / 1.055 ms | 1.092 mm | 0.275° |
| adaptive DLS + centering | 10,000 | 81.40% | 0.163 / 1.165 ms | 1.046 mm | 0.299° |
| CasADi 3.7.2 / Ipopt | 1,000 | 92.30% | 3.814 / 9.257 ms | 0.495 mm | 0.119° |

All rows had zero numerical anomalies. Ipopt excludes one-time symbolic graph construction
from solve latency but includes each NLP call. It is 20–35x slower than adaptive DLS and is
therefore an offline planning/reseeding solver, not a Touch servo candidate.

## Scenario success rates

| Scenario | KDL | Fixed DLS | Adaptive DLS | + Centering | Ipopt |
|---|---:|---:|---:|---:|---:|
| continuous small step | 99.58% | 100% | 100% | 100% | 100% |
| large initial error | 67.31% | 78.52% | 80.20% | 76.48% | 89.82% |
| near joint limit | 59.75% | 62.93% | 64.67% | 60.05% | 79.04% |
| near singular | 32.65% | 85.95% | 90.70% | 94.96% | 93.98% |
| random 6D | 65.87% | 67.37% | 70.97% | 69.35% | 92.81% |
| random position | 63.89% | 79.60% | 82.00% | 87.58% | 98.20% |

Adaptive damping provides a measured near-singular gain of 4.75 percentage points over fixed
DLS. Joint centering further raises near-singular and position-only success, but reduces
large-error, near-limit and random-6D convergence; it should remain a tunable secondary task,
not be enabled blindly. Across the full suite it increases mean minimum joint-limit margin
from 1.036 to 1.078 rad (+4.1%) while leaving overall success essentially unchanged.

Artifacts: `rm75_6fb_scenarios_10000.csv/json`, `rm75_6fb_ipopt_shared_1000.csv/json`, and
`rm75_6fb_by_scenario.json` under `benchmark/results/`.
