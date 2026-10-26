# RM75-6FB offline motion-planning benchmark

Fixed seed: `7506`; repetitions per scenario: `100`; output: `50 Hz`.

This is an offline numerical benchmark. No robot command was published.

| Solver | Scenario | Success | Mean ms | P95 ms | P99 ms | Max pos mm | Max rot deg | Max v | Max a | Max jerk |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| CasADi 3.7.2 / Ipopt | short_line | 100.00% | 4.273 | 7.219 | 9.999 | 0.087 | 0.009 | 0.057 | 0.183 | 2.000 |
| CasADi 3.7.2 / Ipopt | large_cartesian | 90.00% | 252.848 | 446.706 | 641.315 | 1.669 | 0.203 | 0.258 | 0.390 | 2.000 |
| CasADi 3.7.2 / Ipopt | random_6d | 70.00% | 223.852 | 472.558 | 602.438 | 1.836 | 0.175 | 0.240 | 0.377 | 2.000 |
| CasADi 3.7.2 / Ipopt | near_singular | 100.00% | 2.778 | 5.397 | 5.616 | 0.084 | 0.013 | 0.044 | 0.160 | 2.000 |
| CasADi 3.7.2 / Ipopt | near_joint_limit | 99.00% | 7.936 | 15.836 | 19.505 | 0.060 | 0.006 | 0.064 | 0.193 | 2.000 |

## Failure counts

- `CasADi 3.7.2 / Ipopt/large_cartesian`: 10 failed paths. Full reasons are retained in the JSON and CSV.
- `CasADi 3.7.2 / Ipopt/random_6d`: 30 failed paths. Full reasons are retained in the JSON and CSV.
- `CasADi 3.7.2 / Ipopt/near_joint_limit`: 1 failed paths. Full reasons are retained in the JSON and CSV.
