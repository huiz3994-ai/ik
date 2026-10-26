# RM75-6FB offline motion-planning benchmark

Fixed seed: `7506`; repetitions per scenario: `1000`; output: `50 Hz`.

This is an offline numerical benchmark. No robot command was published.

| Solver | Scenario | Success | Mean ms | P95 ms | P99 ms | Max pos mm | Max rot deg | Max v | Max a | Max jerk |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| kdl | short_line | 99.90% | 0.043 | 0.073 | 0.103 | 0.001 | 0.000 | 0.139 | 0.285 | 2.000 |
| kdl | large_cartesian | 71.80% | 2.466 | 4.528 | 14.052 | 0.001 | 0.000 | 0.250 | 0.381 | 2.000 |
| kdl | random_6d | 64.00% | 2.919 | 6.515 | 19.110 | 0.001 | 0.000 | 0.244 | 0.378 | 2.000 |
| kdl | near_singular | 37.60% | 0.216 | 1.019 | 1.088 | 0.001 | 0.000 | 0.266 | 0.394 | 2.000 |
| kdl | near_joint_limit | 99.80% | 0.051 | 0.093 | 0.115 | 0.001 | 0.000 | 0.167 | 0.310 | 2.000 |
| fixed_dls | short_line | 100.00% | 0.027 | 0.044 | 0.055 | 0.989 | 0.472 | 0.059 | 0.185 | 1.999 |
| fixed_dls | large_cartesian | 78.50% | 1.411 | 2.791 | 3.325 | 1.000 | 0.555 | 0.260 | 0.392 | 2.000 |
| fixed_dls | random_6d | 67.90% | 1.414 | 2.987 | 3.520 | 1.000 | 0.571 | 0.258 | 0.389 | 2.000 |
| fixed_dls | near_singular | 100.00% | 0.023 | 0.042 | 0.052 | 0.991 | 0.565 | 0.053 | 0.174 | 2.000 |
| fixed_dls | near_joint_limit | 100.00% | 0.033 | 0.058 | 0.069 | 0.244 | 0.070 | 0.062 | 0.190 | 2.000 |
| adaptive_dls | short_line | 100.00% | 0.026 | 0.042 | 0.055 | 0.989 | 0.472 | 0.054 | 0.177 | 2.000 |
| adaptive_dls | large_cartesian | 67.80% | 2.791 | 8.386 | 13.962 | 1.000 | 0.569 | 0.230 | 0.366 | 2.000 |
| adaptive_dls | random_6d | 58.60% | 2.754 | 7.594 | 15.489 | 1.000 | 0.564 | 0.240 | 0.374 | 2.000 |
| adaptive_dls | near_singular | 100.00% | 0.022 | 0.040 | 0.052 | 0.991 | 0.565 | 0.042 | 0.154 | 2.000 |
| adaptive_dls | near_joint_limit | 100.00% | 0.031 | 0.056 | 0.067 | 0.999 | 0.385 | 0.062 | 0.190 | 2.000 |
| adaptive_dls_nullspace | short_line | 97.10% | 0.030 | 0.051 | 0.101 | 0.997 | 0.472 | 0.062 | 0.186 | 2.000 |
| adaptive_dls_nullspace | large_cartesian | 38.30% | 1.630 | 4.957 | 8.393 | 1.000 | 0.544 | 0.258 | 0.385 | 2.000 |
| adaptive_dls_nullspace | random_6d | 37.80% | 1.585 | 4.996 | 7.832 | 1.000 | 0.550 | 0.248 | 0.382 | 2.000 |
| adaptive_dls_nullspace | near_singular | 100.00% | 0.023 | 0.041 | 0.054 | 0.991 | 0.565 | 0.042 | 0.154 | 2.000 |
| adaptive_dls_nullspace | near_joint_limit | 75.00% | 0.052 | 0.133 | 0.207 | 0.998 | 0.399 | 0.228 | 0.365 | 2.000 |

## Failure counts

- `kdl/short_line`: 1 failed paths. Full reasons are retained in the JSON and CSV.
- `kdl/large_cartesian`: 282 failed paths. Full reasons are retained in the JSON and CSV.
- `kdl/random_6d`: 360 failed paths. Full reasons are retained in the JSON and CSV.
- `kdl/near_singular`: 624 failed paths. Full reasons are retained in the JSON and CSV.
- `kdl/near_joint_limit`: 2 failed paths. Full reasons are retained in the JSON and CSV.
- `fixed_dls/large_cartesian`: 215 failed paths. Full reasons are retained in the JSON and CSV.
- `fixed_dls/random_6d`: 321 failed paths. Full reasons are retained in the JSON and CSV.
- `adaptive_dls/large_cartesian`: 322 failed paths. Full reasons are retained in the JSON and CSV.
- `adaptive_dls/random_6d`: 414 failed paths. Full reasons are retained in the JSON and CSV.
- `adaptive_dls_nullspace/short_line`: 29 failed paths. Full reasons are retained in the JSON and CSV.
- `adaptive_dls_nullspace/large_cartesian`: 617 failed paths. Full reasons are retained in the JSON and CSV.
- `adaptive_dls_nullspace/random_6d`: 622 failed paths. Full reasons are retained in the JSON and CSV.
- `adaptive_dls_nullspace/near_joint_limit`: 250 failed paths. Full reasons are retained in the JSON and CSV.
