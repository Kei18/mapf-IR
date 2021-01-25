auto record by github actions
===
date: 2021-01-25-06-07-00

commit
```
commit 4fc0fc6e7577741cab03dd074d0ac4506fcd3b03
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Mon Jan 25 15:03:36 2021 +0900

    add tests

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 18 | 12489 | 79 |
| HCA | 1 | 1332 | 10126 | 79 |
| WHCA | 1 | 861 | 11865 | 79 |
| RevisitPP | 1 | 2200 | 9603 | 79 |
| ECBS | 1 | 6022 | 9481 | 79 |
| PIBT_COMPLETE | 1 | 22 | 12489 | 79 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 19243 | 678 | 42 |
| ICBS | 1 | 117 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10004 | 10832 | 79 |
| IR_SINGLE_PATHS | 1 | 2142 | 11451 | 79 |
| IR_FIX_AT_GOALS | 1 | 4051 | 11506 | 79 |
| IR_FOCUS_GOALS | 1 | 10004 | 9366 | 79 |
| IR_MDD | 1 | 10026 | 12449 | 79 |
| IR_BOTTLENECK | 1 | 10512 | 12489 | 79 |
| IR_HYBRID | 1 | 10007 | 9713 | 79 |
