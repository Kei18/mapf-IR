auto record by github actions
===
date: 2023-01-14-02-59-31

commit
```
commit 920aba9db9ab7f47be7e2fcb99f026c40956a664
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Sat Jan 14 11:56:22 2023 +0900

    remove secrets

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 252 | 58574 | 386 |
| winPIBT | 1 | 369 | 58117 | 388 |
| PushAndSwap | 1 | 355 | 10965159 | 54639 |
| HCA | 1 | 804 | 54784 | 386 |
| WHCA | 1 | 556 | 56588 | 386 |
| RevisitPP | 1 | 667 | 54725 | 392 |
| ECBS | 1 | 10868 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 306 | 58574 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 20299 | 678 | 42 |
| ICBS | 1 | 76 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10019 | 10681 | 79 |
| IR_SINGLE_PATHS | 1 | 659 | 11726 | 79 |
| IR_FIX_AT_GOALS | 1 | 258 | 11359 | 79 |
| IR_FOCUS_GOALS | 1 | 9456 | 9387 | 79 |
| IR_MDD | 1 | 10008 | 12310 | 79 |
| IR_BOTTLENECK | 1 | 10029 | 12426 | 79 |
| IR_HYBRID | 1 | 10005 | 9387 | 79 |
