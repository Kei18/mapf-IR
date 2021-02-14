auto record by github actions
===
date: 2021-02-14-09-23-00

commit
```
commit b247a02759a1ada6951e3dfbed2b8f91a5794028
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Sun Feb 14 18:20:06 2021 +0900

    delete lib_solver

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 240 | 58574 | 386 |
| winPIBT | 1 | 444 | 58117 | 388 |
| PushAndSwap | 1 | 359 | 10965159 | 54639 |
| HCA | 1 | 881 | 54784 | 386 |
| WHCA | 1 | 655 | 56588 | 386 |
| RevisitPP | 1 | 869 | 54725 | 392 |
| ECBS | 1 | 11551 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 299 | 58574 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 21810 | 678 | 42 |
| ICBS | 1 | 63 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10012 | 11398 | 79 |
| IR_SINGLE_PATHS | 1 | 651 | 11726 | 79 |
| IR_FIX_AT_GOALS | 1 | 255 | 11359 | 79 |
| IR_FOCUS_GOALS | 1 | 10002 | 9388 | 79 |
| IR_MDD | 1 | 10011 | 12313 | 79 |
| IR_BOTTLENECK | 1 | 10010 | 12426 | 79 |
| IR_HYBRID | 1 | 10013 | 9387 | 79 |
