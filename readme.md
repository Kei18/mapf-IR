auto record by github actions
===
date: 2021-02-12-06-20-33

commit
```
commit 8dfda27e50959f6c40c3c3d6692138c1548bd548
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Fri Feb 12 15:17:02 2021 +0900

    fix the version of googletest

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 249 | 58574 | 386 |
| winPIBT | 1 | 436 | 58117 | 388 |
| PushAndSwap | 1 | 359 | 10965159 | 54639 |
| HCA | 1 | 824 | 54784 | 386 |
| WHCA | 1 | 558 | 56589 | 386 |
| RevisitPP | 1 | 861 | 54725 | 392 |
| ECBS | 1 | 10277 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 346 | 58574 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 23837 | 678 | 42 |
| ICBS | 1 | 97 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10006 | 10828 | 79 |
| IR_SINGLE_PATHS | 1 | 682 | 11726 | 79 |
| IR_FIX_AT_GOALS | 1 | 273 | 11359 | 79 |
| IR_FOCUS_GOALS | 1 | 10003 | 9389 | 79 |
| IR_MDD | 1 | 10027 | 12323 | 79 |
| IR_BOTTLENECK | 1 | 10017 | 12426 | 79 |
| IR_HYBRID | 1 | 10020 | 9391 | 79 |
