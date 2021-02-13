auto record by github actions
===
date: 2021-02-13-10-35-32

commit
```
commit 7a8fccd3a3c90aa0885467030ef8d7bb2eae9e0a
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Sat Feb 13 19:32:19 2021 +0900

    modify cmake version

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 261 | 58574 | 386 |
| winPIBT | 1 | 423 | 58117 | 388 |
| PushAndSwap | 1 | 375 | 10965159 | 54639 |
| HCA | 1 | 867 | 54784 | 386 |
| WHCA | 1 | 639 | 56589 | 386 |
| RevisitPP | 1 | 848 | 54725 | 392 |
| ECBS | 1 | 12219 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 328 | 58574 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 24374 | 678 | 42 |
| ICBS | 1 | 75 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10015 | 10754 | 79 |
| IR_SINGLE_PATHS | 1 | 586 | 11726 | 79 |
| IR_FIX_AT_GOALS | 1 | 243 | 11359 | 79 |
| IR_FOCUS_GOALS | 1 | 10003 | 9389 | 79 |
| IR_MDD | 1 | 10016 | 12203 | 79 |
| IR_BOTTLENECK | 1 | 10015 | 12426 | 79 |
| IR_HYBRID | 1 | 10007 | 9390 | 79 |
