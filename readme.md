auto record by github actions
===
date: 2021-02-13-14-38-41

commit
```
commit 24d72b0d31334f168a294f61892bd8548b2a30e5
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Sat Feb 13 23:35:26 2021 +0900

    refactoring

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 243 | 58574 | 386 |
| winPIBT | 1 | 389 | 58117 | 388 |
| PushAndSwap | 1 | 318 | 10965159 | 54639 |
| HCA | 1 | 830 | 54784 | 386 |
| WHCA | 1 | 513 | 56589 | 386 |
| RevisitPP | 1 | 830 | 54725 | 392 |
| ECBS | 1 | 9462 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 287 | 58574 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 20941 | 678 | 42 |
| ICBS | 1 | 61 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10011 | 11371 | 79 |
| IR_SINGLE_PATHS | 1 | 695 | 11726 | 79 |
| IR_FIX_AT_GOALS | 1 | 245 | 11359 | 79 |
| IR_FOCUS_GOALS | 1 | 10004 | 9388 | 79 |
| IR_MDD | 1 | 10014 | 12317 | 79 |
| IR_BOTTLENECK | 1 | 10027 | 12426 | 79 |
| IR_HYBRID | 1 | 10011 | 9390 | 79 |
