auto record by github actions
===
date: 2021-02-14-03-56-48

commit
```
commit 0b75effc86c2b3b4740c63e81be8cedb8179d3a1
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Sun Feb 14 12:53:46 2021 +0900

    integrate IR solvers

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 280 | 58574 | 386 |
| winPIBT | 1 | 473 | 58117 | 388 |
| PushAndSwap | 1 | 401 | 10965159 | 54639 |
| HCA | 1 | 1001 | 54784 | 386 |
| WHCA | 1 | 674 | 56589 | 386 |
| RevisitPP | 1 | 966 | 54725 | 392 |
| ECBS | 1 | 12121 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 335 | 58574 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 25518 | 678 | 42 |
| ICBS | 1 | 81 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10007 | 10957 | 79 |
| IR_SINGLE_PATHS | 1 | 760 | 11726 | 79 |
| IR_FIX_AT_GOALS | 1 | 272 | 11359 | 79 |
| IR_FOCUS_GOALS | 1 | 10004 | 9389 | 79 |
| IR_MDD | 1 | 10009 | 12342 | 79 |
| IR_BOTTLENECK | 1 | 10019 | 12426 | 79 |
| IR_HYBRID | 1 | 10017 | 9391 | 79 |
