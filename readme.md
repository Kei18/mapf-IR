auto record by github actions
===
date: 2021-01-30-06-43-48

commit
```
commit 20c02a05272c5ec43cb92286121630f20cf25bea
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Sat Jan 30 15:40:53 2021 +0900

    add push and swap

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 254 | 58791 | 386 |
| PushAndSwap | 1 | 342 | 10965159 | 54639 |
| HCA | 1 | 3691 | 54784 | 386 |
| WHCA | 1 | 2179 | 56589 | 386 |
| RevisitPP | 1 | 3787 | 54725 | 392 |
| ECBS | 1 | 14426 | 54669 | 389 |
| PIBT_COMPLETE | 1 | 306 | 58791 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 22322 | 678 | 42 |
| ICBS | 1 | 74 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10003 | 10684 | 79 |
| IR_SINGLE_PATHS | 1 | 1850 | 11451 | 79 |
| IR_FIX_AT_GOALS | 1 | 3724 | 11506 | 79 |
| IR_FOCUS_GOALS | 1 | 10004 | 9382 | 79 |
| IR_MDD | 1 | 10039 | 12480 | 79 |
| IR_BOTTLENECK | 1 | 10014 | 12489 | 79 |
| IR_HYBRID | 1 | 10005 | 9398 | 79 |
