auto record by github actions
===
date: 2021-02-11-03-58-16

commit
```
commit f5c520c1d9df8d9831041b719c82267df76dbeb9
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Thu Feb 11 12:54:15 2021 +0900

    update path table

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 251 | 58791 | 386 |
| winPIBT | 1 | 407 | 58263 | 395 |
| PushAndSwap | 1 | 1027 | 1885902 | 8292 |
| HCA | 1 | 803 | 54784 | 386 |
| WHCA | 1 | 545 | 56589 | 386 |
| RevisitPP | 1 | 838 | 54725 | 392 |
| ECBS | 1 | 9618 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 310 | 58791 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 21246 | 678 | 42 |
| ICBS | 1 | 69 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10004 | 10597 | 79 |
| IR_SINGLE_PATHS | 1 | 581 | 11451 | 79 |
| IR_FIX_AT_GOALS | 1 | 1045 | 11506 | 79 |
| IR_FOCUS_GOALS | 1 | 10002 | 9381 | 79 |
| IR_MDD | 1 | 10005 | 12451 | 79 |
| IR_BOTTLENECK | 1 | 10030 | 12489 | 79 |
| IR_HYBRID | 1 | 10002 | 9395 | 79 |
