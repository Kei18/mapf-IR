auto record by github actions
===
date: 2021-01-27-07-06-15

commit
```
commit b2b490ab51679e3034a5e00690699886952277cf
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Wed Jan 27 16:02:47 2021 +0900

    modify bugs

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 276 | 58791 | 386 |
| HCA | 1 | 3781 | 54784 | 386 |
| WHCA | 1 | 2136 | 56589 | 386 |
| RevisitPP | 1 | 4147 | 54725 | 392 |
| ECBS | 1 | 14858 | 54669 | 389 |
| PIBT_COMPLETE | 1 | 377 | 58791 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 24817 | 678 | 42 |
| ICBS | 1 | 78 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10005 | 10832 | 79 |
| IR_SINGLE_PATHS | 1 | 2089 | 11451 | 79 |
| IR_FIX_AT_GOALS | 1 | 4007 | 11506 | 79 |
| IR_FOCUS_GOALS | 1 | 10005 | 9368 | 79 |
| IR_MDD | 1 | 10047 | 12449 | 79 |
| IR_BOTTLENECK | 1 | 10294 | 12489 | 79 |
| IR_HYBRID | 1 | 10004 | 9416 | 79 |
