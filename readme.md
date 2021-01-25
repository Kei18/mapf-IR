auto record by github actions
===
date: 2021-01-25-10-09-43

commit
```
commit acfcb70465308bcbb61a7f770c879c9d685f8cca
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Mon Jan 25 19:06:32 2021 +0900

    update ECBS

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 280 | 58791 | 386 |
| HCA | 1 | 3955 | 54784 | 386 |
| WHCA | 1 | 2258 | 56589 | 386 |
| RevisitPP | 1 | 4169 | 54725 | 392 |
| ECBS | 1 | 14846 | 54669 | 389 |
| PIBT_COMPLETE | 1 | 382 | 58791 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 23487 | 678 | 42 |
| ICBS | 1 | 72 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10005 | 10713 | 79 |
| IR_SINGLE_PATHS | 1 | 1897 | 11451 | 79 |
| IR_FIX_AT_GOALS | 1 | 3831 | 11506 | 79 |
| IR_FOCUS_GOALS | 1 | 10003 | 9377 | 79 |
| IR_MDD | 1 | 10020 | 12449 | 79 |
| IR_BOTTLENECK | 1 | 10015 | 12489 | 79 |
| IR_HYBRID | 1 | 10004 | 9421 | 79 |
