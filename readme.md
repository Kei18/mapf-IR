auto record by github actions
===
date: 2021-01-25-10-45-59

commit
```
commit bfe9800cec7226b7941832b2dce1d9c701839090
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Mon Jan 25 19:40:45 2021 +0900

    update HYBRID

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 272 | 58791 | 386 |
| HCA | 1 | 4308 | 54784 | 386 |
| WHCA | 1 | 2427 | 56589 | 386 |
| RevisitPP | 1 | 4529 | 54725 | 392 |
| ECBS | 1 | 17371 | 54669 | 389 |
| PIBT_COMPLETE | 1 | 397 | 58791 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 26035 | 678 | 42 |
| ICBS | 1 | 81 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10025 | 11100 | 79 |
| IR_SINGLE_PATHS | 1 | 2333 | 11451 | 79 |
| IR_FIX_AT_GOALS | 1 | 4294 | 11506 | 79 |
| IR_FOCUS_GOALS | 1 | 10005 | 9385 | 79 |
| IR_MDD | 1 | 10028 | 12469 | 79 |
| IR_BOTTLENECK | 1 | 10020 | 12489 | 79 |
| IR_HYBRID | 1 | 10003 | 9420 | 79 |
