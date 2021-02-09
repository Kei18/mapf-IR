auto record by github actions
===
date: 2021-02-09-05-34-00

commit
```
commit 8f54cc8f2a8f2b61ccece848100a9b00e0d54bc0
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Tue Feb 9 14:26:50 2021 +0900

    Merge branch 'dev' of https://github.com/Kei18/mapf into dev

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 291 | 58495 | 386 |
| winPIBT | 1 | 479 | 57656 | 392 |
| PushAndSwap | 1 | 1235 | 1721545 | 7664 |
| HCA | 1 | 1740 | 54784 | 386 |
| WHCA | 1 | 1426 | 56589 | 386 |
| RevisitPP | 1 | 1401 | 54725 | 392 |
| ECBS | 1 | 9486 | 54669 | 389 |
| PIBT_COMPLETE | 1 | 332 | 58495 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 13345 | 678 | 42 |
| ICBS | 1 | 162 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10012 | 11665 | 79 |
| IR_SINGLE_PATHS | 1 | 1145 | 11750 | 79 |
| IR_FIX_AT_GOALS | 1 | 685 | 11704 | 79 |
| IR_FOCUS_GOALS | 1 | 10001 | 9402 | 79 |
| IR_MDD | 1 | 10016 | 12558 | 79 |
| IR_BOTTLENECK | 1 | 10043 | 12684 | 79 |
| IR_HYBRID | 1 | 10000 | 9385 | 79 |
