auto record by github actions
===
date: 2021-01-27-11-51-33

commit
```
commit b2ca10ebee619fb2ffcc4797d16ed312914662b8
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Wed Jan 27 20:47:52 2021 +0900

    modify script

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 299 | 58791 | 386 |
| HCA | 1 | 4104 | 54784 | 386 |
| WHCA | 1 | 2321 | 56589 | 386 |
| RevisitPP | 1 | 4515 | 54725 | 392 |
| ECBS | 1 | 15664 | 54669 | 389 |
| PIBT_COMPLETE | 1 | 406 | 58791 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 25633 | 678 | 42 |
| ICBS | 1 | 78 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10011 | 10946 | 79 |
| IR_SINGLE_PATHS | 1 | 2208 | 11451 | 79 |
| IR_FIX_AT_GOALS | 1 | 4196 | 11506 | 79 |
| IR_FOCUS_GOALS | 1 | 10005 | 9379 | 79 |
| IR_MDD | 1 | 10010 | 12449 | 79 |
| IR_BOTTLENECK | 1 | 10004 | 12489 | 79 |
| IR_HYBRID | 1 | 10003 | 9419 | 79 |
