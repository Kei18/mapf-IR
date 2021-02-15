auto record by github actions
===
date: 2021-02-15-07-48-49

commit
```
commit dcfcf47569f6db76d92586836a940715d3a20bf9
Author: Kei18 <keisuke.oku18@gmail.com>
Date:   Mon Feb 15 16:45:48 2021 +0900

    add several comments

```

## sub-optimal solvers
benchmark: ./benchrmark/suboptimal_den520d_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| PIBT | 1 | 282 | 58574 | 386 |
| winPIBT | 1 | 472 | 58117 | 388 |
| PushAndSwap | 1 | 389 | 10965159 | 54639 |
| HCA | 1 | 981 | 54784 | 386 |
| WHCA | 1 | 656 | 56588 | 386 |
| RevisitPP | 1 | 961 | 54725 | 392 |
| ECBS | 1 | 11369 | 54666 | 389 |
| PIBT_COMPLETE | 1 | 332 | 58574 | 386 |

## optimal solvers
benchmark: ./benchrmark/optimal_random-32-32-20_30agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| CBS | 1 | 23997 | 678 | 42 |
| ICBS | 1 | 79 | 678 | 42 |

## anytime solvers
benchmark: ./benchrmark/anytime_arena_300agents.txt

|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| ---: | ---: | ---: | ---: | ---: |
| IR | 1 | 10006 | 10805 | 79 |
| IR_SINGLE_PATHS | 1 | 717 | 11726 | 79 |
| IR_FIX_AT_GOALS | 1 | 263 | 11359 | 79 |
| IR_FOCUS_GOALS | 1 | 10003 | 9387 | 79 |
| IR_MDD | 1 | 10032 | 12166 | 79 |
| IR_BOTTLENECK | 1 | 10036 | 12426 | 79 |
| IR_HYBRID | 1 | 10018 | 9389 | 79 |
