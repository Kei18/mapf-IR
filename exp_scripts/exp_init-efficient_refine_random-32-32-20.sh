#!/bin/bash

set -e

scen_start=1
scen_end=25
force=0

map="random-32-32-20.map"
agents_list="110"
well_formed=0

solvers=(
    "IR_SINGLE_PATHS"
    "IR_FIX_AT_GOALS"
    "IR -S 10"
    "IR -S 30"
    "IR_FOCUS_GOALS"
    "IR_MDD"
    "IR_BOTTLENECK"
    "IR_HYBRID")

refine_limit=500
refine_cnt=10000000
comp_time_limit=90000
refine_option="-x ECBS -X \"-w 1.2\""

for solver in "${solvers[@]}"
do
    bash `dirname $0`/run.sh \
       $map \
       "$agents_list" \
       "$solver ${refine_option} -n ${refine_cnt} -t ${refine_limit} -T ${comp_time_limit}" \
       $scen_start \
       $scen_end \
       $well_formed \
       $force
done


# send message
str_solvers="solvers="
for solver in "${solvers[@]}"
do
    str_solvers=$str_solvers"\n-"$solver
done

MESSAGE="*-----------------------------------
fin experiment\nmap=${map}\nagents=${agents_list}\nwell_formed=${well_formed}\n${str_solvers}
-----------------------------------*"
bash `dirname $0`/slack_notification.sh "$MESSAGE"
