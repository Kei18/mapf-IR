#!/bin/bash
source `dirname $0`/util.sh

start_date=`getDate`

scen_start=1
scen_end=25
force=0

map="random-64-64-20.map"
agents_list="300"
well_formed=1

solver="IR_HYBRID"
options=(
    "-x HCA"
    "-x WHCA -X \"-w 30\""
    "-x RevisitPP"
    "-x ECBS -X \"-w 1.05\""
    "-x PIBT_COMPLETE"
)

refine_limit=500
refine_cnt=10000000
comp_time_limit=90000

for option in "${options[@]}"
do
    bash `dirname $0`/run.sh \
       $map \
       "$agents_list" \
       "$solver ${option} -n ${refine_cnt} -t ${refine_limit} -T ${comp_time_limit}" \
       $scen_start \
       $scen_end \
       $well_formed \
       $force
done


# send message
str_solvers="options="
for option in "${options[@]}"
do
    str_solvers=$str_solvers"\n"$option
done
str_solvers=${str_solvers//\"/\\\"}

MESSAGE="*-----------------------------------
fin experiment\ndate=${start_date}\nmap=${map}\nagents=${agents_list}\nwell_formed=${well_formed}\n${str_solvers}
-----------------------------------*"
bash `dirname $0`/slack_notification.sh "$MESSAGE"
