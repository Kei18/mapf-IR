#!/bin/bash
source `dirname $0`/util.sh

start_date=`getDate`

scen_start=1
scen_end=10
force=0

map="ost000a.map"
agents_list="3000"
well_formed=0

solver="IR_HYBRID -v"

refine_limit=10000
refine_cnt=10000000
comp_time_limit=3600000


bash `dirname $0`/run.sh \
     $map \
     "$agents_list" \
     "$solver ${option} -n ${refine_cnt} -t ${refine_limit} -T ${comp_time_limit}" \
     $scen_start \
     $scen_end \
     $well_formed \
     $force


str_solvers="options=${solver}"

MESSAGE="*-----------------------------------
fin experiment\ndate=${start_date}\nmap=${map}\nagents=${agents_list}\nwell_formed=${well_formed}\n${str_solvers}
-----------------------------------*"
bash `dirname $0`/slack_notification.sh "$MESSAGE"
