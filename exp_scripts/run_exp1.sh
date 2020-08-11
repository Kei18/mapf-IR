#!/bin/sh
SOLVER="-s IR -x PIBT_COMPLETE -y ICBS -n 100"
DATE=`date +%Y%m%d`
scen_start=1
scen_limit=51

run () {
    OUTPUT_DIR=../../data/$DATE/$1/IR_PIBTC
    agents_list=$2
    mkdir -p $OUTPUT_DIR
    for agents in ${agents_list[@]}
    do
        scen=$scen_start
        while [ $scen -lt $scen_limit ]
        do
            scen_file=$1_${agents}agents_${scen}.txt
            ../build/app -i ../instances/$scen_file \
                  -o $OUTPUT_DIR/$scen_file \
                  $SOLVER
            scen=`expr $scen + 1`
        done
    done
}

run random-32-32-20 "30"
run random-32-32-10 "50"
