#!/bin/sh
DATE=`date +%Y%m%d`
scen_start=1
scen_limit=51
field=den520d

run () {
    OUTPUT_DIR=../../data/$DATE/$1/IR_$4
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
                         $3
            scen=`expr $scen + 1`
        done
    done
}

# ECBS
SOLVER="-s IR -x ECBS -X \"-w 1.01\" -y ICBS -n 1000000 -t 3000"
run $field "100" "$SOLVER" ECBS
SOLVER="-s IR -x ECBS -X \"-w 1.05\" -y ICBS -n 1000000 -t 3000"
run $field "300" "$SOLVER" ECBS
SOLVER="-s IR -x ECBS -X \"-w 1.10\" -y ICBS -n 1000000 -t 3000"
run $field "500" "$SOLVER" ECBS

# HCA
SOLVER="-s IR -x HCA -y ICBS -n 1000000 -t 3000"
run $field "100 300 500" "$SOLVER" HCA

# PIBT_COMPLETE
SOLVER="-s IR -x PIBT_COMPLETE -y ICBS -n 1000000 -t 3000"
run $field "100 300 500" "$SOLVER" PIBTC
