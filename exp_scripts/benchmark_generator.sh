#!/bin/sh
BENCHMARK_DIR=../instances/
PARAM_FILE=tmp.txt

mkdir -p $BENCHMARK_DIR

if [ $# != 6 ]
then
    echo "sh ./benchmark_generator.sh {map} {agent_num} {instance_num} {well_formed} {max_timestep} {max_comp_time}"
    exit 1
fi

map=$1
map_trimed=${map%.map}
agent_num=$2
instance_num=$3
well_formed=$4
max_timestep=$5
max_comp_time=$6

seed=0
while [ $seed -lt ${instance_num} ]
do
    touch $PARAM_FILE
    {
        echo map_file=$map
        echo agents=$agent_num
        echo seed=$seed
        echo random_problem=1
        echo well_formed=$well_formed
        echo max_timestep=$max_timestep
        echo max_comp_time=$max_comp_time
    } > $PARAM_FILE
    filename="${map_trimed}_${agent_num}agents_`expr $seed + 1`.txt"
    if [ ${well_formed} -eq 1 ]
    then
        filename="${map_trimed}_${agent_num}agents_well-formed_`expr $seed + 1`.txt"
    fi
    ../build/app -P -i $PARAM_FILE -o $BENCHMARK_DIR/$filename
    echo create instance $BENCHMARK_DIR$filename
    seed=`expr $seed + 1`
done

rm $PARAM_FILE
