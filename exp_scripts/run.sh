#!/bin/sh

## args
map=$1
agents_list=$2
solver=$3
scen_start=$4
scen_end=$5
well_formed=$6
force=$7

PROJECT_DIR=`dirname $0`/..
map_trimed=${map%.map}

## check git status
if [ ${force} -ne 1 ]
then
   GIT_STATUS_RESULT=`git status -s`
   if [ ${#GIT_STATUS_RESULT} -ne 0 ]
   then
       echo "Untracked changes exist. Commit them beforehand."
       git status -s
       exit 1
   fi
fi
GIT_RECENT_COMMIT=`git log -1 --pretty=format:"%H"`

## print experimental info
echo "-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*"
echo "start experiment"
echo "- map: ${map}"
echo "- agents: ${agents_list}"
echo "- solver: ${solver}"
echo "- scen: from ${scen_start} to ${scen_end}"

## create output directory
EXP_DATE=`gdate +%Y-%m-%d_%H-%M-%S-%2N`
OUTPUT_DIR=$PROJECT_DIR/../data/$EXP_DATE/
mkdir -p $OUTPUT_DIR

## build
(cd $PROJECT_DIR/build;
 make > /dev/null 2>&1;)

## main
for agent_num in ${agents_list[@]}
do
    echo "\nagents=${agent_num}"
    scen=$scen_start
    while [ $scen -lt `expr $scen_end + 1` ]
    do
        scen_file="${map_trimed}_${agent_num}agents_${scen}.txt"
        if [ ${well_formed} -eq 1 ]
        then
            scen_file="${map_trimed}_${agent_num}agents_well-formed_${scen}.txt"
        fi
        $PROJECT_DIR/build/app \
                    -i $PROJECT_DIR/instances/$scen_file \
                    -o $OUTPUT_DIR/$scen_file \
                    -s $solver
        scen=`expr $scen + 1`
    done
done

## create status file
STATUS_FILE=$OUTPUT_DIR/status.txt
{
    echo start:$EXP_DATE
    echo end:`date +%Y-%m-%d-%H-%M-%S`
    echo used-commit:$GIT_RECENT_COMMIT
    echo map:$map
    echo agents:$agents_list
    echo solver:$solver
    echo scen_start:$scen_start
    echo scen_end:$scen_end
} > $STATUS_FILE

echo "\nfinish experiment"
echo "result -> ${OUTPUT_DIR}"
echo "*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-"
