#!/bin/bash
# se-batch-process-valkyrie.sh
# Author: Maurice Fallon
# Batch Processes a set of State Estimation Logs

date_str=$(date +"%Y-%m-%d-%H-%M")
echo 'zOutput to directory '$date_str

path=$HOME
path+="/logs/valkyrie/lcm_logs/"
path_out=$HOME
path_out+='/Desktop/results/'

unset files
files[0]='test' 
files=(
       'Valkyrie_100QuickSteps_NoArms-raw'
      )





process_log(){
  path=$1
  path_out=$2
  date_str=$3
  file=$4

  log_in=$path$file
  log_out=$path_out"/"$file"-result"
  log_out_fusion=$log_out"-se-fusion"
  log_out_fusion_mat=$log_out_fusion".mat"

  verbose=0
  if [ $verbose -eq 1 ]
    then
    echo $log_in
    echo $path_out
    echo $log_out
    echo $log_out_fusion
  fi

  # Run estimation and then convert to matlab file (.mat)
  #se-fusion       -U model_LN_RN.urdf -P val_cfg_FIXMI/robot.cfg -L $log_in -pr 0    -l $log_out_fusion
  #bot-log2mat  $log_out_fusion  -c "POSE_BDI|POSE_VICON|POSE_BODY_ALT|POSE_BODY|POSE_MIT" -o $log_out_fusion_mat

  # Only convert the log:
  bot-log2mat  $log_in  -c "POSE_BDI|POSE_VICON|POSE_BODY_ALT|POSE_BODY|POSE_MIT" -o $log_out_fusion_mat

  echo $log_out_fusion_mat
}

tLen=${#files[@]}


mkdir $path_out -p

for (( i=0; i<${tLen}; i++ ));
do
  echo 'Processing: '$i' of '$tLen': '${files[i]}
  process_log $path $path_out $date_str ${files[i]}
done
