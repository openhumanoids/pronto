#!/bin/bash
# se-batch-process-valkyrie.sh
# Author: Maurice Fallon
# Batch Processes a set of State Estimation Logs

date_str=$(date +"%Y-%m-%d-%H-%M")
echo 'Output to directory '$date_str

path=$HOME
#path+="/pronto_test_data/valkyrie/2016-03-04-raluca-turning/"
path+="/logs/valkyrie"
path+='/20160503-vicon-walking/'
path_out=$HOME
path_out+='/Desktop/results-pronto-vicon'

unset files
files=(
        #'raluca-turning-180deg-snippet.lcmlog'
        'lcmlog__2016-05-03_vicon-walking-5'
        'lcmlog__2016-05-03_vicon-walking-4'
        'lcmlog__2016-05-03_vicon-walking-3.snippet'
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
  log_out_fusion_mat-vicon=$log_out_fusion"-vicon.mat"

  verbose=0
  if [ $verbose -eq 1 ]
    then
    echo $log_in
    echo $path_out
    echo $log_out
    echo $log_out_fusion
  fi

  # Run estimation and then convert to matlab file (.mat)
  se-fusion      -P val/robot.cfg -U /val_description/urdf/valkyrie_sim.urdf  -L $log_in -pr 0    -l $log_out_fusion

  bot-log2mat  $log_out_fusion  -c "POSE_BODY|POSE_VICON|POSE_BDI|POSE_BODY_ALT" -o $log_out_fusion_mat
  #bot-log2mat  $log_in -c "VICON_pelvis_val" -o $log_out_fusion_mat-vicon

  # Only convert the log:
  #bot-log2mat  $log_in  -c "POSE_BDI|POSE_BODY" -o $log_out_fusion_mat

  echo $log_out_fusion_mat
}

tLen=${#files[@]}


mkdir $path_out -p

for (( i=0; i<${tLen}; i++ ));
do
  echo 'Processing: '$i' of '$tLen': '${files[i]}
  process_log $path $path_out $date_str ${files[i]}
done
