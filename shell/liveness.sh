#!/bin/bash

num=$(netstat -anlp | grep cloud_bridge | grep ESTABLISHED | wc -l)
if [[ $num -eq 5 ]]; then
  echo 0
else
  echo $num
fi  
