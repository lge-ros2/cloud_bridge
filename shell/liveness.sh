#!/bin/bash

num=$(netstat -anlp | grep cloud_bridge | grep ESTABLISHED | wc -l)
if [[ $num -ne 5 ]]; then
  echo 1
else
  echo 0
fi  
