#!/bin/bash

i=0

echo > /dev/tcp/127.0.0.1/26565
if [ $? -ne 0 ]; then
  #echo 1
  ((i++))
fi

echo > /dev/tcp/127.0.0.1/26566
if [ $? -ne 0 ]; then
  ((i++))
fi

echo > /dev/tcp/127.0.0.1/26567
if [ $? -ne 0 ]; then
  ((i++))
fi

echo > /dev/tcp/127.0.0.1/26568
if [ $? -ne 0 ]; then
  ((i++))
fi
echo > /dev/tcp/127.0.0.1/26569
if [ $? -ne 0 ]; then
  ((i++))
fi

if [[ $i -eq 5 ]]; then
  echo 0
else
  echo 1
fi

