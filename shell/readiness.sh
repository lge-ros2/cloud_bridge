#!/bin/bash

i=0

echo > /dev/tcp/127.0.0.1/26565
if [ $? -eq 0 ]; then
  ((i++))
fi

echo > /dev/tcp/127.0.0.1/26566
if [ $? -eq 0 ]; then
  ((i++))
fi

echo > /dev/tcp/127.0.0.1/26567
if [ $? -eq 0 ]; then
  ((i++))
fi

echo > /dev/tcp/127.0.0.1/26568
if [ $? -eq 0 ]; then
  ((i++))
fi
echo > /dev/tcp/127.0.0.1/26569
if [ $? -eq 0 ]; then
  ((i++))
fi

if [[ $i -eq 5 ]]; then
  echo 0
else
  echo "$i and 1"
  echo 1
fi

