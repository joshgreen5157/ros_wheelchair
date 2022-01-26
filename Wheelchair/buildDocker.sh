#!/bin/bash

Container_Name=$1

if [ "$Container_Name" == "" ]; then
  Container_name="4800_Wheelchair"
fi

echo "Building docker container $Container_Name"

docker build . -t $Container_name

if [[ $? -ne 0 ]]; then
  echo "ERROR: Failed to build docker image"
  exit 1
fi

echo "Success"
exit 0
