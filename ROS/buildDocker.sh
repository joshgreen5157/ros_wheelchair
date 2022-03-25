#!/bin/bash

Container_Name=$1

if [ "$Container_Name" == "" ]; then
  Container_name="autowheelchair"
fi

echo "Building docker container $Container_Name"

docker build . -t $Container_Name

if [[ $? -ne 0 ]]; then
  echo "ERROR: Failed to build docker image"
  exit 1
fi

echo "Success"
exit 0
