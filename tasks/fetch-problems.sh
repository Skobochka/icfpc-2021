#!/bin/bash

for i in `seq 1 59`
do
    wget -c https://poses.live/problems/$i/download
done
