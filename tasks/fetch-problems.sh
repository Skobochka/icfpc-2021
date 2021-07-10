#!/bin/bash

API_TOKEN=29a3adf2-b0d3-4166-8891-9c990df11546

for i in `seq 1 88`
do
    curl -H "Authorization: Bearer $API_TOKEN" https://poses.live/api/problems/$i -o $i.problem
done
