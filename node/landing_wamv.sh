#!/bin/bash

typhoon_h480_num=3
vehicle_num=0
while(( $vehicle_num< typhoon_h480_num)) 
do
    python landing_wamv.py  $vehicle_num&
    let "vehicle_num++"
done