#!/bin/bash
python main.py --filter-config ./configs/filter_config.json \
 --controller-config ./configs/controller_config.json \
 --waypoints-path ./surveyed-waypoints/2025-08-04_11-26-01_waypoints_shifted.json \
 --last-row-waypoint-index 4 \
 --turn-direction left \
 --row-spacing 5.9 \
 --headland-buffer 6.0 \
 --delay 5.0 \
