#!/bin/bash
python main.py --filter-config ./configs/filter_config.json \
 --controller-config ./configs/controller_config.json \
 --tool-config-path ./configs/tool_config.json \
 --waypoints-path ./surveyed-waypoints/2025-08-15_11-20-10_waypoints.json \
 --last-row-waypoint-index 5 \
 --turn-direction left \
 --row-spacing 6.0 \
 --headland-buffer 8.0 \
