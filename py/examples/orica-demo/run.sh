#!/bin/bash
python main.py --filter-config filter_config.json \
 --controller-config controller_config.json \
 --waypoints-path 2025-08-01_10-50-54_waypoints.json \
 --last-row-waypoint-index 4 \
 --turn-direction left \
 --row-spacing 6 \
 --headland-buffer 4 \
 --vis
