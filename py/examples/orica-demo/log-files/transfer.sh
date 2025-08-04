#!/bin/bash

set -e

usage() {
    echo "Usage: $0 -f <filename> -r <robot-name>"
    exit 1
}

# Parse arguments
while getopts "f:r:" opt; do
    case "$opt" in
        f) FILENAME="$OPTARG" ;;
        r) ROBOT="$OPTARG" ;;
        *) usage ;;
    esac
done

if [ -z "$FILENAME" ] || [ -z "$ROBOT" ]; then
    usage
fi

REMOTE_PATH="/mnt/data/$FILENAME"
LOCAL_PATH="./$FILENAME"
REMOTE_PARAMS_PATH="/mnt/service_config/track_follower.json"

echo "==> SCP: $REMOTE_PATH from $ROBOT..."
scp "adminfarmng@${ROBOT}:${REMOTE_PATH}" .

echo "==> Activating Python virtual environment..."
source ~/Documents/farm-ng/workspace/amiga/venv/bin/activate

echo "==> Converting BIN to MCAP..."
python ~/Documents/farm-ng/workspace/amiga/services/foxglove/mcap_conversor.py --events-file "$LOCAL_PATH"

# After conversion, the .bin file is renamed to .mcap
MCAP_FILE="${FILENAME%.bin}.mcap"

if [ ! -f "$MCAP_FILE" ]; then
    echo "❌ Conversion failed: $MCAP_FILE not found."
    exit 1
fi

echo "==> Injecting RPM into MCAP..."
python ~/Documents/farm-ng/workspace/farm_ng_apps/py/log_parsing/mcap_rpm_injector.py "$MCAP_FILE"

echo "✅ Done! Final MCAP: $MCAP_FILE"

# === SCP: TRACK FOLLOWER PARAMETERS ===
echo "==> SCP: $REMOTE_PARAMS_PATH from $ROBOT..."
scp "adminfarmng@${ROBOT}:${REMOTE_PARAMS_PATH}" .

# Rename the config file
PARAMS_FILE="${FILENAME%.*}_params.json"
mv "track_follower.json" "$PARAMS_FILE"

echo "✅ Params file saved as $PARAMS_FILE"
