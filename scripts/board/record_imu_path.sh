#!/bin/sh
set -eu

OUTPUT="${1:-/home/root/imu_path_$(date +%Y%m%d_%H%M%S).csv}"
if [ "$#" -gt 0 ]; then
    shift
fi

if pidof lq_path_follow_imu >/dev/null 2>&1 ||
   pidof lq_path_follow_coco_rewrite >/dev/null 2>&1 ||
   pidof lq_path_follow_rewrite >/dev/null 2>&1; then
    echo "a path-follow process is already running; stop it first" >&2
    exit 1
fi

echo "Keep the vehicle still until [IMU] ready, then move it."
echo "Press Ctrl+C to finish and flush: ${OUTPUT}"

cd /home/root
/home/root/setup_st7735s_spi.sh
exec env \
    LD_LIBRARY_PATH=/home/root/LQ_Dep_libs/opencv-lib:/home/root/LQ_Dep_libs/ncnn-lib \
    ./lq_path_follow_imu \
    --camera /dev/video0 \
    --imu-transport spi \
    --dry-run \
    --no-ncnn \
    --no-stop \
    --imu-csv "${OUTPUT}" \
    --imu-csv-hz 50 \
    "$@"
