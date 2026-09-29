#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SRC_ROOT="${REPO_ROOT}/src"
NAV_SRC="${SRC_ROOT}/navigation"
VISION_SRC="${SRC_ROOT}/vision"
MOTION_SRC="${SRC_ROOT}/motion"
IMU_SRC="${SRC_ROOT}/sensors/imu"
TOF_SRC="${SRC_ROOT}/sensors/tof"
TELEMETRY_SRC="${SRC_ROOT}/telemetry"
DISPLAY_SRC="${SRC_ROOT}/display"
PLATFORM_SRC="${SRC_ROOT}/platform"
DRIVER_SRC="${SRC_ROOT}/drivers/loongson"
BUILD_DIR="${REPO_ROOT}/build/rewrite"
mkdir -p "${BUILD_DIR}"

MODE="${1:-selftest}"
CXX="${CXX:-g++}"
BUILD_JOBS="${BUILD_JOBS:-4}"
FORCE_REBUILD="${FORCE_REBUILD:-0}"
COMMON=(-std=c++17 -O2 -Wall -Wextra
  -I"${SRC_ROOT}/app" -I"${VISION_SRC}" -I"${NAV_SRC}"
  -I"${MOTION_SRC}" -I"${IMU_SRC}" -I"${TOF_SRC}"
  -I"${TELEMETRY_SRC}" -I"${DISPLAY_SRC}" -I"${PLATFORM_SRC}"
  -I"${PLATFORM_SRC}/safety" -I"${DRIVER_SRC}/inc")

CORE_SRC=(
  "${NAV_SRC}/path_params.cpp"
  "${VISION_SRC}/vision_pipeline.cpp"
  "${VISION_SRC}/track_topology.cpp"
  "${NAV_SRC}/path_controller.cpp"
  "${MOTION_SRC}/motion_control.cpp"
  "${MOTION_SRC}/motor_adapter.cpp"
  "${IMU_SRC}/imu_feedback.cpp"
  "${NAV_SRC}/inertial_navigation.cpp"
  "${TOF_SRC}/tof_slope_sensor.cpp"
  "${NAV_SRC}/odometry.cpp"
  "${TELEMETRY_SRC}/http_streamer.cpp"
  "${VISION_SRC}/target_recognizer.cpp"
)

if [[ "${MODE}" == "selftest" ]]; then
  OUT="${BUILD_DIR}/rewrite_path_selftest"
  "${CXX}" "${COMMON[@]}" -DPATH_FOLLOW_NO_OPENCV -DPATH_FOLLOW_NO_HW \
    "${SRC_ROOT}/app/path_selftest.cpp" "${CORE_SRC[@]}" -pthread -o "${OUT}"
  "${OUT}"
  exit 0
fi

if [[ "${MODE}" == "display-example" ]]; then
  OUT="${BUILD_DIR}/rewrite_st7735s_example"
  "${CXX}" "${COMMON[@]}" \
    "${SRC_ROOT}/app/st7735s_example.cpp" \
    "${DISPLAY_SRC}/st7735s.cpp" \
    "${DISPLAY_SRC}/spi1_shared.cpp" \
    "${DRIVER_SRC}/LQ_HW_GPIO.cpp" \
    "${DRIVER_SRC}/LQ_MAP_ADDR.cpp" \
    -pthread -o "${OUT}"
  echo "built ${OUT}"
  exit 0
fi

if [[ "${MODE}" == "motor-test" || "${MODE}" == "motor-test-sim" ]]; then
  SMARTCAR_SRC="${PLATFORM_SRC}"
  OUT="${BUILD_DIR}/rewrite_motor_stutter_test"
  DEFINES=(-DMOTOR_TEST_VARIANT=\"rewrite\")
  INCLUDES=(-I"${SMARTCAR_SRC}")
  SOURCES=(
    "${SRC_ROOT}/app/motor_stutter_test.cpp"
    "${NAV_SRC}/path_params.cpp"
    "${MOTION_SRC}/motion_control.cpp"
    "${MOTION_SRC}/motor_adapter.cpp"
    "${SMARTCAR_SRC}/hal.cpp"
    "${SMARTCAR_SRC}/success_motor.cpp"
  )
  if [[ "${MODE}" == "motor-test-sim" ]]; then
    DEFINES+=(-DSMARTCAR_SIM)
  else
    INCLUDES+=(-I"${DRIVER_SRC}/inc")
    SOURCES+=(
      "${DRIVER_SRC}/LQ_HW_GPIO.cpp"
      "${DRIVER_SRC}/LQ_MAP_ADDR.cpp"
    )
  fi
  "${CXX}" "${COMMON[@]}" "${DEFINES[@]}" "${INCLUDES[@]}" \
    "${SOURCES[@]}" -pthread -lm -o "${OUT}"
  echo "built ${OUT}"
  exit 0
fi

if [[ "${MODE}" == "target" ]]; then
  OPENCV_DIR="${OPENCV_DIR:-}"
  NCNN_DIR="${NCNN_DIR:-}"
  SMARTCAR_SRC="${PLATFORM_SRC}"
  SENSOR_SRC="${SRC_ROOT}/sensors"
  OUT="${BUILD_DIR}/lq_path_follow_rewrite"
  OBJ_DIR="${BUILD_DIR}/obj-target"
  INCLUDES=(-I"${SRC_ROOT}/app" -I"${VISION_SRC}" -I"${NAV_SRC}"
    -I"${MOTION_SRC}" -I"${IMU_SRC}" -I"${TOF_SRC}"
    -I"${TELEMETRY_SRC}" -I"${DISPLAY_SRC}" -I"${SMARTCAR_SRC}"
    -I"${SMARTCAR_SRC}/safety" -I"${DRIVER_SRC}/inc")
  LIBS=(-fopenmp -lgomp -pthread -ldl -lm)
  DEFINES=()
  if [[ -n "${OPENCV_DIR}" ]]; then
    INCLUDES+=(-I"${OPENCV_DIR}/include/opencv4")
    LIBS+=(-L"${OPENCV_DIR}/lib" -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs)
  elif pkg-config --exists opencv4; then
    # shellcheck disable=SC2207
    INCLUDES+=($(pkg-config --cflags opencv4))
    # shellcheck disable=SC2207
    LIBS+=($(pkg-config --libs opencv4))
  else
    echo "OpenCV not found. Set OPENCV_DIR=/path/to/opencv_install." >&2
    exit 2
  fi
  if [[ -n "${NCNN_DIR}" ]]; then
    DEFINES+=(-DREWRITE_WITH_NCNN)
    INCLUDES+=(-I"${NCNN_DIR}/include")
    LIBS+=(-L"${NCNN_DIR}/lib" -lncnn)
  fi
  TARGET_SRC=(
    "${SRC_ROOT}/app/lq_path_follow.cpp"
    "${TELEMETRY_SRC}/status_display.cpp"
    "${DISPLAY_SRC}/st7735s.cpp"
    "${DISPLAY_SRC}/spi1_shared.cpp"
    "${IMU_SRC}/lsm6dsr_spi1.cpp"
    "${CORE_SRC[@]}"
    "${IMU_SRC}/lq_lsm6dsr.cpp"
    "${TOF_SRC}/lq_vl53l0x.cpp"
    "${SMARTCAR_SRC}/hal.cpp"
    "${SMARTCAR_SRC}/success_motor.cpp"
    "${DRIVER_SRC}/LQ_ATIM_PWM.cpp"
    "${DRIVER_SRC}/LQ_HW_ADC.cpp"
    "${DRIVER_SRC}/LQ_HW_GPIO.cpp"
    "${DRIVER_SRC}/LQ_MAP_ADDR.cpp"
  )
  mkdir -p "${OBJ_DIR}"
  BUILD_JOBS="$((BUILD_JOBS < 1 ? 1 : BUILD_JOBS))"

  FLAG_HASH="$(
    {
      "${CXX}" --version | head -n 1
      printf '%q ' "${COMMON[@]}" -fopenmp "${DEFINES[@]}" "${INCLUDES[@]}"
    } | sha256sum | awk '{print $1}'
  )"
  FLAG_FILE="${OBJ_DIR}/flags.sha256"
  if [[ "${FORCE_REBUILD}" == "1" ||
        ! -f "${FLAG_FILE}" ||
        "$(cat "${FLAG_FILE}")" != "${FLAG_HASH}" ]]; then
    find "${OBJ_DIR}" -maxdepth 1 -type f \
      \( -name '*.o' -o -name '*.d' \) -delete
    printf '%s\n' "${FLAG_HASH}" > "${FLAG_FILE}"
  fi

  HEADER_DIRS=("${SRC_ROOT}" "${SMARTCAR_SRC}" "${DRIVER_SRC}/inc")
  OBJECTS=()
  PIDS=()
  DESCRIPTIONS=()

  wait_first_compile() {
    local pid="${PIDS[0]}"
    local description="${DESCRIPTIONS[0]}"
    if ! wait "${pid}"; then
      echo "Compilation failed: ${description}" >&2
      exit 1
    fi
    PIDS=("${PIDS[@]:1}")
    DESCRIPTIONS=("${DESCRIPTIONS[@]:1}")
  }

  for source in "${TARGET_SRC[@]}"; do
    relative="${source#${REPO_ROOT}/}"
    object_name="${relative//\//__}"
    object="${OBJ_DIR}/${object_name%.cpp}.o"
    dependency="${object%.o}.d"
    OBJECTS+=("${object}")
    needs_compile=0
    if [[ ! -f "${object}" || "${source}" -nt "${object}" ]]; then
      needs_compile=1
    elif find "${HEADER_DIRS[@]}" -type f \
      \( -name '*.h' -o -name '*.hpp' \) -newer "${object}" \
      -print -quit | grep -q .; then
      needs_compile=1
    fi
    if [[ "${needs_compile}" == "1" ]]; then
      echo "    CXX ${relative}"
      "${CXX}" "${COMMON[@]}" -fopenmp "${DEFINES[@]}" "${INCLUDES[@]}" \
        -MMD -MP -MF "${dependency}" -c "${source}" -o "${object}" &
      PIDS+=("$!")
      DESCRIPTIONS+=("${relative}")
      if ((${#PIDS[@]} >= BUILD_JOBS)); then
        wait_first_compile
      fi
    fi
  done
  while ((${#PIDS[@]})); do
    wait_first_compile
  done

  needs_link=0
  if [[ ! -f "${OUT}" ]]; then
    needs_link=1
  else
    for object in "${OBJECTS[@]}"; do
      if [[ "${object}" -nt "${OUT}" ]]; then
        needs_link=1
        break
      fi
    done
  fi
  if [[ "${needs_link}" == "1" ]]; then
    echo "    LINK ${OUT#${REPO_ROOT}/}"
    "${CXX}" -fopenmp "${OBJECTS[@]}" "${LIBS[@]}" -o "${OUT}"
    echo "built ${OUT}"
  else
    echo "reused unchanged ${OUT}"
  fi
  exit 0
fi

if [[ "${MODE}" == "replay" ]]; then
  OPENCV_DIR="${OPENCV_DIR:-}"
  # 修复：本分支从未给 SMARTCAR_SRC 赋值（set -u 下展开即挂），
  # 且 LIBS 缺 -ldl -lm（OpenCV 需要 dlopen/dlsym@GLIBC_2.27）。与 .ps1 版对齐。
  SMARTCAR_SRC="${PLATFORM_SRC}"
  INCLUDES=(-I"${SRC_ROOT}/app" -I"${VISION_SRC}" -I"${NAV_SRC}"
    -I"${MOTION_SRC}" -I"${IMU_SRC}" -I"${TOF_SRC}"
    -I"${TELEMETRY_SRC}" -I"${DISPLAY_SRC}" -I"${SMARTCAR_SRC}"
    -I"${SMARTCAR_SRC}/safety" -I"${DRIVER_SRC}/inc")
  LIBS=(-pthread -ldl -lm)
  if [[ -n "${OPENCV_DIR}" ]]; then
    INCLUDES+=(-I"${OPENCV_DIR}/include/opencv4")
    LIBS+=(
      -L"${OPENCV_DIR}/lib"
      -Wl,-rpath-link,"${OPENCV_DIR}/lib"
      -lopencv_core -lopencv_imgproc -lopencv_videoio -lopencv_imgcodecs
    )
  elif pkg-config --exists opencv4; then
    # shellcheck disable=SC2207
    INCLUDES+=($(pkg-config --cflags opencv4))
    # shellcheck disable=SC2207
    LIBS+=($(pkg-config --libs opencv4))
  else
    echo "OpenCV not found. Set OPENCV_DIR=/path/to/opencv_install." >&2
    exit 2
  fi
  OUT="${BUILD_DIR}/rewrite_video_replay"
  "${CXX}" "${COMMON[@]}" "${INCLUDES[@]}" \
    "${SRC_ROOT}/app/video_replay.cpp" \
    "${NAV_SRC}/path_params.cpp" \
    "${VISION_SRC}/vision_pipeline.cpp" \
    "${NAV_SRC}/path_controller.cpp" \
    "${LIBS[@]}" -o "${OUT}"
  echo "built ${OUT}"
  exit 0
fi

echo "Usage: $0 [selftest|target|replay|display-example|motor-test|motor-test-sim]" >&2
exit 2
