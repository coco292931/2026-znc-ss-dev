#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${REPO_ROOT}/build/rewrite"
mkdir -p "${BUILD_DIR}"

MODE="${1:-selftest}"
CXX="${CXX:-g++}"
BUILD_JOBS="${BUILD_JOBS:-4}"
FORCE_REBUILD="${FORCE_REBUILD:-0}"
COMMON=(-std=c++17 -O2 -Wall -Wextra -I"${SCRIPT_DIR}")

CORE_SRC=(
  "${SCRIPT_DIR}/path_params.cpp"
  "${SCRIPT_DIR}/vision_pipeline.cpp"
  "${SCRIPT_DIR}/path_controller.cpp"
  "${SCRIPT_DIR}/motion_control.cpp"
  "${SCRIPT_DIR}/motor_adapter.cpp"
  "${SCRIPT_DIR}/imu_feedback.cpp"
  "${SCRIPT_DIR}/inertial_navigation.cpp"
  "${SCRIPT_DIR}/tof_slope_sensor.cpp"
  "${SCRIPT_DIR}/odometry.cpp"
  "${SCRIPT_DIR}/http_streamer.cpp"
  "${SCRIPT_DIR}/target_recognizer.cpp"
)

if [[ "${MODE}" == "selftest" ]]; then
  OUT="${BUILD_DIR}/rewrite_path_selftest"
  "${CXX}" "${COMMON[@]}" -DPATH_FOLLOW_NO_OPENCV -DPATH_FOLLOW_NO_HW \
    "${SCRIPT_DIR}/path_selftest.cpp" "${CORE_SRC[@]}" -pthread -o "${OUT}"
  "${OUT}"
  exit 0
fi

if [[ "${MODE}" == "display-example" ]]; then
  LQ_DEMO="${LQ_DEMO:-${REPO_ROOT}/Loongson_2k301_LIB-master/Loongson_2k301_LIB-master/LQ_ls2k301_Demo}"
  OUT="${BUILD_DIR}/rewrite_st7735s_example"
  "${CXX}" "${COMMON[@]}" -I"${LQ_DEMO}/Libraries/Driver/inc" \
    "${SCRIPT_DIR}/st7735s_example.cpp" \
    "${SCRIPT_DIR}/st7735s.cpp" \
    "${SCRIPT_DIR}/spi1_shared.cpp" \
    "${LQ_DEMO}/Libraries/Driver/LQ_HW_GPIO.cpp" \
    "${LQ_DEMO}/Libraries/Driver/LQ_MAP_ADDR.cpp" \
    -pthread -o "${OUT}"
  echo "built ${OUT}"
  exit 0
fi

if [[ "${MODE}" == "motor-test" || "${MODE}" == "motor-test-sim" ]]; then
  LQ_DEMO="${LQ_DEMO:-${REPO_ROOT}/Loongson_2k301_LIB-master/Loongson_2k301_LIB-master/LQ_ls2k301_Demo}"
  SMARTCAR_SRC="${REPO_ROOT}/SmartCar/src"
  OUT="${BUILD_DIR}/rewrite_motor_stutter_test"
  DEFINES=(-DMOTOR_TEST_VARIANT=\"rewrite\")
  INCLUDES=(-I"${SCRIPT_DIR}" -I"${SMARTCAR_SRC}")
  SOURCES=(
    "${REPO_ROOT}/tools/motor_stutter_test.cpp"
    "${SCRIPT_DIR}/path_params.cpp"
    "${SCRIPT_DIR}/motion_control.cpp"
    "${SCRIPT_DIR}/motor_adapter.cpp"
    "${SMARTCAR_SRC}/hal.cpp"
    "${SMARTCAR_SRC}/success_motor.cpp"
  )
  if [[ "${MODE}" == "motor-test-sim" ]]; then
    DEFINES+=(-DSMARTCAR_SIM)
  else
    INCLUDES+=(-I"${LQ_DEMO}/Libraries/Driver/inc")
    SOURCES+=(
      "${LQ_DEMO}/Libraries/Driver/LQ_HW_GPIO.cpp"
      "${LQ_DEMO}/Libraries/Driver/LQ_MAP_ADDR.cpp"
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
  LQ_DEMO="${LQ_DEMO:-${REPO_ROOT}/Loongson_2k301_LIB-master/Loongson_2k301_LIB-master/LQ_ls2k301_Demo}"
  SMARTCAR_SRC="${REPO_ROOT}/SmartCar/src"
  SUCCESS_NEW2="${REPO_ROOT}/success-new2"
  OUT="${BUILD_DIR}/lq_path_follow_rewrite"
  OBJ_DIR="${BUILD_DIR}/obj-target"
  INCLUDES=(-I"${SCRIPT_DIR}" -I"${SMARTCAR_SRC}" -I"${SUCCESS_NEW2}" -I"${LQ_DEMO}/Libraries/Driver/inc")
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
    "${SCRIPT_DIR}/lq_path_follow.cpp"
    "${SCRIPT_DIR}/status_display.cpp"
    "${SCRIPT_DIR}/st7735s.cpp"
    "${SCRIPT_DIR}/spi1_shared.cpp"
    "${SCRIPT_DIR}/lsm6dsr_spi1.cpp"
    "${CORE_SRC[@]}"
    "${SUCCESS_NEW2}/lq_lsm6dsr.cpp"
    "${SUCCESS_NEW2}/lq_vl53l0x.cpp"
    "${SMARTCAR_SRC}/hal.cpp"
    "${SMARTCAR_SRC}/success_motor.cpp"
    "${LQ_DEMO}/Libraries/Driver/LQ_ATIM_PWM.cpp"
    "${LQ_DEMO}/Libraries/Driver/LQ_HW_ADC.cpp"
    "${LQ_DEMO}/Libraries/Driver/LQ_HW_GPIO.cpp"
    "${LQ_DEMO}/Libraries/Driver/LQ_MAP_ADDR.cpp"
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

  HEADER_DIRS=("${SCRIPT_DIR}" "${SMARTCAR_SRC}" "${SUCCESS_NEW2}" "${LQ_DEMO}/Libraries/Driver/inc")
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
  INCLUDES=(-I"${SCRIPT_DIR}")
  LIBS=(-pthread)
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
    "${SCRIPT_DIR}/video_replay.cpp" \
    "${SCRIPT_DIR}/path_params.cpp" \
    "${SCRIPT_DIR}/vision_pipeline.cpp" \
    "${SCRIPT_DIR}/path_controller.cpp" \
    "${LIBS[@]}" -o "${OUT}"
  echo "built ${OUT}"
  exit 0
fi

echo "Usage: $0 [selftest|target|replay|display-example|motor-test|motor-test-sim]" >&2
exit 2
