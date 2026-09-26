#!/usr/bin/env bash
set -euo pipefail

BOARD_IP="192.168.43.220"
BOARD_USER="root"
RUN_MODE="none"
BUILD_JOBS=4
BUILD_ONLY=0
FORCE_REBUILD=0
SKIP_MODELS=0
NO_BACKUP=0
CONFIRM_WHEELS_LIFTED=0
IDENTITY_SOURCE=""

usage() {
    cat <<'EOF'
Usage: deploy_rewrite.sh [options]
  --board-ip IP
  --board-user USER
  --run none|dry|motors
  --jobs N
  --build-only
  --force-rebuild
  --skip-models
  --no-backup
  --confirm-wheels-lifted
  --identity-source PATH
EOF
}

while (($#)); do
    case "$1" in
        --board-ip) BOARD_IP="${2:?missing board IP}"; shift 2 ;;
        --board-user) BOARD_USER="${2:?missing board user}"; shift 2 ;;
        --run) RUN_MODE="${2:?missing run mode}"; shift 2 ;;
        --jobs) BUILD_JOBS="${2:?missing job count}"; shift 2 ;;
        --build-only) BUILD_ONLY=1; shift ;;
        --force-rebuild) FORCE_REBUILD=1; shift ;;
        --skip-models) SKIP_MODELS=1; shift ;;
        --no-backup) NO_BACKUP=1; shift ;;
        --confirm-wheels-lifted) CONFIRM_WHEELS_LIFTED=1; shift ;;
        --identity-source) IDENTITY_SOURCE="${2:?missing identity path}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

case "${RUN_MODE}" in
    none|dry|motors) ;;
    *) echo "Invalid --run: ${RUN_MODE}" >&2; exit 2 ;;
esac
[[ "${BUILD_JOBS}" =~ ^[1-9][0-9]*$ ]] || {
    echo "Invalid --jobs: ${BUILD_JOBS}" >&2
    exit 2
}
if [[ "${RUN_MODE}" == "motors" &&
      "${CONFIRM_WHEELS_LIFTED}" != "1" ]]; then
    echo "Refusing motor mode until --confirm-wheels-lifted is passed." >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_DIR="${LQ_ENV_DIR:-/mnt/c/Users/wyc18/Downloads/lq环境配置 (2)}"
TC_NAME="loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
TC_DIR="${ENV_DIR}/${TC_NAME}"
TC_ARCHIVE="${ENV_DIR}/${TC_NAME}.tar.xz"
DEP_ROOT="${ENV_DIR}/LQ_Dep_libs"
OCV="${DEP_ROOT}/opencv_install"
NCNN="${DEP_ROOT}/ncnn_install"
LQ_DEMO="${REPO_ROOT}/Loongson_2k301_LIB-master/Loongson_2k301_LIB-master/LQ_ls2k301_Demo"
OUT="${REPO_ROOT}/build/rewrite/lq_path_follow_rewrite"
REMOTE="${BOARD_USER}@${BOARD_IP}"
REMOTE_PATH="/home/root/lq_path_follow_rewrite"
REMOTE_UPLOAD="${REMOTE_PATH}.upload"
IDENTITY_FILE="${HOME}/.ssh/deploy_rewrite_id_rsa"

need_file() {
    [[ -f "$1" ]] || { echo "Missing file: $1" >&2; exit 1; }
}
need_dir() {
    [[ -d "$1" ]] || { echo "Missing directory: $1" >&2; exit 1; }
}
extract_sha256() {
    { grep -Eo '[0-9a-fA-F]{64}' || true; } |
        tail -n 1 | tr 'A-F' 'a-f'
}

if [[ ! -x "${TC_DIR}/bin/loongarch64-linux-gnu-g++" ]]; then
    need_file "${TC_ARCHIVE}"
    echo "==> Restoring GCC 8.3 toolchain"
    tar -xJf "${TC_ARCHIVE}" -C "${ENV_DIR}"
fi
CXX="${TC_DIR}/bin/loongarch64-linux-gnu-g++"
READELF="${TC_DIR}/bin/loongarch64-linux-gnu-readelf"
need_file "${CXX}"
[[ "$("${CXX}" -dumpversion)" == 8.3* ]] || {
    echo "Refusing compiler other than GCC 8.3: ${CXX}" >&2
    exit 1
}
need_dir "${OCV}/include/opencv4"
need_dir "${OCV}/lib"
need_dir "${NCNN}/include"
need_dir "${NCNN}/lib"

echo "==> Building rewrite with old-world GCC 8.3"
CXX="${CXX}" \
OPENCV_DIR="${OCV}" \
NCNN_DIR="${NCNN}" \
LQ_DEMO="${LQ_DEMO}" \
BUILD_JOBS="${BUILD_JOBS}" \
FORCE_REBUILD="${FORCE_REBUILD}" \
bash "${SCRIPT_DIR}/build_rewrite.sh" target
need_file "${OUT}"
"${READELF}" -h "${OUT}" | grep -q LoongArch || {
    echo "Built artifact is not LoongArch." >&2
    exit 1
}
LOCAL_HASH="$(sha256sum "${OUT}" | awk '{print $1}')"
echo "    ${OUT}"
echo "    SHA-256 ${LOCAL_HASH}"
if [[ "${BUILD_ONLY}" == "1" ]]; then
    exit 0
fi

if [[ -n "${IDENTITY_SOURCE}" ]]; then
    need_file "${IDENTITY_SOURCE}"
    mkdir -p "${HOME}/.ssh"
    chmod 700 "${HOME}/.ssh"
    install -m 600 "${IDENTITY_SOURCE}" "${IDENTITY_FILE}"
fi
mkdir -p "${HOME}/.ssh"
chmod 700 "${HOME}/.ssh"
SSH_OPTS=(
    -o BatchMode=yes
    -o ConnectTimeout=8
    -o StrictHostKeyChecking=accept-new
    -o HostKeyAlgorithms=+ssh-rsa
    -o PubkeyAcceptedAlgorithms=+ssh-rsa
    -o ControlMaster=auto
    -o ControlPersist=60
    -o "ControlPath=${HOME}/.ssh/deploy_rewrite_%C"
)
if [[ -f "${IDENTITY_FILE}" ]]; then
    SSH_OPTS+=( -i "${IDENTITY_FILE}" -o IdentitiesOnly=yes )
fi
ssh_board() { ssh "${SSH_OPTS[@]}" "${REMOTE}" "$@"; }

echo "==> Checking ${REMOTE}"
ssh_board "test \"\$(uname -m)\" = loongarch64"
INSTALLED_HASH="$(
    ssh_board "if [ -f '${REMOTE_PATH}' ]; then sha256sum '${REMOTE_PATH}'; fi" |
    extract_sha256
)"
if [[ "${INSTALLED_HASH}" == "${LOCAL_HASH}" ]]; then
    echo "==> Binary unchanged; upload skipped"
    BINARY_UPDATED=0
else
    BINARY_UPDATED=1
    scp -q "${SSH_OPTS[@]}" "${OUT}" "${REMOTE}:${REMOTE_UPLOAD}"
    REMOTE_HASH="$(
        ssh_board "sha256sum '${REMOTE_UPLOAD}'" | extract_sha256
    )"
    [[ "${REMOTE_HASH}" == "${LOCAL_HASH}" ]] || {
        ssh_board "rm -f '${REMOTE_UPLOAD}'" || true
        echo "Upload checksum mismatch." >&2
        exit 1
    }

    BACKUP="cp -f '${REMOTE_PATH}' '${REMOTE_PATH}.bak'"
    if [[ "${NO_BACKUP}" == "1" ]]; then BACKUP=":"; fi
    ssh_board "set -e; killall lq_path_follow_rewrite 2>/dev/null || true; \
if [ -f '${REMOTE_PATH}' ]; then ${BACKUP}; fi; \
mv -f '${REMOTE_UPLOAD}' '${REMOTE_PATH}'; chmod +x '${REMOTE_PATH}'; sync"
fi

echo "==> Synchronizing calibration"
ssh_board "mkdir -p /home/root/rewrite /home/root/models"
sync_asset() {
    local source="$1"
    local destination="$2"
    local local_hash remote_hash
    need_file "${source}"
    local_hash="$(sha256sum "${source}" | awk '{print $1}')"
    remote_hash="$(
        ssh_board "if [ -f '${destination}' ]; then sha256sum '${destination}'; fi" |
        extract_sha256
    )"
    if [[ "${local_hash}" == "${remote_hash}" ]]; then
        echo "    unchanged ${destination}"
        return
    fi
    scp -q "${SSH_OPTS[@]}" "${source}" "${REMOTE}:${destination}.upload"
    remote_hash="$(
        ssh_board "sha256sum '${destination}.upload'" | extract_sha256
    )"
    [[ "${local_hash}" == "${remote_hash}" ]] || {
        ssh_board "rm -f '${destination}.upload'" || true
        echo "Asset checksum mismatch: ${destination}" >&2
        exit 1
    }
    ssh_board "mv -f '${destination}.upload' '${destination}'"
    echo "    updated ${destination}"
}

sync_asset "${SCRIPT_DIR}/标定数据.txt" "/home/root/rewrite/标定数据.txt"
sync_asset "${SCRIPT_DIR}/setup_st7735s_spi.sh" "/home/root/setup_st7735s_spi.sh"
sync_asset "${SCRIPT_DIR}/record_imu_path.sh" "/home/root/record_imu_path.sh"
sync_asset "${OUT}" "/home/root/lq_path_follow_imu"
ssh_board "chmod +x /home/root/setup_st7735s_spi.sh"
ssh_board "chmod +x /home/root/record_imu_path.sh"
ssh_board "chmod +x /home/root/lq_path_follow_imu"

if [[ "${SKIP_MODELS}" != "1" ]]; then
    for asset in best.ncnn.param best.ncnn.bin model_metadata.json; do
        source="${REPO_ROOT}/target/model/${asset}"
        sync_asset "${source}" "/home/root/models/${asset}"
    done
fi

FINAL_HASH="$(
    ssh_board "sha256sum '${REMOTE_PATH}'" | extract_sha256
)"
[[ "${FINAL_HASH}" == "${LOCAL_HASH}" ]] || {
    echo "Final checksum mismatch." >&2
    exit 1
}

if [[ "${RUN_MODE}" != "none" ]]; then
    MODE_FLAG="--dry-run"
    if [[ "${RUN_MODE}" == "motors" ]]; then
        MODE_FLAG="--enable-motors"
    fi
    RUN_ARGS="--camera /dev/video0 --http 8080 \
--model-dir /home/root/models \
--calibration /home/root/rewrite/标定数据.txt \
--target-actions --target-input-size 32 --target-close-size 0.04 ${MODE_FLAG}"
    ssh_board "set -e; killall lq_path_follow_rewrite 2>/dev/null || true; \
cd /home/root; \
nohup env LD_LIBRARY_PATH=/home/root/LQ_Dep_libs/opencv-lib:/home/root/LQ_Dep_libs/ncnn-lib \
./lq_path_follow_rewrite ${RUN_ARGS} \
> /home/root/lq_path_follow_rewrite.log 2>&1 </dev/null & \
echo \$! > /home/root/lq_path_follow_rewrite.pid; sleep 2; \
kill -0 \$(cat /home/root/lq_path_follow_rewrite.pid)"
    echo "==> Running ${RUN_MODE}: http://${BOARD_IP}:8080"
    echo "==> Telemetry: http://${BOARD_IP}:8080/telemetry"
else
    if [[ "${BINARY_UPDATED}" == "0" ]]; then
        ssh_board "killall lq_path_follow_rewrite 2>/dev/null || true"
    fi
    echo "==> Installed independently; program not started"
fi
