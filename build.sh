#!/usr/bin/env bash
# 从当前包构建，避免递归扫描固件、ROS 1 和参考工程。
set -euo pipefail
cd "$(dirname "$0")"
if [[ "${ROS_VERSION:-}" != 2 ]]; then
    echo '请先执行 dev ros2 加载 ROS 2 环境。' >&2
    exit 2
fi
# WSL 内存有限，默认最多两个编译任务，可由调用者覆盖。
export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
export MAKEFLAGS=${MAKEFLAGS:--j2}
colcon --log-base log-wsl build \
    --base-paths . --packages-select tianbot_core --executor sequential \
    --build-base build-wsl --install-base install-wsl \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
