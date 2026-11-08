#!/bin/bash
# Recompila el firmware a wasm con clang de wasi-sdk (alternativa a simulator/wasm/build.ps1).
# Uso: WASI_SDK_PATH=/c/ruta/wasi-sdk ./rebuild_wasm.sh   (por defecto: simulator/.toolchain)
R="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SDK="${WASI_SDK_PATH:-$(ls -d "$R"/simulator/.toolchain/wasi-sdk* 2>/dev/null | head -1)}"
"$SDK/bin/clang" --target=wasm32-wasi -O2 -std=c11 -Wall -Wextra -Werror -nostartfiles -Wl,--no-entry -Wl,-z,stack-size=262144 -Wl,--strip-all \
 -I "$R/LIDAR System/main" -I "$R/LIDAR System/orchestrator" -I "$R/Vision System/main" -I "$R/shared" \
 "$R/simulator/wasm/core_api.c" "$R/LIDAR System/orchestrator/p4_runtime.c" "$R/LIDAR System/main/park_ackermann.c" "$R/LIDAR System/main/local_obstacle_map.c" "$R/LIDAR System/main/camera_table_receiver.c" "$R/LIDAR System/main/imu_yaw.c" "$R/LIDAR System/orchestrator/wro_controller.c" "$R/LIDAR System/main/park_planner.c" "$R/LIDAR System/main/park_localize.c" "$R/LIDAR System/main/odometry.c" "$R/LIDAR System/main/arena_localizer.c" "$R/LIDAR System/main/race_line.c" \
 "$R/Vision System/main/vision.c" "$R/Vision System/main/vision_table.c" "$R/shared/sensor_table_protocol.c" "$R/shared/config_fields.c" \
 -o "$R/simulator/web/wro_core.wasm" -lm && python "$R/simulator/wasm/manifest.py" --write && echo "wasm ok"
