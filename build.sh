#!/usr/bin/env bash
# 用法:
#   ./build.sh                仅编译
#   ./build.sh flash [PORT]   编译并烧录 (PORT 缺省自动探测 /dev/cu.usbmodem*)
#   ./build.sh monitor [PORT] 串口日志
#   ./build.sh clean
set -euo pipefail
cd "$(dirname "$0")"

IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf-v6.1}"
export IDF_PATH
# shellcheck disable=SC1091
. "$IDF_PATH/export.sh" >/dev/null

detect_port() { ls /dev/cu.usbmodem* 2>/dev/null | head -n1 || true; }

case "${1:-build}" in
  build)
    idf.py set-target esp32s3 >/dev/null 2>&1 || true
    idf.py build
    ;;
  flash)
    PORT="${2:-$(detect_port)}"
    [ -n "$PORT" ] || { echo "未检测到串口, 请连接开发板或手动指定 PORT"; exit 1; }
    idf.py -p "$PORT" build flash
    echo "烧录完成: 连接 Wi-Fi 4WD-Car (12345678), 浏览器打开 http://192.168.4.1"
    ;;
  monitor)
    PORT="${2:-$(detect_port)}"
    idf.py -p "$PORT" monitor
    ;;
  clean) idf.py fullclean ;;
  *) echo "unknown: $1"; exit 1 ;;
esac
