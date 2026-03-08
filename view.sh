#!/usr/bin/env bash
# ─────────────────────────────────────────────
# 快速预览任意 MuJoCo XML 模型
# 用法:
#   ./view.sh                                           # 打开默认模型
#   ./view.sh path/to/your_model.xml                    # 打开指定 XML
# ─────────────────────────────────────────────

DEFAULT_XML="ros2_ws/src/mujoco_pendulum/models/inverted_pendulum.xml"
XML="${1:-$DEFAULT_XML}"

if [ ! -f "$XML" ]; then
  echo "❌ 找不到文件: $XML"
  exit 1
fi

echo "🚀 打开 MuJoCo viewer: $XML"
python3 -m mujoco.viewer --mjcf "$XML"
