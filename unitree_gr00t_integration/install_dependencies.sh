#!/bin/bash
# 安装脚本 - Unitree G1 + GR00T-N1.5 Integration
# 此脚本安装所有必要的依赖

set -e  # 遇到错误立即退出

echo "=========================================="
echo "Unitree G1 + GR00T-N1.5 Integration"
echo "依赖安装脚本"
echo "=========================================="
echo ""

# 获取脚本所在目录的父目录（Isaac-GR00T根目录）
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PARENT_DIR="$( cd "$SCRIPT_DIR/.." && pwd )"
GR00T_DIR="$PARENT_DIR/gr00t"

echo "当前目录: $SCRIPT_DIR"
echo "父目录: $PARENT_DIR"
echo ""

# 检查gr00t目录是否存在
if [ ! -d "$GR00T_DIR" ]; then
    echo "❌ 错误: 未找到 gr00t 目录"
    echo "   期望路径: $GR00T_DIR"
    echo ""
    echo "请确保 unitree_gr00t_integration 位于 Isaac-GR00T 仓库的根目录下"
    echo "目录结构应该是:"
    echo "  Isaac-GR00T/"
    echo "    ├── gr00t/"
    echo "    ├── scripts/"
    echo "    └── unitree_gr00t_integration/"
    exit 1
fi

echo "✓ 找到 gr00t 目录"

# 检查scripts目录是否存在
SCRIPTS_DIR="$PARENT_DIR/scripts"
if [ ! -d "$SCRIPTS_DIR" ]; then
    echo "⚠️  警告: 未找到 scripts 目录"
    echo "   期望路径: $SCRIPTS_DIR"
    echo "   某些功能可能无法使用"
else
    echo "✓ 找到 scripts 目录"
fi

echo ""
echo "步骤 1: 安装 gr00t 包..."
cd "$PARENT_DIR"
if pip install -e . > /dev/null 2>&1; then
    echo "✓ gr00t 包安装成功"
else
    echo "⚠️  警告: gr00t 包安装可能失败，尝试继续..."
    pip install -e . || echo "请手动运行: pip install -e $PARENT_DIR"
fi

echo ""
echo "步骤 2: 安装其他Python依赖..."
cd "$SCRIPT_DIR"
if pip install -r requirements.txt; then
    echo "✓ Python依赖安装成功"
else
    echo "❌ 错误: Python依赖安装失败"
    exit 1
fi

echo ""
echo "步骤 3: 验证安装..."
python3 -c "import gr00t; print('✓ gr00t 模块可以导入')" || {
    echo "❌ 错误: gr00t 模块无法导入"
    echo "请检查安装是否正确"
    exit 1
}

echo ""
echo "=========================================="
echo "✓ 安装完成！"
echo "=========================================="
echo ""
echo "下一步:"
echo "  1. 阅读 README.md 了解使用方法"
echo "  2. 运行快速开始: python quick_start.py --help"
echo ""
