#!/bin/bash
# FSM-Pilot C++ 编译脚本
# 自动处理 Anaconda 与系统库的冲突

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CPP_DIR="$PROJECT_DIR/cpp"
BUILD_DIR="$CPP_DIR/build"
BUILD_TYPE="${1:-Release}"

echo "╔════════════════════════════════════════╗"
echo "║     FSM-Pilot C++ 编译工具             ║"
echo "╚════════════════════════════════════════╝"
echo ""

# =============================================================================
# 检查并处理 Anaconda 环境冲突
# =============================================================================
handle_anaconda() {
    if [ -n "$CONDA_DEFAULT_ENV" ] || [ -n "$CONDA_PREFIX" ]; then
        echo "⚠️  检测到 Conda 环境已激活: $CONDA_DEFAULT_ENV"
        echo "   为避免库冲突，将临时停用 Conda..."
        echo ""

        # 保存当前 PATH，移除 conda 相关路径
        ORIGINAL_PATH="$PATH"
        export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -E "(anaconda|miniconda|conda)" | tr '\n' ':' | sed 's/:$//')

        # 清除 conda 相关环境变量
        unset CONDA_PREFIX
        unset CONDA_DEFAULT_ENV
        unset CONDA_PYTHON_EXE
        unset CONDA_SHLVL

        echo "   ✓ Conda 路径已从 PATH 中移除"
    fi

    # 检查是否存在 Anaconda 安装
    if [ -d "/opt/anaconda3" ] || [ -d "$HOME/anaconda3" ] || [ -d "$HOME/miniconda3" ]; then
        echo "ℹ️  检测到 Anaconda/Miniconda 安装"
        echo "   CMake 将自动排除其库路径"
        echo ""
    fi
}

# =============================================================================
# 检查依赖
# =============================================================================
check_dependencies() {
    echo "检查依赖..."

    local missing=()

    # 检查必要命令
    command -v cmake >/dev/null 2>&1 || missing+=("cmake")
    command -v make >/dev/null 2>&1 || missing+=("make")
    command -v protoc >/dev/null 2>&1 || missing+=("protobuf-compiler")

    # 检查 ROS2
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        echo "⚠️  未检测到 ROS2 Humble"
        echo "   请先安装 ROS2: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi

    # 检查库文件
    [ -f "/usr/lib/x86_64-linux-gnu/libprotobuf.so" ] || missing+=("libprotobuf-dev")
    [ -f "/usr/lib/x86_64-linux-gnu/libyaml-cpp.so" ] || missing+=("libyaml-cpp-dev")

    if [ ${#missing[@]} -gt 0 ]; then
        echo "❌ 缺少以下依赖:"
        printf '   - %s\n' "${missing[@]}"
        echo ""
        echo "请运行以下命令安装:"
        echo "  sudo apt update"
        echo "  sudo apt install -y ${missing[*]}"
        exit 1
    fi

    echo "  ✓ 依赖检查通过"
    echo ""
}

# =============================================================================
# 安装依赖 (可选)
# =============================================================================
install_dependencies() {
    echo "安装编译依赖..."
    sudo apt update
    sudo apt install -y \
        build-essential \
        cmake \
        pkg-config \
        libprotobuf-dev \
        protobuf-compiler \
        libyaml-cpp-dev \
        libspdlog-dev \
        nlohmann-json3-dev \
        libopencv-dev \
        libboost-system-dev \
        libssl-dev \
        libwebsocketpp-dev

    echo "  ✓ 系统依赖安装完成"
    echo ""
}

# =============================================================================
# 编译
# =============================================================================
build() {
    echo "开始编译..."
    echo "  构建类型: $BUILD_TYPE"
    echo "  构建目录: $BUILD_DIR"
    echo ""

    # Source ROS2
    source /opt/ros/humble/setup.bash

    # 创建构建目录
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    # 清除 CMake 缓存 (如果存在冲突)
    if [ -f "CMakeCache.txt" ]; then
        if grep -q "anaconda\|conda" CMakeCache.txt 2>/dev/null; then
            echo "  检测到旧缓存包含 Anaconda 路径，正在清除..."
            rm -rf CMakeCache.txt CMakeFiles/
        fi
    fi

    # 运行 CMake
    echo "  运行 CMake..."
    cmake .. \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_PREFIX_PATH="/opt/ros/humble;/usr;/usr/local" \
        -DProtobuf_DIR="/usr/lib/x86_64-linux-gnu/cmake/protobuf" \
        -DUSE_GRPC=OFF \
        2>&1 | tee cmake_output.log

    if [ ${PIPESTATUS[0]} -ne 0 ]; then
        echo ""
        echo "❌ CMake 配置失败"
        echo "   请查看 $BUILD_DIR/cmake_output.log"
        exit 1
    fi

    echo ""
    echo "  编译中..."
    make -j$(nproc) 2>&1 | tee build_output.log

    if [ ${PIPESTATUS[0]} -ne 0 ]; then
        echo ""
        echo "❌ 编译失败"
        echo "   请查看 $BUILD_DIR/build_output.log"
        exit 1
    fi

    echo ""
    echo "✅ 编译成功!"
    echo ""
    echo "生成的可执行文件:"
    find "$BUILD_DIR" -maxdepth 3 -type f -executable -name "fsm_*" | while read f; do
        echo "  - $f"
    done
}

# =============================================================================
# 主程序
# =============================================================================

# 解析参数
case "${1:-}" in
    --install-deps|-i)
        install_dependencies
        exit 0
        ;;
    --help|-h)
        echo "用法: $0 [选项] [构建类型]"
        echo ""
        echo "选项:"
        echo "  --install-deps, -i    安装编译依赖"
        echo "  --help, -h            显示帮助"
        echo ""
        echo "构建类型:"
        echo "  Release (默认)        优化构建"
        echo "  Debug                 调试构建"
        echo ""
        echo "示例:"
        echo "  $0                    Release 构建"
        echo "  $0 Debug              Debug 构建"
        echo "  $0 --install-deps     安装依赖"
        exit 0
        ;;
esac

# 处理 Anaconda
handle_anaconda

# 检查依赖
check_dependencies

# 编译
build

echo ""
echo "下一步:"
echo "  1. 运行车端节点:    $BUILD_DIR/vehicle_node/fsm_vehicle_node"
echo "  2. 运行云端服务:    $BUILD_DIR/cloud_server/fsm_cloud_server"
echo "  3. 运行操作端:      $BUILD_DIR/operator_client/fsm_operator_client"
