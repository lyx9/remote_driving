# ============================================================
# FSM-Pilot Cloud Server — 多阶段 Docker 镜像
# 编译阶段使用 ROS2 Humble，运行阶段仅保留必要运行库
# ============================================================

# ---- 阶段 1: 编译 ----
FROM ros:humble-ros-base-jammy AS builder

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libboost-all-dev \
    libssl-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libprotobuf-dev \
    protobuf-compiler \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY cpp/ .

# 仅构建 common + cloud_server（不需要 vehicle_node/ROS2 传感器消息）
RUN . /opt/ros/humble/setup.sh && \
    cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF \
      -DUSE_GRPC=OFF \
    && cmake --build build --target fsm_cloud_server -j$(nproc)

# ---- 阶段 2: 运行时 ----
FROM ubuntu:22.04 AS runtime

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libboost-system1.74.0 \
    libssl3 \
    libspdlog1 \
    libyaml-cpp0.7 \
    libprotobuf23 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -r -s /sbin/nologin fsm

WORKDIR /opt/fsm

COPY --from=builder /src/build/cloud_server/fsm_cloud_server ./bin/fsm_cloud_server
COPY --from=builder /src/build/libfsm_common.so             ./lib/

ENV LD_LIBRARY_PATH=/opt/fsm/lib

# 配置文件从外部挂载
VOLUME ["/etc/fsm"]

EXPOSE 8080

USER fsm

ENTRYPOINT ["/opt/fsm/bin/fsm_cloud_server"]
CMD ["/etc/fsm/cloud_config.yaml"]
