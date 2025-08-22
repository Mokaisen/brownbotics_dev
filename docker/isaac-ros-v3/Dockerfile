# ==============================================================
# Isaac Sim 5.0 (Ubuntu 22.04) - Prebuilt Image
# ==============================================================

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------------------
# System dependencies
# --------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    gcc-11 g++-11 \
    cmake \
    git git-lfs \
    curl wget \
    python3 python3-pip python3-dev \
    software-properties-common \
    libx11-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev \
    libxrandr-dev libxinerama-dev libxcursor-dev \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Configure GCC/G++ to use version 11
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200 \
 && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200

# --------------------------------------------------------------
# Workspace setup
# --------------------------------------------------------------
# Copy your local IsaacSim repo into the image
COPY IsaacSim /workspace/isaacsim
WORKDIR /workspace/isaacsim

# --------------------------------------------------------------
# Full build of Isaac Sim (release only)
# --------------------------------------------------------------
ARG ACCEPT_EULA=Yes
ENV ACCEPT_EULA=${ACCEPT_EULA}
RUN ./build.sh --release

# --------------------------------------------------------------
# Default runtime directory
# --------------------------------------------------------------
WORKDIR /workspace/isaacsim/_build/linux-x86_64/release

# Set the default entrypoint to bash
#ENTRYPOINT ["bash"]
ENTRYPOINT ["/bin/bash"]

# Default command
#CMD ["./isaac-sim.sh"]