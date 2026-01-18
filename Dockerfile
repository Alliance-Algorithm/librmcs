FROM ubuntu:24.04 AS builder

# Install build dependencies for RISC-V GNU Toolchain
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    autoconf automake autotools-dev curl python3 python3-pip python3-tomli \
    libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex \
    texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev \
    git ca-certificates file \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

# Build RISC-V toolchain from source
WORKDIR /src
RUN git clone --depth 1 https://github.com/riscv-collab/riscv-gnu-toolchain \
    && cd /src/riscv-gnu-toolchain \
    && git submodule update --init --depth 1 binutils newlib gcc gdb \
    && ./configure --prefix=/opt/riscv32-none-elf --with-arch=rv32gcb --with-abi=ilp32d \
    && make -j$(nproc) newlib \
    && ./.github/dedup-dir.sh /opt/riscv32-none-elf/ \
    && find /opt/riscv32-none-elf -type f -exec sh -c 'file "$1" | grep -q "ELF" && strip "$1"' _ {} \;
    

FROM ubuntu:24.04 AS ci

ARG TARGETARCH
ARG TARGETARCH_UNAME=${TARGETARCH/amd64/x86_64}
ARG TARGETARCH_UNAME=${TARGETARCH_UNAME/arm64/aarch64}

# Set bash as the default shell
SHELL ["/bin/bash", "-c"]

# Configure timezone and locale
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=Etc/UTC
ENV DEBIAN_FRONTEND=noninteractive

# Install system tools, libraries, and compilers
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    # General utilities
    tzdata \
    vim wget curl \
    gnupg2 ca-certificates \
    zsh usbutils \
    cmake make ninja-build \
    git sudo \
    zip unzip xz-utils \
    openssh-client \
    # Host toolchain
    libc6-dev gcc-14 g++-14 \
    pkg-config libusb-1.0-0-dev \
    # Firmware dependencies (HPM SDK)
    libmpc3 \
    python3 python3-pip python3-venv \
    python3-yaml python3-jinja2 \
    # Cleanup
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* \
    # Configure GCC 14 as the default compiler
    && dpkg-divert --divert /usr/bin/gcc.distrib --rename /usr/bin/gcc \
    && dpkg-divert --divert /usr/bin/g++.distrib --rename /usr/bin/g++ \
    && dpkg-divert --divert /usr/bin/cc.distrib --rename /usr/bin/cc \
    && dpkg-divert --divert /usr/bin/c++.distrib --rename /usr/bin/c++ \
    && dpkg-divert --divert /usr/bin/${TARGETARCH_UNAME}-linux-gnu-gcc.distrib --rename /usr/bin/${TARGETARCH_UNAME}-linux-gnu-gcc \
    && dpkg-divert --divert /usr/bin/${TARGETARCH_UNAME}-linux-gnu-g++.distrib --rename /usr/bin/${TARGETARCH_UNAME}-linux-gnu-g++ \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 \
    && update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 50 \
    && update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 50 \
    && ln -sf /usr/bin/gcc-14 /usr/bin/${TARGETARCH_UNAME}-linux-gnu-gcc \
    && ln -sf /usr/bin/g++-14 /usr/bin/${TARGETARCH_UNAME}-linux-gnu-g++

# Copy RISC-V toolchain from builder stage
COPY --from=builder /opt/riscv32-none-elf /opt/riscv32-none-elf
ENV GNURISCV_TOOLCHAIN_PATH=/opt/riscv32-none-elf
ENV PATH="${GNURISCV_TOOLCHAIN_PATH}/bin:${PATH}"

# Download and install ARM GNU Toolchain
RUN VERSION=15.2.rel1 \
    && wget https://developer.arm.com/-/media/Files/downloads/gnu/${VERSION}/binrel/arm-gnu-toolchain-${VERSION}-${TARGETARCH_UNAME}-arm-none-eabi.tar.xz \
        -O arm-gnu-toolchain.tar.xz \
    && tar -xvf arm-gnu-toolchain.tar.xz -C /opt/ \
    && rm arm-gnu-toolchain.tar.xz \
    && mv /opt/arm-gnu-toolchain-${VERSION}-${TARGETARCH_UNAME}-arm-none-eabi /opt/arm-none-eabi
ENV GNUARM_TOOLCHAIN_PATH=/opt/arm-none-eabi
ENV PATH="${GNUARM_TOOLCHAIN_PATH}/bin:${PATH}"

FROM ci AS develop

# Install latest clangd
RUN wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | tee /etc/apt/trusted.gpg.d/apt.llvm.org.asc \
    && echo "deb https://apt.llvm.org/noble/ llvm-toolchain-noble main" > /etc/apt/sources.list.d/llvm.list \
    && apt-get update \
    && version=$(apt-cache search clangd- | grep clangd- | awk -F' ' '{print $1}' | sort -V | tail -1 | cut -d- -f2) \
    && apt-get install -y --no-install-recommends clangd-$version \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-$version 50

# Add optional Tsinghua mirror configuration for CN users
COPY <<EOF /etc/apt/sources.list.d/ubuntu.sources.cn.bak
Types: deb
URIs: http://mirrors.tuna.tsinghua.edu.cn/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF

# Configure 'ubuntu' user and sudo privileges
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# Install Oh My Zsh and configure theme
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" \
    && sed -i 's/ZSH_THEME=\"[a-z0-9\\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc \
    && ln -s /workspaces/librmcs/.devcontainer/.zsh_history ~/.zsh_history
