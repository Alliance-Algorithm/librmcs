FROM ubuntu:24.04

ARG TARGETARCH
ARG TARGETARCH_UNAME=${TARGETARCH/amd64/x86_64}
ARG TARGETARCH_UNAME=${TARGETARCH_UNAME/arm64/aarch64}

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Set timezone and locale
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=Etc/UTC
ENV DEBIAN_FRONTEND=noninteractive

# Install tools and libraries
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    # General
    tzdata \
    vim wget curl \
    gnupg2 ca-certificates \
    zsh usbutils \
    cmake make ninja-build \
    git sudo \
    unzip xz-utils \
    openssh-client \
    # Host
    libc6-dev gcc-14 g++-14 \
    pkg-config libusb-1.0-0-dev \
    # Firmware (HPM SDK)
    libmpc3 \
    python3 python3-pip python3-venv \
    python3-yaml python3-jinja2 \
    # Clean
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* \
    # Specify default gcc version
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

# Download the latest riscv32 toolchain and expand it into /opt/riscv
RUN VERSION=15.2-r1 \
    && wget https://releases.riscstar.com/toolchain/${VERSION}/riscstar-toolchain-${VERSION}-${TARGETARCH_UNAME}-riscv32-none-elf.tar.xz \
        -O riscv-gnu-toolchain.tar.xz \
    && tar -xvf riscv-gnu-toolchain.tar.xz -C /opt/ \
    && rm riscv-gnu-toolchain.tar.xz \
    && mv /opt/riscstar-toolchain-${VERSION}-${TARGETARCH_UNAME}-riscv32-none-elf /opt/riscv32-none-elf
ENV GNURISCV_TOOLCHAIN_PATH=/opt/riscv32-none-elf
ENV PATH="${GNURISCV_TOOLCHAIN_PATH}/bin:${PATH}"

# Download the latest arm-gnu-toolchain and expand it into /opt/arm-none-eabi
RUN VERSION=15.2.rel1 \
    && wget https://developer.arm.com/-/media/Files/downloads/gnu/${VERSION}/binrel/arm-gnu-toolchain-${VERSION}-${TARGETARCH_UNAME}-arm-none-eabi.tar.xz \
        -O arm-gnu-toolchain.tar.xz \
    && tar -xvf arm-gnu-toolchain.tar.xz -C /opt/ \
    && rm arm-gnu-toolchain.tar.xz \
    && mv /opt/arm-gnu-toolchain-${VERSION}-${TARGETARCH_UNAME}-arm-none-eabi /opt/arm-none-eabi
ENV GNUARM_TOOLCHAIN_PATH=/opt/arm-none-eabi
ENV PATH="${GNUARM_TOOLCHAIN_PATH}/bin:${PATH}"

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

# For CN users: optional Tsinghua mirror
COPY <<EOF /etc/apt/sources.list.d/ubuntu.sources.cn.bak
Types: deb
URIs: http://mirrors.tuna.tsinghua.edu.cn/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF

# Change user
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# Install oh my zsh & change theme to af-magic
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" \
    && sed -i 's/ZSH_THEME=\"[a-z0-9\\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc \
    && ln -s /workspaces/librmcs/.devcontainer/.zsh_history ~/.zsh_history
