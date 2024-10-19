FROM archlinux:base-devel

ARG TOOLS_PATH=/opt/stm32-tools

# Install essential system packages and clean cache in the same step to reduce image size
RUN pacman -Syyu --noconfirm && \
    pacman -Sy --noconfirm \
    base-devel \
    wget \
    unzip \
    jre-openjdk \
    libusb \
    usbutils \
    arm-none-eabi-gcc \
    arm-none-eabi-gdb \
    cmake \
    ninja \
    git \
    stlink && \
    pacman -Scc --noconfirm && rm -rf /var/cache/pacman/pkg/*

# Create directory for STM32 tools
RUN mkdir -p ${TOOLS_PATH}

# Install STM32CubeMX
RUN echo "Downloading STM32CubeMX..." && \
    wget -q --timeout=30 --tries=3 https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/82/0b/c5/29/2d/f4/47/4d/stm32cubemx/files/STM32CubeMX-latest-linux.zip \
    -O /tmp/STM32CubeMX.zip && \
    echo "Unzipping..." && \
    unzip /tmp/STM32CubeMX.zip -d ${TOOLS_PATH} && \
    rm /tmp/STM32CubeMX.zip && \
    chmod +x ${TOOLS_PATH}/STM32CubeMX/STM32CubeMX


# Install STM32CubeProgrammer
RUN wget -q --timeout=30 --tries=3 https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/00/79/75/57/6c/ba/43/58/stm32cubeprg/files/STM32CubeProgrammer-latest-linux.zip \
    -O /tmp/STM32CubeProgrammer.zip && \
    unzip /tmp/STM32CubeProgrammer.zip -d ${TOOLS_PATH} && \
    rm /tmp/STM32CubeProgrammer.zip && \
    chmod +x ${TOOLS_PATH}/STM32CubeProgrammer/bin/STM32_Programmer_CLI

# Install STM32CubeFinder
RUN wget -q --timeout=30 --tries=3 https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/67/fb/9f/7a/2b/8d/44/36/STM32CubeFinder/files/STM32CubeFinder-latest-linux.zip \
    -O /tmp/STM32CubeFinder.zip && \
    unzip /tmp/STM32CubeFinder.zip -d ${TOOLS_PATH} && \
    rm /tmp/STM32CubeFinder.zip && \
    chmod +x ${TOOLS_PATH}/STM32CubeFinder/STM32CubeFinder

# Install ST-LINK server
RUN wget -q --timeout=30 --tries=3 https://www.st.com/content/ccc/resource/technical/software/sw_development_suite/group0/55/b6/c7/cc/83/00/4c/6d/st-link-server_v2-0-1-0_linux_x86_64.zip \
    -O /tmp/ST-LINK-server.zip && \
    unzip /tmp/ST-LINK-server.zip -d ${TOOLS_PATH} && \
    rm /tmp/ST-LINK-server.zip && \
    chmod +x ${TOOLS_PATH}/stlink-server/stlink-server

# Create symbolic links for easy access
RUN ln -s ${TOOLS_PATH}/STM32CubeMX/STM32CubeMX /usr/local/bin/STM32CubeMX && \
    ln -s ${TOOLS_PATH}/STM32CubeProgrammer/bin/STM32_Programmer_CLI /usr/local/bin/STM32_Programmer_CLI && \
    ln -s ${TOOLS_PATH}/STM32CubeFinder/STM32CubeFinder /usr/local/bin/STM32CubeFinder && \
    ln -s ${TOOLS_PATH}/stlink-server/stlink-server /usr/local/bin/stlink-server

# Set up udev rules for ST-LINK USB access
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0666", GROUP="plugdev"' > /etc/udev/rules.d/99-stlink.rules && \
    udevadm control --reload-rules && udevadm trigger

# Set default shell to bash
SHELL ["/bin/bash", "-c"]
