FROM  moveit/moveit2:galactic-release

# install pip and python dev dependencies
RUN \
    apt-get -q update && \
    apt-get -q install --no-install-recommends -y python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install pre-commit

# install clang-format
RUN \
    apt-get -q update && \
    apt-get -q install --no-install-recommends -y clang-format-10 && \
    rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
# cf https://stackoverflow.com/a/62354513/11579406
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
