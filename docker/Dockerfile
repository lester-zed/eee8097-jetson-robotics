# FROM nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM nvcr.io/nvidia/pytorch:25.06-py3-igpu

ARG DEBIAN_FRONTEND=noninteractive

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1

# ============================================================
# System packages
# ============================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    \
    # Python
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-numpy \
    python3-opencv \
    python3-tk \
    \
    # Development tools
    git \
    curl \
    wget \
    vim \
    nano \
    htop \
    tmux \
    unzip \
    build-essential \
    cmake \
    pkg-config \
    \
    # USB / serial / camera tools
    usbutils \
    v4l-utils \
    \
    # Multimedia
    ffmpeg \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    \
    # OpenCV / GUI runtime
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgtk-3-0 \
    libgl1 \
    libgomp1 \
    \
    # Image / video development libraries
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libv4l-dev \
    \
    # Useful for some Jetson multimedia components
    kmod \
    \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Workspace
# ============================================================

WORKDIR /workspace


# ============================================================
# Python environment
#
# --system-site-packages is intentional:
# it allows the venv to use Jetson/Ubuntu packages such as
# python3-opencv and python3-numpy.
# ============================================================

RUN python3 -m venv --system-site-packages /opt/venv

ENV PATH="/opt/venv/bin:${PATH}"


# ============================================================
# Python dependencies
# ============================================================

COPY requirements.txt /tmp/requirements.txt

RUN python -m pip install --upgrade pip setuptools wheel && \
    python -m pip install --no-cache-dir -r /tmp/requirements.txt && \
    python -m pip check


# ============================================================
# Entrypoint
# ============================================================

COPY docker/entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]