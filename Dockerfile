FROM ros:kinetic-robot

RUN apt update && \
    apt full-upgrade -y && \
    apt install -y git ssh openssh-server mercurial fish build-essential \
        cmake git vim libgtk2.0-dev pkg-config \
        libqt4-dev libqtcore4 libqt5core5a \
        libavcodec-dev libavformat-dev libswscale-dev \
        python-dev python-numpy libtbb2 libtbb-dev \
        libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

# INSTALL OPENCV
WORKDIR /
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local .. &&\
    make -j$(nproc) && \
    make install

# INSTALL EIGEN
RUN hg clone https://bitbucket.org/eigen/eigen/ \
    && cd eigen && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DEIGEN_TEST_CXX11=ON .. \
    && make install -j$(nproc)

#RUN wget http://bitbucket.org/eigen/eigen/get/3.3.1.tar.gz \
#    && tar xzf 3.3.1.tar.gz \
#    && cd eigen-eigen-f562a193118d \
#    && mkdir build \
#    && cd build \
#    && cmake -DCMAKE_BUILD_TYPE=Release -DEIGEN_TEST_CXX11=ON .. \
#    && make install -j$(nproc) \
#    && cd / \
#    && rm -rf /3.3.1.tar.gz /eigen-eigen-f562a193118d

# SETUP ARUCO
ADD . /aruco
RUN cd /aruco && \
    rm -rf build && mkdir build && cd build && \
    /ros_entrypoint.sh cmake \
        -DUSE_OWN_EIGEN3=ON \
        -DEIGEN3_INCLUDE_DIR=/usr/local/include/eigen3 \
        .. && \
    /ros_entrypoint.sh make -j$(nproc)

