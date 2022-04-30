FROM openptrack/open_ptrack-dep:1804
LABEL maintainer "Matteo Terreran"

ARG branch=1804

RUN echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections && \
    apt-get update && apt-get install -y apt-transport-https apt-utils less lsb-release udev wget && \
    mkdir -p /etc/udev/rules.d/ && \
    cd /tmp/ && \
    wget -O ZED_SDK_Linux_Ubuntu18_2_8.run https://download.stereolabs.com/zedsdk/2.8/ubuntu18 && \
    chmod +x ./ZED_SDK_Linux_Ubuntu18_*.run && \
    ./ZED_SDK_Linux_Ubuntu18_*.run --noexec --keep --target temp_zed && \
    rm -f ZED_SDK_Linux_Ubuntu18_*.run && \
    cd temp_zed/ && \
    ./linux_install_release.sh silent && \
    #rm -rf /var/lib/apt/lists/* && \
    cd / && \
    rm -rf /tmp/temp_zed

RUN git clone --branch v2.8.x https://github.com/stereolabs/zed-ros-wrapper.git ~/workspace/ros/src/zed-ros-wrapper

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && rm -f /etc/apt/sources.list.d/realsense-public.list \
    && apt-get update \
    && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg \
    && apt-get install ros-melodic-ddynamic-reconfigure

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash \
	&& git clone https://github.com/intel-ros/realsense.git ~/workspace/ros/src/realsense\
    && apt-get install ros-melodic-rgbd-launch"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash" \
    && git clone https://github.com/unipd-rvlab/open_ptrack_v2 ~/workspace/ros/src/open_ptrack \
    && cd ~/workspace/ros/src/open_ptrack \
    && git checkout $branch \
    && cd rtpose_wrapper     \  
    && apt-get --assume-yes install libprotobuf-dev libleveldb-dev libsnappy-dev libhdf5-serial-dev protobuf-compiler libboost-all-dev libgflags-dev libgoogle-glog-dev liblmdb-dev    \
    && make all -j8

# OpenPose dependencies
RUN  cd     \ 
    && mkdir third_party  \
    && cd third_party  \
    && git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose  \
    && cd openpose  \
    && git checkout v1.7.0  \
    && git submodule update --init --recursive --remote  \
    && cd models  \
    && ./getModels.sh  \
    && cd ..  \
    && mkdir build  \
    && cd scripts/ubuntu  \
    && chmod +x install_deps.sh  \
    && ./install_deps.sh \
    && cd ~/third_party/openpose/build  \
    && cmake \                                                                      
        -DDOWNLOAD_BODY_25_MODEL=ON \                                        
        -DDOWNLOAD_BODY_COCO_MODEL=ON \                                        
        -DDOWNLOAD_BODY_MPI_MODEL=ON \                                        
        -DDOWNLOAD_FACE_MODEL=ON \                                        
        -DDOWNLOAD_HAND_MODEL=ON \                                    
        -DUSE_CUDNN=OFF  .. \
    ## FIXME as v1.7.0 support AMPERE architectur, however we are using cuda 10 which is not compatible with AMPERE we comment these lines as a quick fix
    ## https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1753
    && sed -ie 's/set(AMPERE "80 86")/#&/g'  ../cmake/Cuda.cmake \
    && sed -ie 's/set(AMPERE "80 86")/#&/g'  ../3rdparty/caffe/cmake/Cuda.cmake \
    && make -j $(nproc) \
    && make install  \
    && echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc \
    && ln -s ~/third_party/openpose/models ~/workspace/ros/src/open_ptrack/openpose_wrapper/  \
    && echo "Added symlink to openpose models: $(ls -l ~/workspace/ros/src/open_ptrack/openpose_wrapper)" 

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash \
    && cd ~/workspace/ros \
    && export LD_LIBRARY_PATH=/root/workspace/ros/devel/lib:/opt/ros/melodic/lib:/opt/ros/melodic/lib/x86_64-linux-gnu:/usr/local/lib:/usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/local/cuda/lib64 \
    && echo "LD_LIBRARY_PATH: $(LD_LIBRARY_PATH)" 
    && catkin_make -j8 -DCMAKE_BUILD_TYPE=Release"

RUN /bin/bash -c "source /root/workspace/ros/devel/setup.bash \
    && roscd yolo_detector/darknet_opt \
    && wget -O coco.weights https://pjreddie.com/media/files/yolo.weights"

RUN cd /root/workspace/ros/src/open_ptrack/recognition/install_scripts \
    && wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2 \
    && bunzip2 shape_predictor_68_face_landmarks.dat.bz2 \ 
    && mv shape_predictor_68_face_landmarks.dat ../data/ \
    && wget https://storage.cmusatyalab.org/openface-models/nn4.small2.v1.t7 \
    && mv nn4.small2.v1.t7 ../data/

RUN ln -s /root/workspace/ros/src/open_ptrack /root/open_ptrack

WORKDIR /root
