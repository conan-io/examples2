FROM osrf/ros:kilted-desktop
RUN apt-get update && apt-get install -y \
curl \
python3-pip \
git \
ros-kilted-nav2-msgs \
&& rm -rf /var/lib/apt/lists/*
RUN pip3 install --upgrade pip && pip3 install conan
RUN conan profile detect
CMD ["bash"]
