wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"

bash Miniforge3-$(uname)-$(uname -m).sh
rm Miniforge3-$(uname)-$(uname -m).sh

cd lbr-stack/

conda create -n ros_env -c conda-forge -c robostack-humble ros-humble-desktop

conda activate ros_env

conda install -c conda-forge compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep

conda activate ros_env

rosdep install --from-paths src -i -r -y

colcon build 
source install/setup.bash