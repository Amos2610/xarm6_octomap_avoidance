WORKSPACE_DIR=$(pwd)
echo "WORKSPACE_DIR: $WORKSPACE_DIR"

echo "Setting up the environment for xarm6_octomap_avoidance"

echo "Installing dependencies..."
sudo apt update && sudo apt install -y \
  ros-noetic-xacro \
  ros-noetic-joint-state-publisher \
  ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-state-publisher \
  ros-noetic-moveit \
  ros-noetic-moveit-commander \
  ros-noetic-moveit-ros-planning-interface \
  ros-noetic-moveit-ros-perception \
  ros-noetic-moveit-ros-visualization \
  ros-noetic-moveit-planners-ompl \
  ros-noetic-octomap-server \
  ros-noetic-octomap-msgs \
  ros-noetic-pcl-ros \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-rviz \
  ros-noetic-ros-control \
  ros-noetic-controller-manager \
  ros-noetic-realsense2-camera 

sudo apt-get install ros-noetic-robot-self-filter

echo "Building the workspace..."
catkin build
source devel/setup.bash
roscd xarm6_octomap_avoidance

echo "Installing Python dependencies..."
pip install -r requirements.txt

echo "✅ Setting up the environment for xarm6_octomap_avoidance is complete."
