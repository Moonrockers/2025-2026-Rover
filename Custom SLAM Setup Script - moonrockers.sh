#!/bin/bash
# Quick setup script for custom SLAM
# Run this to automatically set up the workspace

set -e

echo "========================================="
echo "Custom SLAM Setup Script"
echo "========================================="

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo -e "${YELLOW}ROS2 not found. Please install ROS2 Humble first.${NC}"
    echo "Visit: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 found${NC}"

# Create workspace
WORKSPACE=~/ros2_ws
if [ ! -d "$WORKSPACE" ]; then
    echo "Creating workspace at $WORKSPACE"
    mkdir -p $WORKSPACE/src
fi

cd $WORKSPACE/src

# Create package structure
PKG_NAME=custom_slam
echo "Creating package structure..."

mkdir -p $PKG_NAME/{$PKG_NAME,launch,config,resource}

# Create marker files
touch $PKG_NAME/resource/$PKG_NAME
touch $PKG_NAME/$PKG_NAME/__init__.py

# Create package.xml
cat > $PKG_NAME/package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <n>custom_slam</n>
  <version>1.0.0</version>
  <description>Custom SLAM implementation from scratch for Intel RealSense D435i</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>tf2_ros</depend>
  <depend>realsense2_camera</depend>
  <depend>rviz2</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Create setup.cfg
cat > $PKG_NAME/setup.cfg << 'EOF'
[develop]
script_dir=$base/lib/custom_slam
[install]
install_scripts=$base/lib/custom_slam
EOF

# Create setup.py
cat > $PKG_NAME/setup.py << 'EOF'
from setuptools import setup
import os
from glob import glob

package_name = 'custom_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.rviz') if os.path.exists('config') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Custom SLAM implementation from scratch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_slam_node = custom_slam.custom_slam_node:main',
            'visualizer_node = custom_slam.visualizer_node:main',
        ],
    },
)
EOF

echo -e "${GREEN}✓ Package structure created${NC}"

# Install dependencies
echo "Installing dependencies..."
sudo apt update
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    python3-opencv \
    python3-numpy \
    python3-scipy

echo -e "${GREEN}✓ Dependencies installed${NC}"

# Instructions for manual steps
echo ""
echo "========================================="
echo "Next Steps:"
echo "========================================="
echo ""
echo "1. Copy your Python files to the package:"
echo "   - custom_slam_node.py -> $WORKSPACE/src/$PKG_NAME/$PKG_NAME/"
echo "   - visualizer_node.py -> $WORKSPACE/src/$PKG_NAME/$PKG_NAME/"
echo "   - custom_slam.launch.py -> $WORKSPACE/src/$PKG_NAME/launch/"
echo ""
echo "2. Build the workspace:"
echo "   cd $WORKSPACE"
echo "   colcon build --packages-select $PKG_NAME --symlink-install"
echo ""
echo "3. Source the workspace:"
echo "   source $WORKSPACE/install/setup.bash"
echo ""
echo "4. Run the system:"
echo "   ros2 launch $PKG_NAME custom_slam.launch.py"
echo ""
echo "========================================="

# Create a helper script
cat > $WORKSPACE/run_slam.sh << 'EOF'
#!/bin/bash
source ~/ros2_ws/install/setup.bash
ros2 launch custom_slam custom_slam.launch.py
EOF
chmod +x $WORKSPACE/run_slam.sh

echo -e "${GREEN}✓ Helper script created at $WORKSPACE/run_slam.sh${NC}"
echo ""
echo "After copying files and building, you can run:"
echo "  $WORKSPACE/run_slam.sh"