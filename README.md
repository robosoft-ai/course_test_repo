# course_test_repo

## Let's Get Started
We begin by cloning isaac_ros_common and nova_carter repos to the src folder of our local workspace. My local workspace is ~/workspace/humble_ws
 ```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/realsense-ros
 ```
## Start the IsaacROSDev Container (from the workspace...)
 ```
./src/isaac_ros_common/scripts/run_dev.sh  -d  ~/workspace/humble_ws/
 ```
Then run ls in the terminal to confirm that isaac_ros-dev is set to your host host workspace.    
  
Alternatively, you can run this one from inside workspace/src/isaac_ros_common/scripts. The -d command explicity sets the workspace to workspaces/isaac_ros-dev inside the IsaacROSDev container.
```
./run_dev.sh -d  ~/workspace/humble_ws/
```
## Installations onto the Container....
First, lets get updated... 
 ```
sudo apt-get update  
rosdep update
 ```
We'll need curl too to for later when we download assets into the isaac_ros_assets folder...
 ```
sudo apt-get install -y curl jq tar
 ```
### Install Jetson Stats  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats/index.html#build-package-name)
 ```
sudo apt-get install -y ros-humble-isaac-ros-jetson-stats
 ```

### Install isaac_ros_object_detection and other perception packages from Debian... (optional)
We'll start with pointcloud_to_laserscan...  
 ```
sudo apt-get install -y ros-humble-pointcloud-to-laserscan
 ```
### Install isaac_ros_rtdetr  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_rtdetr/index.html#build-package-name)
 ```
sudo apt-get install -y ros-humble-isaac-ros-rtdetr
 ```
#### Download the isaac_ros_rtdetr assets | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_rtdetr/index.html#download-quickstart-assets)

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_rtdetr"
NGC_RESOURCE="isaac_ros_rtdetr_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=2
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
 ```
Now download the pre-trained Nvidia SyntheitcaDETR models...
 ```
 mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/synthetica_detr/versions/1.0.0_onnx/files/sdetr_grasp.onnx'

```
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/synthetica_detr/versions/1.0.0_onnx/files/sdetr_amr.onnx'
```
Then we'll convert the encrypted model (.etlt) to a TensorRT engine plan and drop it in the isaac_ros_assets/models/synthetica_detr folder...
```
/usr/src/tensorrt/bin/trtexec --onnx=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.onnx --saveEngine=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan
```
```
/usr/src/tensorrt/bin/trtexec --onnx=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_amr.onnx --saveEngine=${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_amr.plan
```

Then get back to the workspace...  
```
cd /workspaces/isaac_ros-dev/
```
and install this package so you can test your vision pipeline later...  
```
sudo apt-get install -y ros-humble-isaac-ros-examples
```
To test this section use this [IsaacSim Tutorial](https://nvidia-isaac-ros.github.io/concepts/object_detection/rtdetr/tutorial_isaac_sim.html)


## Assemble the Workspace

### Clone the Application Repos into the src folder...  
 ```
git clone https://github.com/robosoft-ai/SMACC2.git  
git clone https://github.com/robosoft-ai/nova_carter_sm_library
 ```

### Get a Nav2 release into the workspace src folder...
I like to work from Nav2 releases. Download and unzip, then drop into the src folder of your local workspace. This is currently 1.1.17
 https://github.com/ros-navigation/navigation2/releases/tag/1.1.17  

### Workspace Operations

Create a file called .isaac_ros_common-config with the following context:
 ```
cd src/isaac_ros_common/scripts
 ```
 ```
echo -e "CONFIG_IMAGE_KEY=ros2_humble.nova_carter\nCONFIG_DOCKER_SEARCH_DIRS=(../../nova_carter/docker ../docker)" > .isaac_ros_common-config
 ```
Create a file called .isaac_ros_dev-dockerargs with the following context:
 ```
  echo -e "-v /etc/nova/:/etc/nova/\n-v /opt/nvidia/nova/:/opt/nvidia/nova/" > .isaac_ros_dev-dockerargs
 ```

Clone the perceptor dependency repositories using the vcstool file in the nova_carter repository:
 ```
cd /workspaces/isaac_ros-dev
vcs import --recursive src < src/nova_carter/nova_carter.repos
 ```
A ton of beta ish dependencies are added in this step to make the original perceptor demo work.  
This has now led to build problems involving building the following packages from source. So, after completing the previous vcs import command, remove the following packages form workspace/src

 ```
 REMOVE THESE PKGS FROM THE WORKSPACE/SRC FOLDER BEFORE COMPILING
isaac_ros_nitros
isaac_ros_image_segmentation
isaac_ros_dnn_inference
isaac_ros_image_pipeline
isaac_ros_freespace_segmentation
isaac_ros_map_localization
isaac_perceptor
isaac_ros_depth_segmentation
isaac_ros_dnn_stereo_depth
isaac_ros_mission_client
isaac_ros_apriltag
isaac_ros_nova
isaac_ros_compression
spatio_temporal_voxel_layer
 ```
## Build Workspace
Ok, now you're ready to compile everything...
```
source /opt/ros/humble/setup.bash
```
```
rosdep update
rosdep install --ignore-src --from-paths src -y -r
 ```
 ```
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
 ```

## Launch Application
Source the workspace...  
 ```
source install/setup.bash
 ```
```
ros2 run sm_nav2_test_7 lidar_completion.py --ros-args -r /scan_input:=/scan2 -r /scan_output:=/scan
```
 ```
ros2 launch sm_nav2_test_7 sm_nav2_test_7_launch.py 
 ```

## Test Commands
 ```
ros2 topic echo /detections_output
