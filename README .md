
# Perception Fusion Module

### Contributor: Parth Pahinkar (par8824s@hs-coburg.de)

## 📚 Table of Contents
- [🧾 Overview](#🧾-overview)
- [🏗️ Component Architecture](#🏗️-component-architecture)
- [📡 ROS2 Topics](#📡-ros2-topics)
- [⚙️ Component Functionalities](#⚙️-component-functionalities)
- [🔧 Dependencies](#-dependencies)
- [🛠️ Installation & Setup](#🛠️-installation--setup)
- [🧪 Interface Test Procedure](#🧪-interface-test-procedure)

## 🧾 Overview
The **Perception Fusion Module** processes input data from object detections, depth images, and vehicle pose to accurately estimate and publish the positions of detected objects in a global frame. These positions are then used for visualization in RViz, where the objects are displayed at their correct locations with proper class differentiation.

## 🏗️ Component Architecture
![ Block Diagram](resource/component_architecture.jpeg)

## 📡 ROS2 Topics

| IN/Out | Topic Name                                 | Message Type                   | Description                                                      |
|--------|---------------------------------------------|--------------------------------|------------------------------------------------------------------|
|  Input  | `/camera/camera/depth/image_rect_raw`       | `sensor_msgs/Image`            | Depth image used to estimate distance to detected objects.       |
|  Input  | `/detectnet/detections`                     | `vision_msgs/Detection2DArray` | Object detector bounding boxes and class predictions.            |
|  Input  | `/odom`                                     | `nav_msgs/Odometry`            | Current position and orientation for global frame transformation.|
|  Output | `/fused_objects_out`                        | `vx_custom_msgs/fusedObjects`  | Final fused real-world objects with position and class data.     |
|  Output | `/detected_markers`                         | `visualization_msgs/MarkerArray` | Markers for RViz visualization of detected objects.              |

## ⚙️ Component Functionalities

The **Perception Fusion Module** integrates information from the object detection system, depth camera, and localization system to generate globally referenced object positions.

### Data Inputs
- **Object Detections** (`/detectnet/detections`):  
  Provides bounding box center coordinates `(u, v)` in **pixels** and object class labels.

- **Depth Image** (`/camera/camera/depth/image_rect_raw`):  
  Provides depth values in **millimeters (mm)** for each pixel.

- **Vehicle Odometry** (`/odom`):  
  Provides vehicle position and orientation in the **ENU** coordinate frame.

### Data Outputs
- **Fused Objects** (`/fused_objects_out`):  
  Final output with globally referenced object positions and class data, published as `vx_custom_msgs/fusedObjects`.

- **Detected Markers** (`/detected_markers`):  
  RViz markers corresponding to the detected objects, published as `visualization_msgs/MarkerArray` for visualization.


### Perception Fusion Functionality

The **Perception Fusion Node** subscribes to the following topics to gather necessary data:

1. **Object Detections** (`/detectnet/detections`):  
   This topic provides object detection bounding boxes and class predictions for objects detected by the camera. The node processes this information to identify objects in the scene.

2. **Depth Image** (`/camera/camera/depth/image_rect_raw`):  
   The depth image is used to estimate the distance to detected objects, allowing the node to convert 2D pixel data into 3D space coordinates.

3. **Vehicle Odometry** (`/odom`):  
   The vehicle’s pose and orientation are used to compute transformations between the camera, vehicle frame, and the global frame (ENU). This allows the node to accurately position the detected objects in global space.

### After receiving data from these topics:

Once the Perception Fusion Node receives the data from the subscribed topics, it processes the information through the following steps:

1. **Pixel → Camera Frame Conversion**:  
   The 2D pixel coordinates of the detected objects are converted into 3D camera coordinates using the camera's intrinsic parameters (focal length, optical center).

2. **Camera → Base Link Transformation**:  
   The 3D object positions in the camera frame are transformed into the vehicle's base link frame using fixed extrinsic parameters for the camera's relative position to the vehicle.

3. **Vehicle → map Frame Transformation**:  
   The positions are further transformed from the vehicle frame to the map frame. 

### Finally, the node publishes the following topics:

1. **Fused Objects** (`/fused_objects_out`):  
   The final output topic where the node publishes fused 3D object positions in the global ENU frame along with object classes, IDs, and confidence scores. This information is intended for use by downstream systems, such as the V2X transmitter for CPM (Collective Perception Message) generation.

2. **Detected Markers** (`/detected_markers`):  
   The node also publishes RViz markers that represent the detected objects in the 3D map. These markers are visualized as cuboids with colors corresponding to object classes, allowing users to visualize the detected objects in real-time in the RViz environment.

## 🔧 Dependencies

- **Car Description Module**: This is used to load the ego vehicle and model city map for visual representation.
  
## 🛠️ Installation & Setup

1. **Build the package:**
```bash
colcon build --packages-select sensor_fusion_cpm


Source the workspace:

source install/setup.bash


Run the Perception Fusion Node:

ros2 run sensor_fusion_cpm perception_fusion_node


Launch the Camera:

ros2 launch realsense_camera rs_launch.py


Launch the Object Detection:

ros2 launch ros2_deep_learning detectnet.ros2.launch input:=/camera/camera/colcon/image_raw


Launch the Localization Node:

ros2 run localization_ekf localization_node


Launch the Car Description Module (Ego Vehicle and Model City Map):

ros2 launch car_description_module load_ego_vehicle_and_map.launch.py

🧪 Interface Test Procedure

Source the workspace:

source install/setup.bash


Check for available topics:

ros2 topic list


Echo the fused objects output:

ros2 topic echo /fused_objects_out


