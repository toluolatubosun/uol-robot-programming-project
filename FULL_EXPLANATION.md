# Emergency Equipment Inventory System

## Summary
Autonomous mobile robot system for detecting and geo-locating emergency equipment (fire extinguishers, first aid kits) in indoor environments. Uses SLAM for mapping, m-explore for autonomous frontier-based exploration, YOLOv8 for object detection, and Nav2 for navigation. Features multi-angle verification, spatial clustering for de-duplication, and comprehensive statistical reporting across multiple world configurations.

## Setup

### Prerequisites
- Docker with VS Code Dev Containers extension
- NVIDIA GPU (for YOLO training and inference) (Optional)

### Installation
1. Open project in VS Code
2. Reopen in Dev Container (Ctrl+Shift+P → "Reopen in Container")
3. Post-create script automatically:
   - Installs ROS2 Humble packages
   - Builds workspace with `colcon build`
   - Pulls custom m-explore fork (https://github.com/robo-friends/m-explore-ros2)

## Project Structure
```
results/
	exploration/                                     # Exploration and detection results
		World_A/
			trial_1.json                                 
			trial_x.json                                
		World_B/
			trial_1.json
			trial_x.json                                 
	maps/                                            # Final SLAM maps per trial
		World_A/
			trial_1.pgm                                  # Map image (grayscale occupancy grid)
			trial_1.yaml                                 # Map metadata (resolution, origin, thresholds)
			trial_x.pgm/yaml                             
		World_B/
			trial_1.pgm
			trial_1.yaml
			trial_x.pgm/yaml                             # Trials 2-4

scripts/
	final_aggregation.py                           # Aggregates statistics across all worlds
	map_coverage.py                                # Calculates map coverage percentage from .pgm files, and updates the results JSON files
	trials_aggregation.py                          # Aggregates statistics across multiple trials for each world

src/
	john_bot/
		config/                                          # Configuration files
			explore_params.yaml                            # M-explore frontier exploration parameters
			mapper_params_online_async.yaml                # SLAM toolbox online async configuration
			nav2_params.yaml                               # Nav2 navigation stack parameters
			navigation.rviz                                # RViz visualization configuration
		data/
			worlds_description.json                        # Ground truth object positions for validation
		john_bot/                                        # Python ROS2 nodes
			exploration_monitor.py                         # Monitors exploration status, saves maps, tracks coverage
			object_detection.py                            # YOLOv8 detection with 3D positioning
			results_manager.py                             # Spatial clustering, JSON output, ground truth comparison
			verification_controller.py                     # Multi-angle verification navigation
		launch/
			john_bot_exploration.launch.py                 # Launches exploration monitor and verification controller
			john_bot.launch.py                             # Launches Gazebo, world, robot, object detection node, and, results manager node
		models/                                          # Gazebo 3D models
			Fire Extinguisher/
			First Aid Kit/
		worlds/                                          # Gazebo world files
			World_A.world                                  # Maze layout
			World_B.world                                  # Four-room layout
		yolo_model_weights/
			combined_dataset_best.pt                       # Trained YOLOv8m model (95.6% mAP@0.5)

	m-explore-ros2/                                     # Custom m-explore fork for ROS2 Humble
		...

	yolo_model_training/
		combined/                                        # Combined dataset training
			combined_dataset/                              # Merged fire extinguisher + first aid kit datasets
			runs/                                          # Training outputs
			combine_datasets.py                            # Script to merge individual datasets
			train_model.py                                 # YOLO training script
		fire_extinguisher/                               # Individual fire extinguisher training
			fire_extinguisher_dataset/                     # Downloaded Roboflow dataset
			runs/                                          # Training outputs
			download_dataset.py                            # Roboflow dataset download script
			train_model.py                                 # YOLO training script
		first_aid_box/                                   # Individual first aid kit training
			first_aid_box_dataset/                         # Downloaded Roboflow dataset
			runs/                                          # Training outputs
			download_dataset.py                            # Roboflow dataset download script
			train_model.py                                 # YOLO training script
		.env                                             # Roboflow API keys
```

## Launch Instructions

You need 2 terminals to run the system:

**Terminal 1**
<br>
This command
- Launches the robot in the specified world, for a certain trial
- It starts the object dectection node, that passes camera images to yolo for object detection. A camera view with object detection is available
- It starts the result manager node, which records the: world, trials, time taken and detections in a JSON file
- NOTE: You should update the `world_id` and `trial_id`
```bash
ros2 launch john_bot john_bot.launch.py world_id:=A trial_id:=1
```

**Terminal 2**
<br>
This command
- Launches Rviz for visualization of SLAM mapping, Global and Local costmaps and robot navigation
- It starts the exploration monitor node, which periodically saves the SLAM map and tracks exploration status
- It starts the verification controller node, which makes the robot to revisit detected objects from multiple angles for improved detection accuracy. This happens immediately after the exploration is complete.
```bash
ros2 launch john_bot john_bot_exploration.launch.py use_sim_time:=true
```

## System Process Overview

When launching the system with both the robot launch and exploration launch commands, Gazebo opens with the LIMO robot spawned in the specified world environment. A camera window displays the robot's RGB feed with real-time YOLOv8 object detection visualized with bounding boxes, while RViz provides visualization of SLAM mapping, exploration frontiers, and navigation paths.

The robot uses m-explore's frontier-based navigation to autonomously explore the environment, performing exploration and object detection simultaneously. As the robot moves, it builds a map using slam_toolbox while continuously scanning for emergency equipment with the RGB-D camera. Detected objects are processed through spatial clustering to eliminate duplicates, with unique detections accumulated in the results manager.

When exploration completes (all frontiers exhausted), the robot autonomously returns to its starting position and the exploration monitor saves the final SLAM map. At this point, the verification phase automatically begins. The verification controller loads all detections from the JSON file, calculates an optimized route using greedy nearest-neighbor algorithm to minimize travel distance, then systematically revisits each detection location.

At each location, the robot attempts verification from multiple viewing angles (0°, 120°, 240°) to achieve full coverage. It calculates safe standoff positions using costmap collision checking, navigates to each viewpoint using Nav2, and collects detections for confirmation. Objects that cannot be re-detected from any angle are removed as false positives. After completing all verifications, the robot returns to origin, and the updated results are saved with verification timing metadata.

## Nodes Overview

### object_detection.py

**Purpose**: YOLOv8-based object detection with 3D positioning in global frame

The detection node loads the trained YOLOv8m model (`combined_dataset_best.pt`) with GPU acceleration when available, falling back to CPU if necessary. It subscribes to both RGB (`/limo/depth_camera_link/image_raw`) and depth (`/limo/depth_camera_link/depth/image_raw`) camera feeds, running YOLO inference with a 0.45 confidence threshold on incoming RGB frames. It opens a CV2 windows to display the camera feed with bounding boxes and labels overlaid for detected objects.

For 3D positioning, the node implements a multi-step pipeline: detected objects from RGB bounding boxes are projected to the camera frame using depth data and camera intrinsics, then transformed to the global `odom` frame using TF2. This produces accurate 3D positions for each detection in the map coordinate system.

The node publishes detections on two topics: `/detected_objects` (Detection3DArray) containing full detection metadata including class IDs, confidence scores, and poses, and `/detected_objects_poses` (PoseArray) for visualization in rviz. Detection can be dynamically enabled or disabled during operation through the `enable_detection` service (SetBool), allowing control over when object detection runs. By default, detection is enabled on startup.

### results_manager.py

**Purpose**: Detection aggregation with spatial clustering for de-duplication and recording in JSON file

The results manager receives detections from the object detection node from the `/detected_objects`  topic and maintains a list of unique objects using spatial clustering. When a new detection arrives, the node calculates its 3D Euclidean distance to all existing detections. If the distance is below the clustering threshold (2.0m by default), it's considered a duplicate of an existing object; otherwise, it's added as a new unique detection. For duplicates, the node updates the existing entry with the higher-confidence detection, ensuring the best observation is retained. The de-duplicated list is published to `/deduplicated_objects_poses` for rviz visualization.

The node subscribes to the exploration status topic (`/explore/status`) using TRANSIENT_LOCAL QoS durability, allowing it to receive the current exploration state even when joining late. When exploration completes and the robot returns to origin, the node performs a final save of detections and stops periodic updates, preparing for the verification phase.

Results are saved periodically (every 10 seconds) to a JSON file structured as `results/exploration/World_{world_id}/trial_{trial_id}.json`, containing detection metadata including class IDs, 3D positions, confidence scores, and timestamps. The node also broadcasts trial configuration (world_id, trial_id) on `/trial_config` with TRANSIENT_LOCAL QoS so other nodes can access the current trial parameters.

### exploration_monitor.py

**Purpose**: Periodic map saving during exploration with final map capture on completion

The exploration monitor tracks the exploration process and saves SLAM maps at regular intervals and upon completion. It subscribes to trial configuration (`/trial_config`) and exploration status (`/explore/status`) using TRANSIENT_LOCAL QoS durability, allowing it to receive the current state even when starting after other nodes.

During active exploration, the node periodically triggers map saves (every 30 seconds by default) using the `map_saver_cli` tool from nav2_map_server as a subprocess. This creates timestamped map snapshots throughout the exploration process, providing data preservation in case of unexpected failures. When the exploration status indicates the robot has returned to origin, the node performs a final map save and stops periodic saving, marking the exploration phase as complete.

Map files are saved to a directory structure matching the results_manager convention: `results/maps/World_{world_id}/trial_{trial_id}.pgm/yaml`. This naming alignment ensures consistency across the results pipeline and enables the aggregation scripts to locate maps by world and trial identifiers. The saved maps contain the full SLAM-generated occupancy grid used for coverage calculation during post-processing.

### verification_controller.py

**Purpose**: Multi-angle re-verification of detected objects with Nav2 navigation and false positive removal

The verification controller monitors exploration status and automatically triggers the verification phase when the robot returns to origin. It loads the detection results from the JSON file generated during exploration, then optimizes the verification route using greedy nearest-neighbor algorithm to visit objects in order of proximity, minimizing total travel distance.

For each detection, the node attempts verification from multiple viewing angles (0°, 120°, 240°) to achieve 360-degree coverage. Before navigating, it calculates safe standoff positions at the configured distance (1.5m default) using costmap collision checking with robot footprint clearance. If the primary angle is blocked by obstacles, the node tries fallback angles (±45°, ±90°) until finding a valid viewpoint. Navigation is performed using the Nav2 action client, with detection dynamically disabled during movement and re-enabled only at standoff positions to avoid spurious detections.

At each standoff location, the node collects detections for a configured duration (5 seconds default), then matches them against the expected object position and class within a tolerance threshold (1.0m). If an object is confirmed from any viewing angle, it's retained; otherwise, it's removed as a false positive. After completing all verifications, the node updates the JSON file with verified detections, verification start and end times, verification duration, and returns the robot to its captured initial pose.

## YOLO Model Training

### Individual Model Training (Fire Extinguisher or First Aid Kit)

1. **Download dataset**<br>:
From project root, run the below command to download either the fire extinguisher or first aid kit dataset from Roboflow:
```bash
python3 yolo_model_training/<first_aid_box or fire_extinguisher>/download_dataset.py
```

2. **Optional**: Rename downloaded folder if needed

3. **Configure training** in `train_model.py`:
   - Epochs (e.g., 75)
   - Model size (yolov8n, yolov8s, yolov8m, yolov8l, yolov8x)
   - Image size (default 640)

4. **Run training**:
From project root, run the below command to train either the fire extinguisher or first aid kit model:
```bash
python3 yolo_model_training/<first_aid_box or fire_extinguisher>/train_model.py
```

5. **Deploy model**:
   - Training outputs: `runs/detect/train/weights/best.pt` and `last.pt`
   - Move `best.pt` to `john_bot/yolo_model_weights/`
   - Rename to `combined_dataset_best.pt` (required filename for object detection node)

### Combined Model Training (Both Classes)

1. **Ensure individual datasets downloaded** (see above)

2. **Configure dataset paths** in `combine_datasets.py`:
   - Verify folder names match downloaded datasets
   - Ensure paths to individual datasets are correct

3. **Combine datasets**:
Run the below command from project root to merge the two datasets into a combined dataset:
```bash
python3 yolo_model_training/combined/combine_datasets.py
```

4. **Configure training** in `train_model.py`:
   - Epochs (e.g., 75)
   - Model size (yolov8m recommended)
   - Image size (default 640)

5. **Run training**:
```bash
python3 yolo_model_training/combined/train_model.py
```

6. **Deploy model**:
   - Same as individual model deployment

## Running Results Aggregation Scripts

Scripts must be run **in order** from the project root directory after completing all trials.

### Step 1: Calculate Map Coverage

Calculates coverage percentage from `.pgm` map files and adds to trial JSON files.
```bash
python3 scripts/map_coverage.py
```

### Step 2: Aggregate Per-World Statistics

Calculates statistics per world with mean ± std across trials.
```bash
python3 scripts/trials_aggregation.py
```

It outputs:
- `results/summary/World_A_summary.csv` - World A statistics
- `results/summary/World_B_summary.csv` - World B statistics
- Each CSV contains: individual trial metrics + aggregate row (mean ± std)

### Step 3: Final Overall Aggregation

Calculates overall statistics across all worlds and trials.
```bash
python3 scripts/final_aggregation.py
```

It outputs:
- `results/summary/final_aggregate.csv` - Overall statistics with mean ± std across all 8 trials

## Attributions

### 3D Models
- First aid kit models generated using AI (Microsoft Copilot 3D)
- Fire extinguisher model sourced from Gazebo Fuel: [Gazebo Fuel - Fire Extinguisher Model](https://app.gazebosim.org/OpenRobotics/fuel/models/Fire%20Extinguisher)
- Models were converted from GLB to Gazebo-compatible DAE format using [Convert 3D - GLB to DAE Converter](https://convert3d.org/)

### YOLO Training Datasets
- **Fire Extinguisher**: [Roboflow Universe - Fire Extinguisher Detection](https://universe.roboflow.com/final-year-project-jau7b/fire-extinguisher-detection-z1wqp)
- **First Aid Kit**: [Roboflow Universe - First Aid Kit Detection](https://universe.roboflow.com/firstaidkit-gdncc/firstaidkit)

### M-Explore Fork
- Custom fork of ROS2 m-explore was used; https://github.com/robo-friends/m-explore-ros2
- Modifications included:
  - Added exploration status tracking and publishing to ROS2 topic
  - Status publishing made use of TRANSIENT_LOCAL message durability to ensure late-joining nodes receive the last status
- Pull request submitted upstream for status publishing feature