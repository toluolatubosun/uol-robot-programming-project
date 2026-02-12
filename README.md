# Emergency Equipment Inventory System

## Summary

The robot autonomous detects and geo-locates emergency equipment (fire extinguishers, first aid kits) in an environment using SLAM mapping, m-explore (frontier-based exploration), YOLOv8, and Nav2. The robot simultaneously explores and detects objects, applying spatial clustering for de-duplication. Upon exploration completion, it autonomously returns to origin and initiates verification. The verification involves revisiting each detection and reverifying from multiple angles, an optimized route is computed, and false positives are removed if verification fails. The system generates comprehensive statistical reports in JSON format and saves the map. Scripts are provided to aggregate JSON results into CSV format.

## Launch Instructions

Open the project in a DevContainer, then run the commands:

**Terminal 1: Robot, Object Detection & Results Manager**

Launches Gazebo with robot in specified world, object detection node (camera view with YOLO bounding boxes), and results manager node for JSON recording.

**Update `world_id` (A or B) and `trial_id` (1-4) parameters:**
```bash
ros2 launch john_bot john_bot.launch.py world_id:=A trial_id:=1
```

**Terminal 2: Exploration & Verification**

Launches RViz, exploration monitor node (periodic map saves), and verification controller node (automatic multi-angle verification after exploration).
```bash
ros2 launch john_bot john_bot_exploration.launch.py use_sim_time:=true
```

**Results Location:**
- Detection results: `results/exploration/World_{world_id}/trial_{trial_id}.json`
- Final map: `results/maps/World_{world_id}/trial_{trial_id}.pgm/yaml`

**Post-Trial: Running Aggregation Scripts**

After completing all trials, run these scripts **in order** from project root:

1. Calculate map coverage and update JSON files:
```bash
python3 scripts/map_coverage.py
```

2. Generate per-world statistics (outputs CSV files in `results/summary/`):
```bash
python3 scripts/trials_aggregation.py
```

3. Generate overall aggregate statistics:
```bash
python3 scripts/final_aggregation.py
```