colcon build --symlink-install

source install/setup.bash

ros2 launch john_bot john_bot.launch.py world_id:=A trial_id:=2

ros2 launch john_bot john_bot_exploration.launch.py use_sim_time:=true

# CHECK THE `FULL_EXPLANATION.md` FOR MORE DETAILS