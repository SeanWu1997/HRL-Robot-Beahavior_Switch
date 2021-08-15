# HRL-Robot-Beahavior_Switch
*Base_action(SAC): Training the robot local navigation action.
*Area_model(DQN): Loading pre-trained base_action model to train the global planner.
*Area_switch(DQN): Training high-level behavior switcher.
## Steps
1. $ cd ~/catkin_ws/src
2. $ git clone https://github.com/SeanWu1997/HRL-Robot-Beahavior_Switch.git
## Remark
* fusion_msg
: Define the format between sending and receiving information.
* fusion_layers
: Acting as the data receiving end and plug-in into Costmap_2D.
* fusion_data_sim
: Acting as a data publishing node, the template provided to users can be modified according to any situation.

