search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=left_arm.srdf
robot_name_in_srdf=left_arm
moveit_config_pkg=left_arm_moveit_config
robot_name=left_arm
planning_group_name=left_arm
ikfast_plugin_pkg=left_arm_left_arm_ikfast_plugin
base_link_name=arm_L_link0
eef_link_name=arm_L_link6
ikfast_output_path=/home/hp/nachi_ws/src/ikfast folder/left_arm_left_arm_ikfast_plugin/src/left_arm_left_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
