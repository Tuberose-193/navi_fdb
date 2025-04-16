# colcon build --symlink-install

cmds=(
	
	
	
        
	
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py\
		use_sim_time:=true rviz:=true
	"
	
	
        "ros2 launch kiss_matcher_ros visualizer_launch.py \
		use_sim_time:=true rviz:=true
	"
	#"  ros2 launch remove_pointcloud remove_pointcloud.launch.py \
	   
	#"
	"  ros2 launch kiss_backend kiss_backend.launch.py \
	   use_sim_time:=true rvuz2:=true
	"
	
	"ros2 launch point_lio mapping_mid360.launch.py \
	use_sim_time:=true "
	
    # "ros2 launch fast_lio mapping_mid360.launch.launch.py \
	# 	use_sim_time:=true rviz:=false
	# "
	# "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py \
	# 	use_sim_time:=true
	# " 
	# "ros2 launch icp_localization_ros2 bringup.launch.py \
	# 	use_sim_time:=true
	# "
)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;source /usr/share/gazebo/setup.sh;$cmd;exec bash;"
	sleep 0.2
done
