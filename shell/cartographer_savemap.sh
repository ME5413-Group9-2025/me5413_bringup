rosservice call /finish_trajectory 0
rosservice call /write_state "filename: '/home/liuxiao/Desktop/mymap.pbstream' include_unfinished_submaps: false"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/liuxiao/Desktop/mymap -pbstream_filename=/home/liuxiao/Desktop/mymap.pbstream -resolution=0.05