# ROS-Term-Carry-Order-and-Place-

Subsumption algorithm on ros that has three levels

sudo apt-get install ros-kinetic-joy  
edit ~/.bashrc, add "source /home/..../rosws/devel/setup.bash" to the end of the .bashrc file.  
close the terminal and reopen the terminal   
run roscore  

To test with one robot go ros-ws-one :    
catkin make    
. ~/ros-ws-one/devel/setup.bash  
run vrep, open scenes/term_project_2018_no_human_single_robot.ttt  
roslaunch my_subsumption final_project.launch  

To test with multi robot go ros-ws-two :  
catkin make  
. ~/ros-ws-two/devel/setup.bash   
run vrep, open scenes/term_project_2018_no_human.ttt  
roslaunch my_subsumption final_project.launch  
