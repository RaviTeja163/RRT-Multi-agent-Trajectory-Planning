Project Title -
Multi-agent trajectory planning: A decentralized iterative algorithm based on single-agent dynamic RRT*


Author - Ravi Teja Alapati


Dependencies -
Python 2.7
ubuntu 16.04
ROS Kinetic


Libraries -
os
shutil
math
random
numpy
matplotlib.cm
matplotlib.pyplot
scipy.interpolate


Instructions to run the python simulation-
Go to the directory where code is present and run the following command:
python main.py

The final paths of agent 1, agent 2 and agent 3 are saved in final_traj1.txt, final_traj2.txt, final_traj3.txt.


Instructions to run the ROS Gazebo simulation-
Launch the all_traj.launch file using the following command:
roslaunch multi-turtle all_traj.launch

The robots are spawned and moved to their goal positions in Gazebo.




