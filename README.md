# RRT-Multi-agent-Trajectory-Planning
Multi-turtle bot navigation in Gazebo with  a decentralized iterative algorithm based on single-agent dynamic RRT*

Video result of Gazebo simulation: https://youtu.be/C8Iq0kzaR6g

Documentation: https://github.com/RaviTeja163/RRT-Multi-agent-Trajectory-Planning/blob/main/README.txt

Abstract: I address the problem planning trajectories of multirobot systems in an environment, while avoiding obstacles as well as other robots, adhering to a pre-defined clearance from each other. The proposed method uses the prune-and-graft approach for re-planning, which requires a maximum of n iteration for each robot to generate its collision-free trajectory. The entire method is decentralized with limited sharing of information (only pertaining to the shared space). I used a dynamic RRT* algorithm for this purpose undertaking a single-agent planning method.

Full Report: https://github.com/RaviTeja163/RRT-Multi-agent-Trajectory-Planning/blob/main/Report.pdf

Reference: P. Verbari, L. Bascetta, and M. Prandini, “Multi-agent trajectory planning: A decentralized iterative algorithm based on single-agent dynamic
rrt,” in 2019 American Control Conference (ACC). IEEE, 2019, pp.
1977–1982
