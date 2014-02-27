quadrotorSimulator
==================
Author: Joe Polin
Email: joepolin111@gmail.com


Simulate a Quadrotor Nano+ for MEAM 620. Includes dynamics, path planning, trajectory generation, control, and visualization.

The framework was developed and provided by the teaching staff for MEAM 620 (Advanced Robotics) at the University of Pennsylvania. 


----------

To run the simulation,

Edit the runsim.m file, and change the the value of 'nmap' according to which map you would like to see. A brief summary of each map is below:

0-empty map, no obstacles.

1-A lot of obstacles

2-Two big blocks

3-A number of obstacles interwoven

4-Custom map with one obstacle, used more for debugging

5-Same as map1, but will show an animation of the path planning and save a video

If the code doesn't run for any reason (perhaps I inadvertently used a custom command that I wrote for my own computer), let me know and I will address it immediately. 

----------


While much of the testing code was provided by the teaching staff, I personally wrote:


Phase 1:

load_map.m--Load a text file specifying a map and obstacles, and discretize it according to user input

collide.m--Check whether a certain point collides with obstacles on a given map

dijkstra.m--Path planning using dijkstra, A*, and greedy search

plot_path--Given a map and path, provide visualization of desired path


Phase 2:

controller.m--A attitude and position controller for a Nano+ quadrotor


Phase 3:

trajectory_generator.m--Given a desired path, convert the path into a trajectory with position, velocity, and acceleration defined for every time step. Also, perform trajectory smoothing for a more flyable trajectory.


Extras (not required):

animate_frame.m--Used when nmap==5, records the frame and changes the view.

testtraj.m--Helps visualize the trajectory (analogous to plot_path.m)
