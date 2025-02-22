This workspace involves two packages. pick_place_pkg which will launch a open manipulator x in pybullet, and the omx_pick_place package has a cpp file to perform pick and place operation to stack two cylinders over one other. To run the stacking program, follow the following guidlines:

First terminal:
	cd omx_pick_and_place_ws/
	source install/setup.bash
	ros2 run pick_place_pkg omx_sim.py 

Second terminal:
	cd omx_pick_and_place_ws/
	source install/setup.bash
	ros2 run omx_pick_place omx_pick_place 

Youtube demonstration link: https://youtu.be/4GbxXxYktpY