ugv.xacro
- the master file that the launch file should point to to spawn the robot
- includes physical links, like the chassis, wheels, sensors 
- includes joints, like how the wheels are attached
- includes the other description files

materials.xacro
- the aesthetics
- defines colors using RGBA numbers 

ugv.gazebo
- whereas the typical urdf files only describe shape and size, this file describes the simulation physics
- can specify friction coefficients, stiffness, code that drives the robot/helps sensors scan

ugv.trans 
- relates the joints to actuators so the rover can move in gazebo
