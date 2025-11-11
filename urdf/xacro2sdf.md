source install/setup.bash 
xacro ugv.xacro > ugv.urdf
gz sdf -p ugv.urdf > ugv.sdf