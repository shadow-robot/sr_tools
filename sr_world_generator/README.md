# sr_world_generator

A lot of the code here is deprecated, since we changed our `.world` and `.scene` generation method. We now simply run a simulated robot as normal, rely on `gazebo2rviz` populating Rviz from Gazebo, save the world and scene files using Gazebo and Rviz's GUIs respectively, then clean the world file using [clean_world_file.launch](launch/clean_world_file.launch).
