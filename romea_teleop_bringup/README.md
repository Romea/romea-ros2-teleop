# 1) Overview #

The romea_teleop_bringup package provides  : 

 - launch files able to launch ros2 romea teleop drivers according meta-description of mobile base (see [romea mobile base stack]()), meta-description of joystick (see (romea joystick stack)[]) and teleop configuration file  provided by user (see next section for telop configuration file overview). There is four teleop drivers into romea teleop drivers : 
    - skid_steering_teleop_node 
    - omni_steering_teleop_node
    - one_axle_steering_teleop_node
    - two_axle_steering_teleop_node

   It is possible to launch any of them via command line : 

    ```console
    ros2 launch romea_gps_bringup gps_driver.launch.py robot_namespace:=robot base_meta_description_file_path:=/path_to_file/base_meta_description_file.yaml 
    joystick_meta_description_file_path:=/path_to_file/joystick_meta_description_file.yaml
    teleop_configuration_file_path:= /path_to_file/teleop_configuration_file.yaml
    ```
   where :

   - *robot_namespace* is the name of the robot 
   - *base_meta_description_file_path* is the absolute path of mobile base meta-description file    
   - *joystick_meta_description_file_path* is the absolute path of joystick meta-description file    
   - *teleop_configuration_file_path* is the absolute path of teleop configuration file

   The telop node will be selected automatically according the mobile base meta-description and teleop configuration    

 - a python module able to load and parse joystick configuration file

# 2) Teleop configurations 

As seen below teleop configuration file is a yaml file constituted by three items. The first item called cmd_output describes the configuration of the output of the teleoperation nodes where the type of output message and its priority can be specified. The second item called cmd_range contains the maximum values for each component of the control messages. Some of these maximum values are optional and can be deduced from the mechanical limits of the robot, others like the maximum linear or rotational speeds of the robot are required and defined for slow and turbo modes. These values are in any case clamped according to the mechanical limits of the robot. The last item called joystick_mapping is optional and gives the mapping between the different components of the command messages and the buttons and axes of the joystick. Default mappings can be found in the config directory of the romea_teleop_description package.

Example of configuration files for  :
- one_axle_steering_teleop_node 
```yaml
 cmd_output: # output configuration
   message_type: romea_mobile_base_msgs/OneAxleSteeringCommand # type of msg published by teleop
   message_priority: 100 # priority for cmd_mux
 cmd_range: # cmd range configuration
   maximal_linear_speed: 
    slow_mode: 1.0 # m/s clamped according maximal wheel speed (see robot configuration)
    turbo_mode: 2.0 # m/s clamped according maximal wheel speed (see robot configuration)
   #maximal_steering_angle:  # optional deduced or clamped according mechanical limits (see robot configuration)
```

- two_axle_steering_teleop_node 
```yaml
 cmd_output: # output configuration
   message_type: romea_mobile_base_msgs/TwoAxleSteeringCommand # type of msg published by teleop
   message_priority: 100 # priority for cmd_mux
 cmd_range: # cmd range configuration
   maximal_linear_speed: 
    slow_mode: 1.0 # clamped according maximal wheel speed (see robot configuration)
    turbo_mode: 2.0 # clamped according maximal wheel speed (see robot configuration)
   #maximal_front_steering_angle: # optional deduced or clamped according mechanical limits (see robot configuration)
   #maximal_rear_steering_angle: # optional deduced or clamped according mechanical limits (see robot configuration)
```
- skid_steering_teleop_node 
```yaml
cmd_output: # output configuration
  message_type: romea_mobile_base_msgs/SkidSteeringCommand # type of msg published by teleop
  message_priority: 100 # priority for cmd_mux
cmd_range: # cmd range configuration
  maximal_linear_speed:
    slow_mode: 1.0 # clamped according maximal wheel speed (see robot configuration)
    turbo_mode: 2.0 # clamped according maximal wheel speed (see robot configuration)
  maximal_angular_speed:
    slow_mode: 0.5 # clamped according maximal wheel speed and track (see robot configuration)
    turbo_mode: 1.0 # clamped according maximal wheel speed and track (see robot configuration)
```
- omni_steering_teleop_node 
```yaml
cmd_output: # output configuration
  message_type: romea_mobile_base_msgs/OmniSteeringCommand # type of msg published by teleop
  message_priority: 100 # priority for cmd_mux
cmd_range: # cmd range configuration
  maximal_linear_speed:
    slow_mode: 1.0 # clamped according maximal wheel speed (see robot configuration)
    turbo_mode: 2.0 # clamped according maximal wheel speed (see robot configuration)
  maximal_lateral_speed:
    slow_mode: 1.0 # clamped according maximal wheel speed (see robot configuration)
    turbo_mode: 1.0 # clamped according maximal wheel speed (see robot configuration)
  maximal_angular_speed:
    slow_mode: 0.5 # clamped according maximal wheel speed, track and wheelbase (see robot configuration)
    turbo_mode: 1.0 # clamped according maximal wheel speed, track and wheelbase (see robot configuration)
```
