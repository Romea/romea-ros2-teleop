# 1 Overview #

This package contains teleop nodes for skid, ackermann (one axle), omni and four wheel (two axles) steering robots. They translate data coming from joystick message into command messages supported by vehicle controllers. These nodes can interact with romea cmd_mux_node (see romea_cmd_mux package) in order to register/unregister automatically teleop output topic.

# 2 Nodes #


### 2.1 skid_steering_teleop_node ###

### 2.1.1 Subscribed Topics ###

- joy : (sensor_msgs/Joy)

  Joystick messages to be translated to velocity commands.  

### 2.1.2 Published Topics ###

- cmd_vel (geometry_msgs/Twist)

  Ros velocity messages arising from joystick commands. Available if parameter cmd_output.type is equal to geometry_msgs/Twist.

- cmd_skid_steering (romea_mobile_base_msgs/SkidSteeringCommand)

  Romea skid steering command messages arising from joystick commands. Available if parameter cmd_output.type is equal to romea_mobile_base_msgs/SkidSteeringCommand.

### 2.1.3 Parameters ###

- joystick.type (string, defaut none)

    Type of the joystick (xbox,dualshock4...), see available supported joysticks in romea_joy package

- jostick.remapping.axes.linear_speed (string, default node)

    Id of the stick used to control linear_speed, see  example configuration files 

- jostick.remapping.axes.angular_speed (string, default none)

    Id of the stick used to control angular_speed, see  example configuration files 

- joystick.remapping.buttons.slow_mode (string, default none)

    Id of the button used to slow motion mode,  see  example configuration files 

- joystick.remapping.buttons.turbo_mode (string, default none) 

    Id of the button used to turbo motion mode,  see  example configuration files 

- cmd_output.type (string, default none)

  Type of messages published by teleop node : geometry_msgs/Twist or   romea_mobile_base_msgs/SkidSteeringCommand

- cmd_output.priority(int, default -1)

  Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_skid_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.scale.maximal_linear_speed.slow_mode (double, default none)

    Output maximal linear speed when slow mode is activated   

- cmd_range.scale.maximal_linear_speed.turbo_mode (double, default none)

    Output maximal linear speed when turbo mode is activated 

- cmd_range.scale.maximal_angular_speed.slow_mode (double, default none)

    Output maximal angular speed  when slow motion mode is activated   

- cmd_range.scale.maximal_angular_speed_.turbo_mode (double, default none)

    Output maximal angular speed  when turbo motion mode is activated      


# 2.2 omni_steering_teleop_node #

### 2.2.1 Subscribed Topics ###

  - joy : (sensor_msgs/Joy)

    Joystick messages to be translated to velocity commands.  

### 2.2.2 Published Topics ###

  - cmd_vel (geometry_msgs/Twist)

    Ros velocity messages arising from joystick commands. Available if parameter cmd_output.type is equal to geometry_msgs/Twist.
    
- cmd_omni_steering (romea_mobile_base_msgs/OmniSteeringCommand)

  Romea omni steering command messages arising from joystick commands. Available if parameter cmd_output.type is equal to romea_mobile_base_msgs/OmniSteeringCommand.


### 2.2.3 Parameters ###

- joystick.type (string, defaut none)

     Type of the joystick (xbox,dualshock4...), see available supported joysticks in romea_joy package

- jostick.remapping.axes.linear_speed (string, default node)

     Id of the stick used to control linear_speed, see  example configuration files 

- jostick.remapping.axes.lateral_speed (string, default node)

     Id of the stick used to control lateral_speed, see  example configuration files 

- jostick.remapping.axes.angular_speed (string, default none)

     Id of the stick used to control angular_speed, see  example configuration files 

- joystick.remapping.buttons.slow_mode (string, default none)

     Id of the button used to slow motion mode,  see  example configuration files 

- joystick.remapping.buttons.turbo_mode (string, default none) 

     Id of the button used to turbo motion mode,  see  example configuration files 

- cmd_output.type (string, default none)

     Type of messages published by teleop node : geometry_msgs/Twist or   romea_mobile_base_msgs/OmniSteeringCommand

- cmd_output.priority(int, default -1)

     Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_omni_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.scale.maximal_linear_speed.slow_mode (double, default none)

     Output maximal linear speed when slow mode is activated   

- cmd_range.scale.maximal_linear_speed.turbo_mode (double, default none)

     Output maximal linear speed when turbo mode is activated 

- cmd_range.scale.maximal_lateral_speed.slow_mode (double, default none)

     Output maximal lateral speed when slow mode is activated   

- cmd_range.scale.maximal_lateral_speed.turbo_mode (double, default none)

     Output maximal linear speed when turbo mode is activated 

- cmd_range.scale.maximal_angular_speed.slow_mode (double, default none)

     Output maximal angular speed  when slow motion mode is activated   

- cmd_range.scale.maximal_angular_speed_.turbo_mode (double, default none)

     Output maximal angular speed  when turbo motion mode is activated      


# 2.3 one_axle_teleop_node #

### 2.3.1 Subscribed Topics ###

  - joy : (sensor_msgs/Joy)

    Joystick messages to be translated to velocity commands.  

### 2.3.2 Published Topics ###

- cmd_vel (geometry_msgs/Twist)    

    Ros velocity messages arising from Joystick commands. To ensure compatibility with ackermann controller from ros_controllers package geometry_msgs::Twist msg is used to command instead of an ackermann msg. Linear speed is set into twist.linear.x parameter and steering angle is set into twist.angular.z parameter (yes it is weird) .  Available if parameter cmd_output.type is equal to (geometry_msgs/Twist). 

- cmd_omni_steering (romea_mobile_base_msgs/AxleSteeringCommand)

  Romea omni steering command messages arising from joystick commands. Available if parameter cmd_output.type is equal to romea_mobile_base_msgs/OneAxleSteeringCommand.

### 2.4.3 Parameters ###

  - joystick.type (string, defaut none)

    Type of the joystick (xbox,dualshock4...), see available supported joysticks in romea_joy package

- jostick.remapping.axes.linear_speed (string, default node)

  Id of the stick used to control linear_speed, see  example configuration files 

- jostick.remapping.axes.stering_angle (string, default node)

  Id of the stick used to control steering_angle, see  example configuration files 

- joystick.remapping.buttons.slow_mode (string, default none)

  Id of the button used to slow motion mode,  see  example configuration files 

- joystick.remapping.buttons.turbo_mode (string, default none) 

  Id of the button used to turbo motion mode,  see  example configuration files 

- cmd_output.type (string, default none)

  Type of messages published by teleop node : geometry_msgs/Twist or   romea_mobile_base_msgs/OneAxleSteeringCommand

- cmd_output.priority(int, default -1)

  Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_one_axle_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.scale.maximal_linear_speed.slow_mode (double, default none)

  Output maximal linear speed when slow mode is activated   

- cmd_range.scale.maximal_linear_speed.turbo_mode (double, default none)

  Output maximal linear speed when turbo mode is activated 

- cmd_range.scale.maximal_steering_angle (double, default none)

  Output maximal steering angle          

# 2.4 two_axle_steering_teleop_node #

### 2.4.1 Subscribed Topics ###

  - joy : (sensor_msgs/Joy)

    Joystick messages to be translated to velocity commands.  

### 2.4.2 Published Topics ###

  - cmd_4ws (four_wheel_steering_msgs/FourWheelSteering)

    Ros velocity messages arising from joystick commands. Available if parameter cmd_output.type is equal to four_wheel_steering_msgs/FourWheelSteering.

- cmd_two_axle_steering (romea_mobile_base_msgs/TwoAxleSteeringCommand)

  Romea two axle steering command messages arising from joystick commands. Available if parameter cmd_output.type is equal to romea_mobile_base_msgs/TwoAxleSteeringCommand.

### 2.4.3 Parameters ###

- joystick.type (string, defaut none)

  Type of the joystick (xbox,dualshock4...), see available supported joysticks in romea_joy package

- jostick.remapping.axes.linear_speed (string, default node)

  Id of the stick used to control linear_speed, see  example configuration files 

- jostick.remapping.axes.front_stering_angle (string, default node)

  Id of the stick used to control front_steering_angle, see  example configuration files 

- jostick.remapping.axes.rear_stering_angle (string, default node)

  Id of the stick used to control rear_steering_angle, see  example configuration files 

- joystick.remapping.buttons.slow_mode (string, default none)

  Id of the button used to slow motion mode,  see  example configuration files 

- joystick.remapping.buttons.turbo_mode (string, default none) 

  Id of the button used to turbo motion mode,  see  example configuration files 

- cmd_output.type (string, default none)

  Type of messages published by teleop node : four_wheel_steering_msgs/FourWheelSteering or   romea_mobile_base_msgs/TwoAxleSteeringCommand

- cmd_output.priority(int, default -1)

  Priority of messages from 0 to 255. If this parameters is defined by user, the teleop will call automatically service cmd_mux/subscribe to register output topic (cmd_vel or cmd_one_axle_steering) to cmd_mux node ( see romea_cmd_mux package). 

- cmd_range.scale.maximal_linear_speed.slow_mode (double, default none)

  Output maximal linear speed when slow mode is activated   

- cmd_range.scale.maximal_linear_speed.turbo_mode (double, default none)

  Output maximal linear speed when turbo mode is activated 

- cmd_range.scale.maximal_front_steering_angle (double, default none)

  Output maximal front steering angle          

- cmd_range.scale.maximal_front_steering_angle (double, default none)

  Output maximal front steering angle 

