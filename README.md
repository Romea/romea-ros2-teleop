# 1 Overview #

This package contains teleop nodes for skid, ackermann, omni and four wheel steering robots. They translate data coming from joystick message into command message supported by vehicle controllers.

# 2 Nodes #

### 2.1 Common parameters ###

- cmd_mux_autoconnect(bool, default true)

  This option must be true when romea cmd_mux is used to multiplex vehicle command messages. During initialisation step, the teleop node will automatically register cmd_vel topic to the command multiplexer with a priority defined by user (see below). If another topic is already register with the same priority, unsubscription to cmd_mux is rejected and initialisation failed.

- priority(int, default 127);

  priority of cmd_vel topic from 0 to 255. The higher the more priority it has over the others topic register to command mutiplexer. This value not need to be set if cmd_mux_autoconnect is set to false.


### 2.2 skid_steering_teleop_node ###

### 2.2.1 Subscribed Topics ###

- joy : (sensor_msgs::Joy)

  Joystick messages to be translated to velocity commands.  

### 2.2.2 Published Topics ###

- cmd_vel (geometry_msgs::Twist)    

  Command velocity messages arising from Joystick commands.

### 2.2.3 Parameters ###

  - linear_axis_id (int, defaut none)

    Id of the stick used to control linear speed

  - angular_axis_id (int, default none)

    Id of the stick used to control angular speed

  - enable_button_id (int, default none)

    Id of the deadman button used to activate slow motion mode

  - linear_speed_scale (double, default none)

    Scale of linear speed when slow motion mode is activated   

  - angular_speed_scale (double, default none)

    Scale of angular speed when slow motion mode is activated    

  - enable_turbo_button_id (int, default none)

    Id of the deadman button used to activate turbo motion mode

  - linear_speed_turbo_scale (double, default none)

    Scale of linear speed when turbo motion mode is activated   

  - angular_speed_turbo_scale (double, default none)

    Scale of angular speed when turbo motion mode is activated    

  - deadzone (double, default 0.05)

    Dead zone value of sticks


# 2.3 omni_steering_teleop_node #

### 2.3.1 Subscribed Topics ###

  - joy : (sensor_msgs::Joy)

    Joystick messages to be translated to velocity commands.  

### 2.3.2 Published Topics ###

  - cmd_vel (geometry_msgs::Twist)    

    Command velocity messages arising from Joystick commands.


### 2.3.3 Parameters ###

  - linear_axis_id (int, defaut none)

      Id of the stick used to control linear speed

  - lateral_axis_id (int, defaut none)

      Id of the stick used to control lateral speed

  - angular_axis_id (int, default none)

      Id of the stick used to control angular speed

  - enable_button_id (int, default none)

      Id of the deadman button used to activate slow motion mode

  - linear_speed_scale (double, default none)

      Scale of linear speed when slow motion mode is activated   

  - lateral_speed_scale (double, default none)

      Scale of lateral speed when slow motion mode is activated   

  - angular_speed_scale (double, default none)

      Scale of angular speed when slow motion mode is activated    

  - enable_turbo_button_id (int, default none)

      Id of the deadman button used to activate turbo motion mode

  - linear_speed_turbo_scale (double, default none)

      Scale of linear speed when turbo motion mode is activated   

  - lateral_speed_scale (double, default none)

      Scale of lateral speed when turbo motion mode is activated   

  - angular_speed_turbo_scale (double, default none)

      Scale of angular speed when turbo motion mode is activated    

  - deadzone (double, default 0.05)

    Dead zone value of sticks


# 2.4 ackerman_steering_teleop_node #

### 2.4.1 Subscribed Topics ###

  - joy : (sensor_msgs::Joy)

    Joystick messages to be translated to velocity commands.  

### 2.4.2 Published Topics ###

  - cmd_vel (geometry_msgs::Twist)    

    Command velocity messages arising from Joystick commands. To ensure compatibility with ackermann controller from ros_controllers package geometry_msgs::Twist msg is used to command instead of an ackermann msg. Linear speed is set into twist.linear.x parameter and steering angle is set into twist.angular.z parameter (yes it is weird)  

### 2.4.3 Parameters ###

  - linear_axis_id (int, defaut none)

    Id of the stick used to control linear speed

  - steering_axis_id(int, defaut none)

    Id of the stick used to control steerig angle

  - steering_angle_scale (double, default none)

    Scale  of steering angle

  - enable_button_id (int, default none)

    Id of the deadman button used to activate slow motion mode

  - linear_speed_scale (double, default none)

    Scale of linear speed when slow motion mode is activated   

  - enable_turbo_button_id (int, default none)

    Id of the deadman button used to activate turbo motion mode

  - linear_speed_turbo_scale (double, default none)

    Scale of linear speed when turbo motion mode is activated   

  - deadzone (double, default 0.05)

    Dead zone value of sticks

# 2.5 four_wheel_steering_teleop_node #

### 2.5.1 Subscribed Topics ###

  - joy : (sensor_msgs::Joy)

    Joystick messages to be translated to velocity commands.  

### 2.5.2 Published Topics ###

    - cmd_four_wheel_steering (four_wheel_steering_msgs::FourWheelSteering)    

      Four wheel steering command messages arising from Joystick commands.

### 2.4.3 Parameters ###

    - linear_axis_id (int, defaut none)

      Id of the stick used to control linear speed

    - front_steering_axis_id(int, defaut none)

      Id of the stick used to control front steerig angle

    - front_steering_angle_scale (double, default none)

      Scale  of front steering angle

    - rear_steering_axis_id(int, defaut none)

      Id of the stick used to control rear steerig angle

    - rear_steering_angle_scale (double, default none)

      Scale  of rear steering angle

    - enable_button_id (int, default none)

      Id of the deadman button used to activate slow motion mode

    - linear_speed_scale (double, default none)

      Scale of linear speed when slow motion mode is activated   

    - enable_turbo_button_id (int, default none)

      Id of the deadman button used to activate turbo motion mode

    - linear_speed_turbo_scale (double, default none)

      Scale of linear speed when turbo motion mode is activated   

    - deadzone (double, default 0.05)

      Dead zone value of sticks
