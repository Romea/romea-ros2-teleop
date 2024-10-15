# R Teleop Description

# 1 Overview #

This package contains :
  - Default remappings between the components of the control messages and the joystick buttons and axes. 
  - A Python module able to complete and clamp user teleop settings according joystick configuration, default remappings and geometrical and mechanical parameters of the robots in order to generate teleop node configuration (see romea_teleop_drivers package).

# 2 Example of teleop node configuration generation  #

Let's consider that we want to teleoperate an one axle steering robot  in using a xbox joystick handle by joy_node from [joy package](https://github.com/ros-drivers/joystick_drivers). Let's suppose that :

- the teleop configuration provided by user is : 

```yaml
 cmd_output: # output configuration
   message_type: romea_mobile_base_msgs/OneAxleSteeringCommand # type of msg published by teleop
   message_priority: 100 # priority for cmd_mux
 cmd_range: # cmd range configuration
   maximal_linear_speed: 
    slow_mode: 1.0 # maximal linear speed (m/s) for slow mode 
    turbo_mode: 5.0 # maximal linear speed (m/s) for turbo mode
```
where the command range configuration is partially defined and no joystick mapping is given 

- the geometric and mechanical parameters of the robot are (see [romea_mobile_base_description](https://gitlab.irstea.fr/romea_ros2/interfaces/vehicles/romea_mobile_base/-/tree/main/romea_mobile_base_description) package for more info ):
```yaml
type: 1FAS2RWD
geometry:
  axles_distance: 2. # m
  front_axle:
    wheels_distance: 1.2. #m
  rear_axle:
    wheels_distance: 1.6. #m

front_axle_steering_control:
  command:
    maximal_angle: 0.5  #rad

rear_wheels_speed_control:
    command:
      maximal_speed: 2. #m/s
```


In this case : 
  - joystick configuration (see file [xbox_joy.yaml](https://gitlab.irstea.fr/romea_ros2/interfaces/teleoperation/romea_joy/-/blob/main/romea_joystick_description/config/xbox_joy.yaml)  from config directory of romea_joystick_description package) is :   
```yaml
buttons:
  mapping:
    LB: 4
    RB: 5
  values:
    unpressed: 0
    pressed: 1
axes:
  sticks:
    mapping:
      Vertical_Left_Stick: 1
      Horizontal_Right_Stick: 3
    values:
      range: [-1.0, 1.0] #[right left] or [down up ]
```
and default remappings (see file [xbox_one_axle_steering_remappings.yaml](https://gitlab.irstea.fr/romea_ros2/interfaces/teleoperation/romea_teleop/-/blob/main/romea_teleop_description/config/xbox_one_axle_steering_remappings.yaml)  from config directory of this romea_teleop_description package)  are: 
```yaml
axes:
  linear_speed: Vertical_Left_Stick
  steering_angle: Horizontal_Right_Stick
buttons:
  slow_mode: LB
  turbo_mode: RB
```
It is possible to use python module  : 
```python
from romea_teleop_description import complete_teleop_configuration

joystick_type= "xbox"
joystick_driver= "joy"
mobile_base = # load mobile base parameters from a yaml file
user_configuration = # load user configuration from a yaml file

teleop_node_configuration =  complete_teleop_configuration(
    user_configuration, mobile_base, joystick_type, joystick_driver
):

print(teleop_node_configuration)    
```
in order to generate the complete configuration of one axle steering teleop node : 
```yaml
  cmd_output: # output configuration
    message_type: romea_mobile_base_msgs/OneAxleSteeringCommand # same as user config
    message_priority: 100 # same as user config
  cmd_range: # cmd range configuration
    maximal_linear_speed: 
      slow_mode: 1.0 # same as user config
      turbo_mode: 2.0 # clamped according maximal wheel speed (see robot configuration)
    maximal_steering_angle: # deduced from according mechanical limits (see robot configuration)
  joystick_mapping: # filled according joystick configuration dans default remappings 
    axes: # 
      linear_speed: 1 # id of the joystick axe used to get linear_speed 
      steering_angle: 3 # id of the joystick axe used to get steering_angle
    buttons:
      slow_mode: 4 # id of the joystick button used to get slow_mode status
      turbo_mode: 5 # id of the joystick button used to get turbo_mode status
```
where : 
  - maximal_steering_angle has been added and is equal to front_axle_steering_control.command.maximal_angle from mobile base configuration 
  - maximal_linear_speed  for turbo mode has been clamped and is equal to rear_wheels_speed_control.maximal_speed parameter from mobile base configuration  
  - joystick mapping is deduced from joystick configuration and default teleop joystick remappings 



The complete_teleop_configuration function of romea_teleop_description Python module can be used to generate configuration of any kind of ROMEA teleop_drivers nodes. It's possible to see an implementation into romea_teleop_bringup python module.

