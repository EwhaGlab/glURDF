Talos_data
==========

## Introduction

This repository is dedicated to use the TALOS robot with the stack-of-tasks.

Specific data were generated to account to some specific problems when generating/planning motion
with the SoT.

For instance in order to avoid using xacro (with ROS) plain URDF were recorded.

Some dummy data were added (rotor inertia) to help the dynamic regularization.

For initial and data validated by PAL-Robotics please see the http://github.com/pal-robotics/talos_robot
repository.

If you have problem in starting the robot in a specific position you should look at the 
[python section](#python)

## Fixed joint
To start the robot in the air you can use:
```
roslaunch talos_data talos_gazebo.launch enable_fixed_robot:=true
```

## Straight robot
To start the robot straight (i.e. not in half-sitting) you can use:
```
roslaunch talos_data talos_gazebo.launch starting_half_sitting:=false
```

## Flexibilities

To start the flexibilities:
```
roslaunch talos_data talos_gazebo.launch enable_leg_passive:=true
```

You should see in the robot model two new links:
```
leg_left_1_link_passive
leg_right_1_link_passive
```
and two new joints
```
leg_left_1_joint_passive
leg_right_1_joint_passive
```

They are updated by the plugin in the src directory:
```
src/SpringPlugin.cc
```

This plugin is specified in the file:
```
urdf/leg/leg.urdf.xacro
```
by the following lines
```
    <gazebo>
      <plugin filename="libSpringPlugin.so" name="spring_leg_${prefix}_1_joint_passive">
        <joint_spring>leg_${prefix}_1_joint_passive</joint_spring>
        <kp>970.0</kp>
        <kd>0.0</kd>
      </plugin>
    </gazebo>
```

You can create new joints by expanding the robot kinematic tree
and adding the same lines in the xacro files.

The spring constants are modified by the associated fields
*kp* and *kd*.
The name of the joint to which this apply is specified by the field
*joint_spring*.

The plugin is generating a torque based on the actuator state using:
$$\tau = - kp q - kd \dot{q}$$ where $q$ is the actuator position and $\dot{q}$
is the actuator velocity.

This might be changed in the future to cope with a different equilibrium point.

Finally to have the state of the joint in the topic */joint_states* you need to add the following lines:
```
    <transmission name="leg_${prefix}_1_passive_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <actuator name="leg_${prefix}_1_motor" >
        <mechanicalReduction>1.0</mechanicalReduction>
      </actuator>
      <joint name="leg_${prefix}_1_joint_passive">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      </joint>
    </transmission>
```

This is telling to ros_control that the passive joint has Position and Effort interfaces.

## Python for launching nodes

In the directory scripts you can start a complete simulation by doing:
```
./start_talos_gazebo.py
```
In this specific case it will start the robot with a wide base.





