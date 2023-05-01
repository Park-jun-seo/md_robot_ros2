import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    
    usb1 = LaunchConfiguration('usb', default='/dev/ttyUSB0')  
    usb2 = LaunchConfiguration('usb', default='/dev/ttyUSB1')        
    
    use_MDUI = LaunchConfiguration('use_MDUI', default='0')             #<!-- 0: not use MDUI, 1: use MDUI -->
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0535')
    wheel_length = LaunchConfiguration('wheel_length', default='0.47')
    motor_pole = LaunchConfiguration('motor_pole', default='20')
    reduction = LaunchConfiguration('reduction', default='1')

    reverse_direction1 = LaunchConfiguration('reverse_direction', default='0')   #<!-- 0: forward, 1: reverse -->
    reverse_direction2 = LaunchConfiguration('reverse_direction', default='0')   #<!-- 0: forward, 1: reverse -->

    maxrpm = LaunchConfiguration('maxrpm', default='600')            #<!-- unit: RPM -->
    motor_posi = LaunchConfiguration('motor_posi', default='0')          #<!-- motor pisition 0: hall sensor, 1: encoder -->
    encoder_PPR = LaunchConfiguration('encoder_PPR', default='4096')            #<!-- if use encoder position, encoder PPR -->
    position_proportion_gain = LaunchConfiguration('position_proportion_gain', default='20')  #<!-- reference PID 203(PID_GAIN) -->
    speed_proportion_gain = LaunchConfiguration('speed_proportion_gain', default='50')     #<!-- reference PID 203(PID_GAIN) -->
    integral_gain = LaunchConfiguration('integral_gain', default='1800')           #<!-- reference PID 203(PID_GAIN) -->
    slow_start = LaunchConfiguration('slow_start', default='300')                #<!-- unit: RPM -->
    slow_down = LaunchConfiguration('slow_down', default='300')                 #<!-- unit: RPM -->
    
    fb_state1 = LaunchConfiguration('fb_state', default='f')   
    fb_state2 = LaunchConfiguration('fb_state', default='b')          
 

    node1 = Node(
        package='md_ros2',
        executable='md_ros2',
        name='f_md_ros2',
        parameters=[{'usb': usb1,
                     'use_MDUI': use_MDUI, 
                     'wheel_radius': wheel_radius, 
                     'wheel_length': wheel_length, 
                     'motor_pole': motor_pole, 
                     'reduction': reduction, 
                     'reverse_direction': reverse_direction1, 
                     'maxrpm': maxrpm, 
                     'motor_posi': motor_posi, 
                     'encoder_PPR': encoder_PPR, 
                     'position_proportion_gain': position_proportion_gain, 
                     'speed_proportion_gain': speed_proportion_gain, 
                     'integral_gain': integral_gain, 
                     'slow_start': slow_start, 
                     'slow_down': slow_down,
                     'fb_state':fb_state1}],
        output='screen'
    )

    node2 = Node(
        package='md_ros2',
        executable='md_ros2',
        name='b_md_ros2',
        parameters=[{'usb': usb2,
                     'use_MDUI': use_MDUI, 
                     'wheel_radius': wheel_radius, 
                     'wheel_length': wheel_length, 
                     'motor_pole': motor_pole, 
                     'reduction': reduction, 
                     'reverse_direction': reverse_direction2, 
                     'maxrpm': maxrpm, 
                     'motor_posi': motor_posi, 
                     'encoder_PPR': encoder_PPR, 
                     'position_proportion_gain': position_proportion_gain, 
                     'speed_proportion_gain': speed_proportion_gain, 
                     'integral_gain': integral_gain, 
                     'slow_start': slow_start, 
                     'slow_down': slow_down,
                     'fb_state':fb_state2}],
        output='screen'
    )



    ld.add_action(node1)
    ld.add_action(node2)

    return ld
