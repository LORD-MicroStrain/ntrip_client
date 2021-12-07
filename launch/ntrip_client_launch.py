from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
      return LaunchDescription([
            # Declare arguments with default values
            DeclareLaunchArgument('host',         default_value='20.185.11.35'),
            DeclareLaunchArgument('port',         default_value='2101'),
            DeclareLaunchArgument('mountpoint',   default_value='VTRI_RTCM3'),
            DeclareLaunchArgument('authenticate', default_value='True'),
            DeclareLaunchArgument('username',     default_value='user'),
            DeclareLaunchArgument('password',     default_value='pass'),

           # ****************************************************************** 
           # NTRIP Client Node
           # ****************************************************************** 
           Node(
                 name='ntrip_client_node',
                 namespace='ntrip_client',
                 package='ntrip_client',
                 executable='ntrip_ros.py',
                 parameters=[
                   {
                     # Required parameters used to connect to the NTRIP server
                     'host': LaunchConfiguration('host'),
                     'port': LaunchConfiguration('port'),
                     'mountpoint': LaunchConfiguration('mountpoint'),

                     # If this is set to true, we will read the username and password and attempt to authenticate. If not, we will attempt to connect unauthenticated
                     'authenticate': LaunchConfiguration('authenticate'),

                     # If authenticate is set the true, we will use these to authenticate with the server
                     'username': LaunchConfiguration('username'),
                     'password': LaunchConfiguration('password'),

                     # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                     'rtcm_frame_id': 'odom'
                   }
                 ],
                 # Uncomment the following section and replace "/gq7/nmea/sentence" with the topic you are sending NMEA on if it is not the one we requested
                #  remappings=[
                #    ("/ntrip_client/nmea", "/gx5/nmea/sentence")
                #  ],
                #  
           )
      ])
