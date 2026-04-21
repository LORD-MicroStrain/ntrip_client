
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
      return LaunchDescription([
        # Declare arguments with default values
        DeclareLaunchArgument('namespace',             default_value='/'),
        DeclareLaunchArgument('node_name',             default_value='serial_gnss_device'),
        DeclareLaunchArgument('debug',                 default_value='false'),
        DeclareLaunchArgument('port',                  default_value='/dev/ttyACM1'),
        DeclareLaunchArgument('baudrate',              default_value='115200'),
        

        # Pass an environment variable to the node
        SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')),

        # ******************************************************************
        # Serial GNSS Device Node
        # ******************************************************************
        Node(
              name=LaunchConfiguration('node_name'),
              namespace=LaunchConfiguration('namespace'),
              package='ntrip_client',
              executable='serial_gnss_device_ros.py',
              parameters=[
                {
                  # Required parameters used to connect to serial GNSS device
                  'port': LaunchConfiguration('port'),
                  'baudrate': LaunchConfiguration('baudrate'),
                  
                  # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                  'nmea_frame_id': 'odom',

             
                  # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                  'nmea_max_length': 128,
                  'nmea_min_length': 3
                } 
              ],
        )
      ])

            
        