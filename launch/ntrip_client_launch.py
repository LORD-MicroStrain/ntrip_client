from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
      return LaunchDescription([
          # Declare arguments with default values
          DeclareLaunchArgument('namespace',             default_value='/'),
          DeclareLaunchArgument('node_name',             default_value='ntrip_client'),
          DeclareLaunchArgument('debug',                 default_value='false'),
	  DeclareLaunchArgument('host',                  default_value='20.185.11.35'),
          DeclareLaunchArgument('port',                  default_value='2101'),
          DeclareLaunchArgument('mountpoint',            default_value='VRS_RTCM3'),
          DeclareLaunchArgument('ntrip_version',         default_value='None'),
          DeclareLaunchArgument('user_agent',            default_value='NTRIP ros_ntrip_client', description='HTTP User-Agent sent to the caster. Must start with "NTRIP ". rtk2go blocks the stock "NTRIP ntrip_client_ros".'),
          DeclareLaunchArgument('ntrip_server_hz',       default_value='1'), # set this to 1 for rtk2go
          DeclareLaunchArgument('send_nmea',             default_value='true', description='Forward NMEA from the "nmea" topic up to the caster. Needed for virtual/relayed (VRS) mountpoints; set false for plain base stations to skip the subscription and avoid uploading position.'),
          DeclareLaunchArgument('authenticate',          default_value='True'),
          DeclareLaunchArgument('username',              default_value='user'),
          DeclareLaunchArgument('password',              default_value='pass'),
          DeclareLaunchArgument('ssl',                   default_value='False'),
          DeclareLaunchArgument('cert',                  default_value='None'),
          DeclareLaunchArgument('key',                   default_value='None'),
          DeclareLaunchArgument('ca_cert',               default_value='None'),
          DeclareLaunchArgument('rtcm_message_package',  default_value='rtcm_msgs'),
          DeclareLaunchArgument('reconnect_attempt_wait_max_seconds', default_value='120', description='Ceiling for the exponential reconnect backoff. Retries are persistent (never give up); raise this to reduce footprint during long caster outages (e.g. 600 for rtk2go DDoS/maintenance downtime).'),

          # Pass an environment variable to the node
          SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')),


          # ******************************************************************
          # NTRIP Client Node
          # ******************************************************************
          Node(
                name=LaunchConfiguration('node_name'),
                namespace=LaunchConfiguration('namespace'),
                package='ntrip_client',
                executable='ntrip_ros.py',
                parameters=[
                  {
                    # Required parameters used to connect to the NTRIP server
                    'host': LaunchConfiguration('host'),
                    'port': LaunchConfiguration('port'),
                    'mountpoint': LaunchConfiguration('mountpoint'),

                    # Optional parameter that will set the NTRIP version in the initial HTTP request to the NTRIP caster.
                    'ntrip_version': LaunchConfiguration('ntrip_version'),

                    # User-Agent presented to the caster. Must start with "NTRIP ".
                    # rtk2go blocks the stock "NTRIP ntrip_client_ros" and refuses such clients with a sourcetable response.
                    'user_agent': LaunchConfiguration('user_agent'),
                      
                    # Rate to request correction messages. Some servers will sandbox clients that request too often
                    'ntrip_server_hz': LaunchConfiguration('ntrip_server_hz'),

                    # If this is set to true, we will read the username and password and attempt to authenticate. If not, we will attempt to connect unauthenticated
                    'authenticate': LaunchConfiguration('authenticate'),

                    # If authenticate is set the true, we will use these to authenticate with the server
                    'username': LaunchConfiguration('username'),
                    'password': LaunchConfiguration('password'),

                    # Whether to connect with SSL. cert, key, and ca_cert options will only take effect if this is true
                    'ssl': LaunchConfiguration('ssl'),

                    # If the NTRIP caster uses cert based authentication, you can specify the cert and keys to use with these options
                    'cert': LaunchConfiguration('cert'),
                    'key':  LaunchConfiguration('key'),

                    # If the NTRIP caster uses self signed certs, or you need to use a different CA chain, specify the path to the file here
                    'ca_cert': LaunchConfiguration('ca_cert'),

                    # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                    'rtcm_frame_id': 'odom',

                    # Whether to forward NMEA from the "nmea" topic up to the caster.
                    # Needed for virtual/relayed (VRS) mountpoints; disable for plain base stations.
                    'send_nmea': LaunchConfiguration('send_nmea'),

                    # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                    'nmea_max_length': 128,
                    'nmea_min_length': 3,

                    # Use this parameter to change the type of RTCM message published by the node. Defaults to "mavros_msgs", but we also support "rtcm_msgs"
                    'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),

                    # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                    'reconnect_attempt_max': 10,
                    'reconnect_attempt_wait_seconds': 10,
                    # Reconnect backoff: wait doubles from reconnect_attempt_wait_seconds up to
                    # reconnect_attempt_wait_max_seconds, then holds at that ceiling. Retries are
                    # persistent (no give-up) so the node recovers from long caster outages on its own.
                    'reconnect_attempt_wait_max_seconds': LaunchConfiguration('reconnect_attempt_wait_max_seconds'),

                    # How many seconds is acceptable in between receiving RTCM. If RTCM is not received for this duration, the node will attempt to reconnect
                    'rtcm_timeout_seconds': 10 #was 4 changed for rtk2go reqs
                  }
                ],
                # Uncomment the following section and replace "/gx5/nmea/sentence" with the topic you are sending NMEA on if it is not the one we requested
                #remappings=[
                #  ("nmea", "/gx5/nmea/sentence")
                #],
          )
      ])
