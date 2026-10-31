import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Empty
from rclpy.signals import SignalHandlerOptions

class ObstacleAvoider(Node):
    # this function is called when the node is created
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.start_time = self.get_clock().now()  
        # add a logger to log messages
        self.get_logger().info(
            f'[ t {self.elapsed_time():6.1f}s | ]  '
            f'Obstacle Avoider Node has been started.')
        # add a publisher to publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        # add a subscriber to subscribe to the LaserScan messages
        self.scan_sub = self.create_subscription(
             LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        #---------------Parameters---------------- 
        # distances to check for obstacles
        self.declare_parameter('stop_distance', 0.4)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.declare_parameter('near_distance', 0.25)
        self.near_distance = self.get_parameter('near_distance').value
        self.declare_parameter('clear_distance', 0.6)
        self.clear_distance = self.get_parameter('clear_distance').value
        # speeds and timeouts
        self.declare_parameter('turn_speed', 0.5)
        self.turn_speed = self.get_parameter('turn_speed').value
        self.declare_parameter('forward_speed', 0.2)
        self.forward_speed = self.get_parameter('forward_speed').value
        self.declare_parameter('stuck_timeout', 15.0)
        self.stuck_timeout = self.get_parameter('stuck_timeout').value
        # Angles for the sectors 
        self.declare_parameter('start_front', -15)
        self.start_front = self.get_parameter('start_front').value
        self.declare_parameter('end_front', 15)
        self.end_front = self.get_parameter('end_front').value
        self.declare_parameter('start_left', 60)
        self.start_left = self.get_parameter('start_left').value
        self.declare_parameter('end_left', 120)
        self.end_left = self.get_parameter('end_left').value
        self.declare_parameter('start_right', 240)
        self.start_right = self.get_parameter('start_right').value
        self.declare_parameter('end_right', 300)
        self.end_right = self.get_parameter('end_right').value
        self.declare_parameter('start_near', -70)
        self.start_near = self.get_parameter('start_near').value
        self.declare_parameter('end_near', 70)
        self.end_near = self.get_parameter('end_near').value

        #---------------State Variables----------------
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')
        self.near = float('inf')  
        self.state = 'forward' 
        self.turn_direction = 1.0  # 1 for left, -1 for right
        # add a service to enable or disable obstacle avoidance
        self.enabled = True
        self.enable_srv = self.create_service(
            SetBool, 'enable_obstacle_avoidance', self.enable_callback)
        self.reset_client = self.create_client(Empty, '/reset_world')     


    # This function will be called periodically to control the robot's movement
    def control_loop(self):
        msg = Twist()

        if not self.enabled:
            # If obstacle avoidance is disabled, stop the robot
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)
            return

        if self.state == 'forward':
            if self.front < self.stop_distance or self.near < self.near_distance:
                # Obstacle detected in front, decide to turn left or right
                if self.left > self.right:
                    self.turn_direction = 1.0  # turn left
                else:
                    self.turn_direction = -1.0  # turn right
                self.state = 'turning'
                self.get_logger().info(
                    f'[ t {self.elapsed_time():6.1f}s | ]  '
                    f'Obstacle detected! Turning {"left" if self.turn_direction == 1.0 else "right"}.')
                self.turn_start = self.get_clock().now()  # record the time when turning starts
            else:
                # Path is clear, move forward
                msg.linear.x = self.forward_speed

        elif self.state == 'turning':
             
            # reset the world if the turn has lasted longer than stuck_timeout
            elapsed = (self.get_clock().now() - self.turn_start).nanoseconds / 1e9 
            if elapsed > self.stuck_timeout:
                self.get_logger().info(
                    f'[ t {self.elapsed_time():6.1f}s | ]  '
                    f'Resetting world due to prolonged turning.')
                self.reset_world()  
                self.state = 'forward'  
             
            # If the path is clear, go forward
            elif self.front > self.clear_distance and self.near > self.near_distance:
                self.state = 'forward'  
                self.get_logger().info(
                    f'[ t {self.elapsed_time():6.1f}s | ]  '
                    f'Path is clear, moving forward.')
            
            # If the path is not clear, keep turning
            else:
                msg.linear.x = 0.0
                msg.angular.z = self.turn_speed * self.turn_direction

        # Log the distances
        self.get_logger().info(
            f'[ t {self.elapsed_time():6.1f}s | ]  '
            f'[{self.state:<8}] front {self.front:5.2f} | near {self.near:5.2f} | '
            f'left {self.left:5.2f} | right {self.right:5.2f}',
            throttle_duration_sec=0.5)
        
        self.cmd_pub.publish(msg)
        
    # This function will be called when a new LaserScan message is received
    def scan_callback(self, msg):
        # msg is the LaserScan that just arrived
        self.front = self.sector_min(msg, self.start_front, self.end_front)
        self.left = self.sector_min(msg, self.start_left, self.end_left)
        self.right = self.sector_min(msg, self.start_right, self.end_right)
        self.near = self.sector_min(msg, self.start_near, self.end_near)

    # This function will be called when the service is called 
    # to enable or disable obstacle avoidance
    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f'Obstacle avoidance {"enabled" if self.enabled else "disabled"}.'
        self.get_logger().info(
            f'[ t {self.elapsed_time():6.1f}s | ]  '
            f'{response.message}')
        return response

    # This function returns the minimum valid distance in a sector of the LaserScan
    def sector_min(self, msg, start, end):
        # Empty list to collect the valid readings
        valid = []
        # Loop through the specified sector of the LaserScan ranges
        for i in range (start, end + 1):
            r = msg.ranges[i]
            if msg.range_min <= r <= msg.range_max:
                # if it's valid, add it to the list
                valid.append(r)
        # list empty: no valid readings, nothing detected, path is clear
        if len(valid) == 0:
            return float('inf')
        # if the list is not empty return the minimum value
        else:
            return min(valid)

    def reset_world(self):
        if not self.reset_client.service_is_ready():
            self.get_logger().warn('Reset world service client not available.')
            return
        self.reset_client.call_async(Empty.Request())
        self.get_logger().info(
            f'[ t {self.elapsed_time():6.1f}s | ]  '
            f'Reset world service requested.')

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            f'[ t {self.elapsed_time():6.1f}s | ]  '
            f'Robot stopped.')

    def elapsed_time(self):
        # Calculate the elapsed time since the node was started
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    obstacle_avoider = ObstacleAvoider()
    try:
        rclpy.spin(obstacle_avoider)
    except KeyboardInterrupt:
        obstacle_avoider.get_logger().info(
            f'[ t {obstacle_avoider.elapsed_time():6.1f}s | ]  '
            f'Keyboard interrupt, shutting down.')
    finally:
        obstacle_avoider.stop_robot()
        obstacle_avoider.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()