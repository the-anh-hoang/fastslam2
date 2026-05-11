"""
CLF (CARMEN Log Format) publisher node.

Reads a .clf file and publishes:
  - /scan        (sensor_msgs/LaserScan)
  - /odom        (nav_msgs/Odometry)
  - TF: odom -> base_footprint
  - TF: base_footprint -> base_scan (static)

Supports two CARMEN line formats:

FLASER (Intel, MIT CSAIL, etc.):
  FLASER <num_scans> <ranges...> <odom_x> <odom_y> <odom_theta>
         <laser_x> <laser_y> <laser_theta> <timestamp>

ROBOTLASER1 (Freiburg, ACES, etc.):
  ROBOTLASER1 <laser_type> <start_angle> <fov> <angular_res> <max_range>
              <accuracy> <remission_mode> <num_readings> <readings...>
              <num_remissions> <remissions...>
              <laser_pose_x> <laser_pose_y> <laser_pose_theta>
              <robot_pose_x> <robot_pose_y> <robot_pose_theta>
              ... <timestamp> <hostname> <logger_timestamp>
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class ClfPublisher(Node):
    def __init__(self):
        super().__init__('clf_publisher')

        self.declare_parameter('clf_file', '')
        self.declare_parameter('publish_rate', 5.0)  # Hz — how fast to replay
        self.declare_parameter('laser_frame', 'base_scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        # laser offset from base (meters). Intel dataset: laser roughly at robot center
        self.declare_parameter('laser_x_offset', 0.0)
        self.declare_parameter('laser_y_offset', 0.0)

        clf_file = self.get_parameter('clf_file').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.laser_frame = self.get_parameter('laser_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.laser_x = self.get_parameter('laser_x_offset').get_parameter_value().double_value
        self.laser_y = self.get_parameter('laser_y_offset').get_parameter_value().double_value

        if not clf_file:
            self.get_logger().fatal('No clf_file parameter provided!')
            raise SystemExit(1)

        # Parse the CLF file
        self.entries = self._parse_clf(clf_file)
        self.get_logger().info(f'Loaded {len(self.entries)} laser readings from {clf_file}')
        self.current_idx = 0

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Static TF: base_footprint -> base_scan
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_laser_tf()

        # Timer for replay
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._timer_callback)

    def _parse_clf(self, filepath):
        """Parse CARMEN .clf file, extract FLASER and ROBOTLASER1 entries."""
        entries = []
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                tokens = line.split()

                try:
                    if tokens[0] == 'FLASER':
                        # FLASER <num_scans> <ranges...>
                        #        <laser_x> <laser_y> <laser_theta>
                        #        <odom_x> <odom_y> <odom_theta> <timestamp>
                        num_readings = int(tokens[1])
                        readings = [float(tokens[2 + i]) for i in range(num_readings)]

                        offset = 2 + num_readings
                        # First triplet is corrected laser pose — skip it
                        # laser_x = float(tokens[offset])
                        # laser_y = float(tokens[offset + 1])
                        # laser_theta = float(tokens[offset + 2])
                        # Second triplet is raw odometry — use this
                        robot_x = float(tokens[offset + 3])
                        robot_y = float(tokens[offset + 4])
                        robot_theta = float(tokens[offset + 5])
                        timestamp = float(tokens[offset + 6])

                        # FLASER doesn't encode scan params — assume 180 beams, 180° FOV
                        # SICK LMS200: max reliable range ~8m, values like 81.83 are out-of-range
                        entries.append({
                            'start_angle': -math.pi / 2.0,
                            'fov': math.pi,
                            'angular_res': math.pi / num_readings,
                            'max_range': 18.0,
                            'num_readings': num_readings,
                            'readings': readings,
                            'robot_x': robot_x,
                            'robot_y': robot_y,
                            'robot_theta': robot_theta,
                            'timestamp': timestamp,
                        })

                    elif tokens[0] == 'ROBOTLASER1':
                        start_angle = float(tokens[2])
                        fov = float(tokens[3])
                        angular_res = float(tokens[4])
                        max_range = float(tokens[5])
                        num_readings = int(tokens[8])

                        readings = [float(tokens[9 + i]) for i in range(num_readings)]

                        offset = 9 + num_readings
                        num_remissions = int(tokens[offset])
                        offset += 1 + num_remissions

                        # skip laser pose (offset+0,1,2), take robot pose (offset+3,4,5)
                        robot_x = float(tokens[offset + 3])
                        robot_y = float(tokens[offset + 4])
                        robot_theta = float(tokens[offset + 5])
                        timestamp = float(tokens[-3])

                        entries.append({
                            'start_angle': start_angle,
                            'fov': fov,
                            'angular_res': angular_res,
                            'max_range': max_range,
                            'num_readings': num_readings,
                            'readings': readings,
                            'robot_x': robot_x,
                            'robot_y': robot_y,
                            'robot_theta': robot_theta,
                            'timestamp': timestamp,
                        })

                except (IndexError, ValueError) as e:
                    self.get_logger().warn(f'Skipping malformed line: {e}')
                    continue
        return entries

    def _publish_static_laser_tf(self):
        """Publish static transform: base_footprint -> base_scan."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame
        t.transform.translation.x = self.laser_x
        t.transform.translation.y = self.laser_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Static TF: {self.base_frame} -> {self.laser_frame} '
            f'({self.laser_x}, {self.laser_y}, 0.0)'
        )

    def _timer_callback(self):
        if self.current_idx >= len(self.entries):
            self.get_logger().info('Finished replaying dataset.')
            self.timer.cancel()
            return

        entry = self.entries[self.current_idx]
        now = self.get_clock().now().to_msg()

        # --- Publish LaserScan ---
        scan_msg = LaserScan()
        scan_msg.header.stamp = now
        scan_msg.header.frame_id = self.laser_frame
        scan_msg.angle_min = entry['start_angle']
        scan_msg.angle_max = entry['start_angle'] + entry['fov']
        scan_msg.angle_increment = entry['angular_res']
        scan_msg.range_min = 0.02
        scan_msg.range_max = entry['max_range']
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.publish_rate
        scan_msg.ranges = [
            r if r < entry['max_range'] else float('inf')
            for r in entry['readings']
        ]
        self.scan_pub.publish(scan_msg)

        # --- Publish Odometry ---
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = entry['robot_x']
        odom_msg.pose.pose.position.y = entry['robot_y']
        odom_msg.pose.pose.position.z = 0.0
        yaw = entry['robot_theta']
        odom_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.odom_pub.publish(odom_msg)

        # --- Publish TF: odom -> base_footprint ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = entry['robot_x']
        t.transform.translation.y = entry['robot_y']
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

        if self.current_idx % 100 == 0:
            self.get_logger().info(
                f'Publishing {self.current_idx}/{len(self.entries)} '
                f'pose=({entry["robot_x"]:.2f}, {entry["robot_y"]:.2f}, {yaw:.2f})'
            )
        self.current_idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = ClfPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()