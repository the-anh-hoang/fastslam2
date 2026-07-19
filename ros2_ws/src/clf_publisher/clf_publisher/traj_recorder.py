import os
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


class TrajRecorder(Node):
    def __init__(self):
        super().__init__('traj_recorder')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'base_footprint')
        self.declare_parameter('output_file', 'results/trajectory.tum')
        self.declare_parameter('poll_rate', 20.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value
        output_file = self.get_parameter('output_file').value
        poll_rate = self.get_parameter('poll_rate').value

        out_dir = os.path.dirname(output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        self.file = open(output_file, 'w')
        self.last_stamp = None
        self.count = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / poll_rate, self._poll)

        self.get_logger().info(
            f'Recording {self.target_frame} -> {self.source_frame} to {output_file}'
        )

    def _poll(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        if stamp == self.last_stamp or stamp == 0.0:
            return
        self.last_stamp = stamp

        tr = t.transform.translation
        q = t.transform.rotation
        self.file.write(
            f'{stamp:.6f} {tr.x:.6f} {tr.y:.6f} {tr.z:.6f} '
            f'{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n'
        )
        self.file.flush()
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'Recorded {self.count} poses')

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.get_logger().info(f'Wrote {node.count} poses')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
