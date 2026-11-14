"""Keepout filter mask publisher.

Stage 3 placeholder: publishes the Nav2 keepout filter mask (OccupancyGrid,
reliable + transient_local) and lets polygons sent on /keepout_mask_test be
painted into it. The Stage 5 mission supervisor replaces this node as the
mask publisher, reusing the pure functions of ``keepout_mask``.
"""
import rclpy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from vr_robot_pkg import keepout_mask as km


class KeepoutMaskNode(Node):
    def __init__(self):
        super().__init__('keepout_mask_node')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('origin_x', -2.5)
        self.declare_parameter('origin_y', -4.25)
        self.declare_parameter('width', 310)
        self.declare_parameter('height', 210)
        self.declare_parameter('mask_topic', '/keepout_filter_mask')
        self.declare_parameter('republish_period', 0.0)

        p = self.get_parameter
        self.frame_id = p('frame_id').value
        self.spec = km.GridSpec(
            resolution=float(p('resolution').value),
            origin_x=float(p('origin_x').value),
            origin_y=float(p('origin_y').value),
            width=int(p('width').value),
            height=int(p('height').value))
        self.mask = km.empty_mask(self.spec)

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            OccupancyGrid, p('mask_topic').value, qos)
        self.create_subscription(
            PolygonStamped, '/keepout_mask_test', self._on_polygon, 10)

        period = float(p('republish_period').value)
        if period > 0.0:
            self.create_timer(period, self.publish_mask)
        self.publish_mask()

    def _on_polygon(self, msg):
        pts = [(pt.x, pt.y) for pt in msg.polygon.points]
        if not pts:
            self.mask = km.empty_mask(self.spec)
        else:
            km.rasterize_polygon(self.mask, self.spec, pts)
        self.publish_mask()

    def publish_mask(self):
        msg = OccupancyGrid()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.frame_id
        msg.info.map_load_time = now
        msg.info.resolution = self.spec.resolution
        msg.info.width = self.spec.width
        msg.info.height = self.spec.height
        msg.info.origin.position.x = self.spec.origin_x
        msg.info.origin.position.y = self.spec.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = km.to_occupancy_data(self.mask)
        self.pub.publish(msg)
        self.get_logger().info(
            f'Published keepout mask: {int((self.mask == km.KEEPOUT).sum())} '
            'keepout cells')


def main(args=None):
    rclpy.init(args=args)
    node = KeepoutMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
