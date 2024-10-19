import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        
        # Create a publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Create a buffer and listener for TF transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer to check for transforms and publish markers
        self.timer = self.create_timer(1, self.timer_callback)

    def publish_marker(self, marker_id, marker_type, pose, scale, color, frame_id="base"):
        marker = Marker()
        marker.header.frame_id = frame_id  # Set frame to 'base' or other relevant frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_shapes"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        # Set the pose (position and orientation)
        marker.pose.position.x = float(pose['position'][0])
        marker.pose.position.y = float(pose['position'][1])
        marker.pose.position.z = float(pose['position'][2])
        marker.pose.orientation.x = float(pose['orientation'][0])
        marker.pose.orientation.y = float(pose['orientation'][1])
        marker.pose.orientation.z = float(pose['orientation'][2])
        marker.pose.orientation.w = float(pose['orientation'][3])

        # Set the scale of the marker (Ensure visibility by setting a reasonable scale)
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])

        # Set the color of the marker (RGBA) and ensure alpha is 1 for full visibility
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = max(0.5, float(color[3]))  # Ensure some visibility with alpha

        marker.lifetime = Duration(seconds=0).to_msg()  # Marker stays forever
        self.get_logger().info(f'Publishing marker id {marker_id} in frame {frame_id}')
        self.marker_pub.publish(marker)

    def timer_callback(self):
        # Try to get the transform from each frame
        marker_definitions = [
            {"marker_id": 0, "frame_id": "object", "scale": [0.5, 0.5, 0.5], "color": [0.0, 1.0, 0.0, 1.0], "type": Marker.CYLINDER},
            {"marker_id": 1, "frame_id": "robot", "scale": [0.5, 0.5, 0.5], "color": [0.0, 1.0, 0.0, 1.0], "type": Marker.CUBE},
            {"marker_id": 2, "frame_id": "camera", "scale": [0.5, 0.2, 0.2], "color": [0.0, 1.0, 0.0, 1.0], "type": Marker.ARROW},
        ]

        for marker_info in marker_definitions:
            try:
                # Set the parent frame to 'base' for all markers except the 'camera' one
                parent_frame = 'base'
                if marker_info["marker_id"] == 2:
                    parent_frame = 'robot'
                
                # Look for the transform in the tf buffer
                transform = self.tf_buffer.lookup_transform(parent_frame, marker_info['frame_id'], rclpy.time.Time())
                
                # Extract the position and orientation from the transform
                pose = {
                    "position": [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z],
                    "orientation": [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
                }

                # Publish the marker with the frame matching the TF parent frame
                self.publish_marker(marker_id=marker_info["marker_id"], marker_type=marker_info["type"],
                                    pose=pose, scale=marker_info["scale"], color=marker_info["color"], frame_id=parent_frame)
            except Exception as e:
                self.get_logger().warn(f'Could not find transform for {marker_info["frame_id"]} relative to {parent_frame}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
