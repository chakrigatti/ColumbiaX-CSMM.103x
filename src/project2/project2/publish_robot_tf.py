import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
import tf2_ros
import math
from rclpy.time import Time
import tf_transformations
import numpy as np

class PublishRobotTF(Node):

    def __init__(self):
        super().__init__('robot_tf_publisher')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_dict={}
        # Set timer to publish transforms at regular intervals
        self.define_transforms()
        self.timer = self.create_timer(0.1, self.broadcast_transforms)

    def message_from_transform(self,T):
        '''
        http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Transform.html

        geometry_msgs/Vector3 translation
        geometry_msgs/Quaternion rotation
        '''
        msg = Transform()
        q = tf_transformations.quaternion_from_matrix(T)
        translation = tf_transformations.translation_from_matrix(T)
        msg.translation.x = translation[0]
        msg.translation.y = translation[1]
        msg.translation.z = translation[2]
        msg.rotation.x = q[0]
        msg.rotation.y = q[1]
        msg.rotation.z = q[2]
        msg.rotation.w = q[3]
        return msg
    
    def get_rotation_matrix_from_vector(self,vector):
        # Normalize the input vector (new x-axis)
        vector = np.array(vector)
        x_axis = vector / np.linalg.norm(vector)
        
        # Create an arbitrary vector that is not parallel to the input vector
        arbitrary_vec = np.array([0, 0, 1]) if np.allclose(x_axis, [1, 0, 0]) else np.array([1, 0, 0])
        
        # Compute the new z-axis as the cross product of the new x-axis and the arbitrary vector
        z_axis = np.cross(x_axis, arbitrary_vec)
        z_axis /= np.linalg.norm(z_axis)  # Normalize the z-axis
        
        # Compute the new y-axis as the cross product of the new z-axis and the new x-axis
        y_axis = np.cross(z_axis, x_axis)
        
        # Return the rotation matrix (new basis)
        rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
        
        # Create the 4x4 transformation matrix
        transformation_matrix = np.eye(4)  # Start with an identity matrix
        transformation_matrix[:3, :3] = rotation_matrix  # Set the upper left 3x3 to the rotation matrix
        
        return transformation_matrix
        
    def inverse_transformation_matrix(self,matrix):
        # Extract the rotation part (upper-left 3x3) and translation part (last column, first 3 elements)
        rotation_matrix = matrix[:3, :3]
        translation_vector = matrix[:3, 3]
        
        # Invert the rotation matrix (since it is orthogonal, the inverse is the transpose)
        rotation_matrix_inv = np.transpose(rotation_matrix)
        
        # Invert the translation: -R_inv * T
        translation_inv = -np.dot(rotation_matrix_inv, translation_vector)
        
        # Construct the inverse transformation matrix
        inverse_matrix = np.eye(4)  # Start with an identity matrix
        inverse_matrix[:3, :3] = rotation_matrix_inv  # Set the rotation part
        inverse_matrix[:3, 3] = translation_inv  # Set the translation part
        
        return inverse_matrix

    def define_transforms(self):

        ####

        T1 = tf_transformations.concatenate_matrices(
    	    tf_transformations.quaternion_matrix(tf_transformations.quaternion_from_euler(0.79, 0.0, 0.79)),
    	    tf_transformations.translation_matrix((0.0, 1.0, 1.0))
        )
        object_transform = self.message_from_transform(T1)
        self.create_transform(frame_name='object',parent_frame='base',transform = object_transform)
        
        ####

        T2 = tf_transformations.concatenate_matrices(
            tf_transformations.quaternion_matrix(tf_transformations.quaternion_about_axis(1.5, (0.0, 0.0, 1.0))),
            tf_transformations.translation_matrix((0.0,-1.0,0.0))
        )
        robot_transform = self.message_from_transform(T2)
        self.create_transform(frame_name='robot',parent_frame='base',transform=robot_transform)

        ####

        camera_translation_matrix = tf_transformations.translation_matrix((0.0,0.1,0.1))

        camera_transform_in_base_frame = tf_transformations.concatenate_matrices(
            T2,
            camera_translation_matrix
        )
        base_transform_in_camera_frame = self.inverse_transformation_matrix(camera_transform_in_base_frame)

        object_origin_in_base_frame = T1[:,3]
        object_origin_in_camera_frame = np.matmul(base_transform_in_camera_frame,object_origin_in_base_frame)

        R3 = self.get_rotation_matrix_from_vector(object_origin_in_camera_frame[:3])
        T3 = tf_transformations.concatenate_matrices(
                camera_translation_matrix,
                R3
            )

        camera_transform = self.message_from_transform(T3)
        self.create_transform(frame_name='camera',parent_frame='robot',transform=camera_transform)

    
    def create_transform(self, frame_name,parent_frame,transform=None,translation=None,rotation=None):
        t = None
        if frame_name not in self.tf_dict:
            t = TransformStamped()
            self.tf_dict[frame_name] = t
            t.header.frame_id = parent_frame
            t.child_frame_id = frame_name
        else:
            t = self.tf_dict[frame_name]

        t.header.stamp = self.get_clock().now().to_msg()

        if transform is not None :
            t.transform = transform
        else :
            if translation is not None :
                t.transform.translation.x = translation['x']
                t.transform.translation.y = translation['y']
                t.transform.translation.z = translation['z']

            if rotation is not None:
                t.transform.rotation.x = rotation['x']
                t.transform.rotation.y = rotation['y']
                t.transform.rotation.z = rotation['z']
                t.transform.rotation.w = rotation['w']
        

    def broadcast_transforms(self):
        for x in self.tf_dict:
            self.tf_dict[x].header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(self.tf_dict[x])



def main(args=None):
    rclpy.init(args=args)
    node = PublishRobotTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
