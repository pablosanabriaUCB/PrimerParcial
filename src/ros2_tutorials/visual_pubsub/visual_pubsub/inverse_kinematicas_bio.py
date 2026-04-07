import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematicas_bio')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.target_sub = self.create_subscription(
            Point, 'target_position', self.target_callback, 10)

        # 4 articulaciones
        self.q = np.array([0.0, 0.0, 0.0, 0.0])

        # Longitudes del URDF
        self.l1 = 10.0
        self.l2 = 6.0
        self.l3 = 4.0
        self.l4 = 3.0

        self.target_pos = np.array([10.0, 5.0])  # Solo XY

        self.timer = self.create_timer(0.1, self.update_joints)

        self.step_size = 0.1
        self.tolerance = 0.01
        self.damping = 0.1

    def forward_kinematics(self, q):
        q1, q2, q3, q4 = q

        c1 = q1
        c2 = q1 + q2
        c3 = q1 + q2 + q3
        c4 = q1 + q2 + q3 + q4

        x = (self.l1 * np.cos(c1) +
             self.l2 * np.cos(c2) +
             self.l3 * np.cos(c3) +
             self.l4 * np.cos(c4))

        y = (self.l1 * np.sin(c1) +
             self.l2 * np.sin(c2) +
             self.l3 * np.sin(c3) +
             self.l4 * np.sin(c4))

        return np.array([x, y])

    def jacobian(self, q):
        q1, q2, q3, q4 = q

        c1 = q1
        c2 = q1 + q2
        c3 = q1 + q2 + q3
        c4 = q1 + q2 + q3 + q4

        # Derivadas
        j11 = - (self.l1*np.sin(c1) + self.l2*np.sin(c2) +
                 self.l3*np.sin(c3) + self.l4*np.sin(c4))
        j12 = - (self.l2*np.sin(c2) + self.l3*np.sin(c3) + self.l4*np.sin(c4))
        j13 = - (self.l3*np.sin(c3) + self.l4*np.sin(c4))
        j14 = - self.l4*np.sin(c4)

        j21 = (self.l1*np.cos(c1) + self.l2*np.cos(c2) +
               self.l3*np.cos(c3) + self.l4*np.cos(c4))
        j22 = (self.l2*np.cos(c2) + self.l3*np.cos(c3) + self.l4*np.cos(c4))
        j23 = (self.l3*np.cos(c3) + self.l4*np.cos(c4))
        j24 = self.l4*np.cos(c4)

        return np.array([
            [j11, j12, j13, j14],
            [j21, j22, j23, j24]
        ])

    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y])
        self.get_logger().info(f"Target: {self.target_pos}")

    def update_joints(self):
        current = self.forward_kinematics(self.q)
        error = self.target_pos - current

        if np.linalg.norm(error) > self.tolerance:

            J = self.jacobian(self.q)

            # Damped Least Squares
            JT = J.T
            J_dls = JT @ np.linalg.inv(J @ JT + self.damping * np.eye(2))

            dq = J_dls @ error

            self.q += dq * self.step_size

        # Publicar
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['q1', 'q2', 'q3', 'q4']
        msg.position = self.q.tolist()

        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()