import rclpy
from rclpy.node import Node
from my_interface.msg import DescriptionMsg

class FinalSubscriber(Node):

    def __init__(self):
        super().__init__('nodo5')
        
        self.subscription = self.create_subscription(
            DescriptionMsg,
            '/filtered_sensor',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # El Nodo 5 muestra el mensaje final promediado
        self.get_logger().info(f'Filtered_Sensor value: {msg.name}: {msg.sensor_value:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FinalSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()