import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from my_interface.msg import DescriptionMsg

class MyAggregator(Node):

    def __init__(self):
        super().__init__('nodo4')
             
        # --- SUSCRIPTORES (Reciben Float32) ---
        self.sub1 = self.create_subscription(Float32, '/sensor1', self.cb_sensor1, 10)
        self.sub2 = self.create_subscription(Float32, '/sensor2', self.cb_sensor2, 10)
        self.sub3 = self.create_subscription(Float32, '/sensor3', self.cb_sensor3, 10)

        # --- PUBLICADOR (Envía DescriptionMsg) ---
        self.publisher_ = self.create_publisher(DescriptionMsg, '/filtered_sensor', 10)

        # Variables para guardar los datos
        self.val1 = 0.0
        self.val2 = 0.0
        self.val3 = 0.0

        # --- TIMER ---
        timer_period = 0.5  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cb_sensor1(self, msg):
        self.val1 = msg.data

    def cb_sensor2(self, msg):
        self.val2 = msg.data

    def cb_sensor3(self, msg):
        self.val3 = msg.data

    def timer_callback(self):
        # 1. Calcular el promedio
        promedio = (self.val1 + self.val2 + self.val3) / 3.0
        
        # 2. Mostrar en la terminal del Nodo 4 lo que está pasando
        self.get_logger().info(
            f'DATOS: [S1: {self.val1:.2f}, S2: {self.val2:.2f}, S3: {self.val3:.2f}] | PROMEDIO: {promedio:.2f}'
        )

        msg_out = DescriptionMsg()
        msg_out.sensor_value = float(promedio)
        msg_out.name = "Filtered_Sensor"
        self.publisher_.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = MyAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()