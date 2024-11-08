import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt

class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')
        self.subscription = self.create_subscription(
            Float32, '/clock', self.listener_callback, 10
        )
        self.data = []
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(-1, 1)
        self.get_logger().info("CrazyfliePlotNode started")

    def listener_callback(self, msg):
        self.get_logger().info("Listener callback")
        self.get_logger().info(f"Message received: {msg.data}")
        self.data.append(msg.data)
        self.update_plot()

    def update_plot(self):
        self.get_logger().info(f"Data: {self.data}")
        self.line.set_xdata(range(len(self.data)))
        self.line.set_ydata(self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
