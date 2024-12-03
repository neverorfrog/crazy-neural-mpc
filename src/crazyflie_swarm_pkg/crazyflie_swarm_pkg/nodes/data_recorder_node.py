from crazyflie_swarm_pkg.nodes.crazyflie_swarm_node import CrazyflieSwarmNode
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from crazyflie_swarm_pkg.utils import SwarmConfig, load_config

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
import datetime

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.config = self._get_config()
        self.writer = SequentialWriter()

        # Opening bag writer
        folder_name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        storage_options = StorageOptions(uri=f'crazy_bag/{folder_name}', storage_id='sqlite3')
        converter_options = ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Saving Control Input
        # control_topic_info = TopicMetadata(id = 0, name = )
        
    def _get_config(self) -> SwarmConfig:
        self.declare_parameter("swarm_config_path", "")
        swarm_config_path = (
            self.get_parameter("swarm_config_path").get_parameter_value().string_value
        )
        config = load_config(swarm_config_path, SwarmConfig)

        self.get_logger().info("DataRecorderNode started with parameters:")
        for cf_config in config.crazyflies:
            self.get_logger().info(f"  - {cf_config.name}: {cf_config.uri}")
        return config

    def topic_callback(self, msg):
        self.writer.write(
            'chatter',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    sbr = DataRecorderNode()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()