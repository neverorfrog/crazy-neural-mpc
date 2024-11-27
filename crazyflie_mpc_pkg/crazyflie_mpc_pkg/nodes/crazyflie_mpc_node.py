from enum import Enum

import numpy as np
import rclpy
import rclpy.logging
import rclpy.node
from rclpy import executors
from rclpy.impl.rcutils_logger import RcutilsLogger

from crazyflie_mpc_pkg.mpc.mpc import ModelPredictiveController
from crazyflie_mpc_pkg.mpc.quadrotor_model import QuadrotorSimplified
from crazyflie_mpc_pkg.utils.configuration import MpcConfig
from crazyflie_swarm_pkg.utils import load_config


class Motors(Enum):
    MOTOR_CLASSIC = 1  # https://store.bitcraze.io/products/4-x-7-mm-dc-motor-pack-for-crazyflie-2 w/ standard props
    MOTOR_UPGRADE = 2  # https://store.bitcraze.io/collections/bundles/products/thrust-upgrade-bundle-for-crazyflie-2-x


class CrazyflieMPC(rclpy.node.Node):
    def __init__(
        self,
        node_name: str,
        mpc: ModelPredictiveController,
        quad: QuadrotorSimplified,
        plot_trajectory: bool = False,
    ):
        super().__init__(node_name)
        self.logger: RcutilsLogger = self.get_logger()
        self.logger.info("Crazyflie MPC Node: %s" % node_name)
        self.logger.info("Initialization completed...\n\n")


def main(args=None):
    rclpy.init(args=args)

    N_AGENTS = 4

    config = load_config("crazyflie_mpc_pkg/config/config.yaml", MpcConfig)
    rclpy.logging.get_logger("main").info(f"Configuration: {config}")

    quadmodel = QuadrotorSimplified(config.quadmodel)
    rclpy.logging.get_logger("main").info("Quadmodel initalized...")

    mpc = ModelPredictiveController("crazyflie", quadmodel, config)
    rclpy.logging.get_logger("main").info("MPC initalized...")

    nodes = [
        CrazyflieMPC("mpc_cf" + str(i), mpc, quadmodel)
        for i in np.arange(1, 1 + N_AGENTS)
    ]
    executor = executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        while rclpy.ok():
            node.get_logger().info(
                "Beginning multiagent executor, shut down with CTRL-C"
            )
            executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
