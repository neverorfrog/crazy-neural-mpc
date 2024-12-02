from collections import deque
from typing import Deque

import numpy as np
import rclpy
import rclpy.logging
import rclpy.node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy import executors
from rclpy.impl.rcutils_logger import RcutilsLogger
from std_srvs.srv import Empty

from crazyflie_mpc_pkg.mpc.mpc import ModelPredictiveController
from crazyflie_mpc_pkg.mpc.quadrotor_model import QuadrotorSimplified
from crazyflie_mpc_pkg.trajectories.circular_trajectory import CircularTrajectory
from crazyflie_mpc_pkg.trajectories.trajectory import Trajectory, TrajectoryState
from crazyflie_mpc_pkg.utils.configuration import MpcConfig
from crazyflie_mpc_pkg.utils.definitions import FlightMode, MotorType
from crazyflie_swarm_interfaces.msg import AttitudeSetpoint, CrazyflieState
from crazyflie_swarm_interfaces.srv import Land, TakeOff
from crazyflie_swarm_pkg.utils import load_config


class CrazyflieMPC(rclpy.node.Node):
    def __init__(
        self,
        cf_name: str,
        mpc: ModelPredictiveController,
        quad: QuadrotorSimplified,
        reference: Trajectory,
        to_plot: bool = False,
    ):
        node_name = "mpc_" + cf_name
        super().__init__(node_name)
        self.cf_name = cf_name
        self.motor = MotorType.CLASSIC  # TODO: put in config
        self.logger: RcutilsLogger = self.get_logger()
        self.mpc = mpc
        self.quad = quad
        self.reference = reference
        self.to_plot = to_plot

        # Initialize stuff
        self.flight_mode: FlightMode = FlightMode.IDLE
        self.cmd_queue: Deque[np.ndarray] = None
        self.position = np.zeros(3)
        self.attitude = np.zeros(3)
        self.velocity = np.zeros(3)

        # Mpc specific stuff
        self.is_flying = False
        self.trajectory_changed = True
        self.trajectory_t0 = self.get_clock().now()

        # * Subscriptions
        self.state_sub = self.create_subscription(
            CrazyflieState, f"/{cf_name}/state", self._state_callback, 10
        )

        # * Publishers
        self.reference_pub = self.create_publisher(Path, f"/{cf_name}/reference", 10)
        self.mpc_pub = self.create_publisher(Path, f"/{cf_name}/mpc", 10)
        self.create_timer(1.0 / 10.0, self._mpc_callback)

        self.cmd_attitude_pub = self.create_publisher(
            AttitudeSetpoint, f"/{cf_name}/cmd_attitude", 10
        )
        self.create_timer(1.0 / 50.0, self._cmd_callback)

        # * Services (act on all active crazyflies)
        self.takeoff_srv = self.create_service(TakeOff, "/mpc_takeoff", self._takeoff_callback)
        self.land_srv = self.create_service(Land, "/mpc_land", self._land_callback)
        self.hover_srv = self.create_service(Empty, "/mpc_hover", self._hover_callback)
        self.trajectory_srv = self.create_service(Empty, "/mpc_traj", self._trajectory_callback)

        self.logger.info("Crazyflie MPC Node: %s" % node_name)
        self.logger.info("Initialization completed...\n\n")

    def _cmd_callback(self):
        """
        Publishes the attitude setpoint to the crazyflie at 50Hz.
        It accesses a deque of the last N control inputs and publishes the oldest one
        """
        if self.flight_mode is FlightMode.IDLE:
            return

        if not self.is_flying:
            self.is_flying = True
            cmd = np.array([0.0, 0.0, 0.0, 0.0])
            self.cmd_attitude_setpoint(cmd)

        if not self.cmd_queue or len(self.cmd_queue) == 0:
            self.logger.warning("No control input available")
            return

        self.cmd_attitude_setpoint(self.cmd_queue.popleft())

    def cmd_attitude_setpoint(self, cmd):
        assert len(cmd) == 4, "Control input must be of length 4"

        setpoint = AttitudeSetpoint()
        setpoint.roll = np.degrees(cmd[0])
        setpoint.pitch = np.degrees(cmd[1])
        setpoint.yaw_rate = 3.0 * (np.degrees(cmd[2]))  # TODO dunno why
        setpoint.thrust = self.thrust_to_pwm(cmd[3])  # TODO dunno why
        self.logger.info(f"Attitude setpoint: {setpoint}")
        self.cmd_attitude_pub.publish(setpoint)

    def _mpc_callback(self):
        if self.flight_mode is FlightMode.IDLE:
            return

        if self.trajectory_changed:
            self.trajectory_start_position = self.position
            self.trajectory_t0 = self.get_clock().now()
            self.trajectory_changed = False

        t = (self.get_clock().now() - self.trajectory_t0).nanoseconds / 10.0**9

        trajectory = self._reference_callback(t)  # updates and publishes the reference trajectory
        yref = trajectory[:, :-1]
        yref_e = trajectory[:, -1]

        x0 = np.array([*self.position, *self.velocity, *self.attitude])
        x_mpc, u_mpc = self.mpc.solve(x0, yref, yref_e)
        self.cmd_queue = deque(
            u_mpc.T
        )  # store the control inputs in a deque (it is important to have the control inputs per rows)

    def _state_callback(self, msg: CrazyflieState):
        self.position = np.array([msg.position[0], msg.position[1], msg.position[2]])
        self.attitude = np.array(
            [
                np.radians(msg.euler_orientation[0]),
                np.radians(msg.euler_orientation[1]),
                np.radians(msg.euler_orientation[2]),
            ]
        )

        if self.attitude[2] > np.pi:
            self.attitude[2] -= 2 * np.pi
        elif self.attitude[2] < -np.pi:
            self.attitude[2] += 2 * np.pi

        self.velocity = np.array(
            [msg.linear_velocity[0], msg.linear_velocity[1], msg.linear_velocity[2]]
        )

    def _takeoff_callback(self, request: TakeOff.Request, response: TakeOff.Response):
        try:
            height = request.height
            self.takeoff_duration = request.duration
            self.flight_mode = FlightMode.TAKEOFF
            self.takeoff_goal_pos = np.array([self.position[0], self.position[0], height])
            response.success = True

        except Exception as e:
            self.logger.error(f"Error in takeoff_callback: {e}")
            response.success = False

        return response

    def _land_callback(self, request: Land.Request, response: Land.Response):
        try:
            self.land_duration = request.duration
            self.flight_mode = FlightMode.LANDING
            self.land_goal_pos = np.array([self.position[0], self.position[1], 0.0])
            response.success = True

        except Exception as e:
            self.logger.error(f"Error in land_callback: {e}")
            response.success = False

        return response

    def _hover_callback(self, request: Empty.Request, response: Empty.Response):
        try:
            self.flight_mode = FlightMode.HOVER
            self.hover_goal_pos = self.position

        except Exception as e:
            self.logger.error(f"Error in hover_callback: {e}")

        return response

    def _trajectory_callback(self, request: Empty.Request, response: Empty.Response):
        try:
            self.flight_mode = FlightMode.TRAJECTORY
            self.trajectory_changed = True

        except Exception as e:
            self.logger.error(f"Error in trajectory_callback: {e}")

        return response

    def _reference_callback(self, t: float) -> np.ndarray:
        yref = self._compute_reference(t)

        ref_msg = Path()
        ref_msg.header.frame_id = "world"
        ref_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.mpc.N):
            ref_pose = PoseStamped()
            ref_pose.pose.position.x = yref[0, i]
            ref_pose.pose.position.y = yref[1, 0]
            ref_pose.pose.position.z = yref[2, i]
            ref_msg.poses.append(ref_pose)

        self.reference_pub.publish(ref_msg)
        return yref

    def _compute_reference(self, t: float) -> np.ndarray:
        """
        Computes the reference trajectory based on the current flight mode and time.
        Returns a 9xN array of reference states.
        """

        t_mpc_array = np.linspace(t, self.mpc.tf + t, self.mpc.N + 1)
        if self.flight_mode is FlightMode.IDLE:
            return
        if self.flight_mode is FlightMode.TAKEOFF:
            error = self.takeoff_goal_pos - self.position
            progress_array = [
                1.0
                / (
                    1.0
                    + np.exp(
                        -(12.0 * (t_mpc - self.takeoff_duration) / self.takeoff_duration + 6.0)
                    )
                )
                for t_mpc in t_mpc_array
            ]
            yref = np.array(
                [
                    np.array([*(self.position + error * progress), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    for progress in progress_array
                ]
            ).T
        if self.flight_mode is FlightMode.LANDING:
            error = self.land_goal_pos - self.position
            progress_array = [
                1.0
                / (1.0 + np.exp(-(12.0 * (t_mpc - self.land_duration) / self.land_duration + 6.0)))
                for t_mpc in t_mpc_array
            ]
            yref = np.array(
                [
                    np.array([*(self.position + error * progress), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    for progress in progress_array
                ]
            ).T
        if self.flight_mode is FlightMode.HOVER:
            yref = np.repeat(
                np.array([[*self.hover_goal_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T,
                self.mpc.N + 1,
                axis=1,
            )
        if self.flight_mode is FlightMode.TRAJECTORY:
            yref = []
            for t_mpc in t_mpc_array:
                traj_state: TrajectoryState = self.reference.update(t_mpc)
                yref.append(
                    np.array(
                        [
                            traj_state.x,
                            traj_state.y,
                            traj_state.z,
                            traj_state.dx,
                            traj_state.dy,
                            traj_state.dz,
                            0.0,
                            0.0,
                            0.0,
                        ]
                    )
                )
            yref = np.array(yref).T
        return yref

    def thrust_to_pwm(self, collective_thrust: float) -> int:
        collective_thrust = max(collective_thrust, 0.0)  # make sure it's not negative
        if self.motor == MotorType.CLASSIC:
            return int(
                max(
                    min(
                        24.5307 * (7460.8 * np.sqrt((collective_thrust / 4.0)) - 380.8359),
                        65535,
                    ),
                    0,
                )
            )
        elif self.motor == MotorType.UPGRADE:
            return int(
                max(
                    min(
                        24.5307 * (6462.1 * np.sqrt((collective_thrust / 4.0)) - 380.8359),
                        65535,
                    ),
                    0,
                )
            )


def main(args=None):
    rclpy.init(args=args)

    N_AGENTS = 1

    config = load_config("src/crazyflie_mpc_pkg/config/config.yaml", MpcConfig)
    rclpy.logging.get_logger("main").info(f"Configuration: {config}")

    quadmodel = QuadrotorSimplified(config.quadmodel)
    rclpy.logging.get_logger("main").info("Quadmodel initalized...")

    mpc = ModelPredictiveController("crazyflie", quadmodel, config)
    rclpy.logging.get_logger("main").info("MPC initalized...")

    reference = CircularTrajectory(center=[0, -1, 0], freq=0.1)
    rclpy.logging.get_logger("main").info("Reference trajectory initalized...")

    nodes = [
        CrazyflieMPC("cf" + str(i), mpc, quadmodel, reference) for i in np.arange(1, 1 + N_AGENTS)
    ]
    executor = executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    try:
        while rclpy.ok():
            node.get_logger().info("Beginning multiagent executor, shut down with CTRL-C")
            executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
