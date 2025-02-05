import time
from threading import Event
from typing import Dict

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils.multiranger import Multiranger

from crazyflie_swarm_pkg.crazyflie.crazyflie_state import CrazyState
from crazyflie_swarm_pkg.utils import RangeDirection, RingBuffer, log


class CrazyflieRobot:
    def __init__(
        self,
        uri,
        name=None,
        ro_cache=None,
        rw_cache=None,
        logger=None,
        multiranger=False,
        initial_position=None,
        is_simulated=True,
    ):
        self.uri = uri
        self.name = name
        self.cf = Crazyflie(ro_cache=ro_cache, rw_cache=rw_cache)
        self.scf = SyncCrazyflie(self.uri, cf=self.cf)
        self.logger = logger
        self.is_simulated = is_simulated
        self.state_update_dt = 10 # ms
        
        self.hovering_height = 0.3
        self.emergency_stop_distance = 0.1  # TODO: put in config

        # State
        self.initial_position = initial_position
        self.state = CrazyState()
        self.state.init_x = initial_position.x
        self.state.init_y = initial_position.y
        self.state.init_z = initial_position.z

        log(f"{self.state}", self.logger)

        self.estimators: Dict[str, LogConfig] = {}

        # Connection
        self.__connection_timeout = 10  # seconds
        self.__connection_opened = False

        # Flow deck
        self.__flow_deck_attached = False
        self.flow_deck_attached_event = Event()
        self.flow_deck_attached_event.clear()

        # Multiranger
        multiranger_buffer_size = 10
        self.multiranger = multiranger
        self.__multiranger_attached = False
        self.multiranger_attached_event = Event()
        self.multiranger_attached_event.clear()
        self.multiranger_sensor = Multiranger(self.scf)
        self.__buffer: Dict[RangeDirection, RingBuffer] = {}
        self.__buffer[RangeDirection.FRONT] = RingBuffer(multiranger_buffer_size, (1,))
        self.__buffer[RangeDirection.RIGHT] = RingBuffer(multiranger_buffer_size, (1,))
        self.__buffer[RangeDirection.BACK] = RingBuffer(multiranger_buffer_size, (1,))
        self.__buffer[RangeDirection.LEFT] = RingBuffer(multiranger_buffer_size, (1,))
        self.__buffer[RangeDirection.UP] = RingBuffer(multiranger_buffer_size, (1,))
        self.__mean_buffer: Dict[RangeDirection, float] = {}
        self.__mean_buffer[RangeDirection.FRONT] = 0.0
        self.__mean_buffer[RangeDirection.RIGHT] = 0.0
        self.__mean_buffer[RangeDirection.BACK] = 0.0
        self.__mean_buffer[RangeDirection.LEFT] = 0.0
        self.__mean_buffer[RangeDirection.UP] = 0.0

        self.flight_mode = "idle"
        self.take_off_done = False
        self.is_flying = False

    # * Initialization
    def initialize(self):
        log(f"Connecting to Crazyflie {self.name} ...", self.logger)
        start_initialization = time.time()
        self.open_connection()

        # * Led sanity check
        if not self.is_simulated:
            self.set_led(255.0)

            self.scf.cf.param.add_update_callback(
                group="deck",
                name="bcFlow2",
                cb=self.flow_deck_attached_callback,
            )
            if self.multiranger:
                self.scf.cf.param.add_update_callback(
                    group="deck",
                    name="bcMultiranger",
                    cb=self.multiranger_deck_attached_callback,
                )

            while (
                not self.__connection_opened
                or not self.__flow_deck_attached
                or (not self.__multiranger_attached and self.multiranger)
            ):
                if time.time() - start_initialization > self.__connection_timeout:
                    log(f"Initialization timeout for {self.name}", self.logger)
                    self.close_connection()
                    return False
                time.sleep(0.1)

            log(f"Crazyflie {self.name} connected.", self.logger)

            log(
                f"Resetting estimators of Crazyflie {self.name} ...",
                self.logger,
            )
            self.reset_estimator()
            log(f"Estimators of Crazyflie {self.name} reset.", self.logger)

            self.set_led(0.0)

        log(
            f"Starting estimators of Crazyflie {self.name} ...",
            self.logger,
        )
        self.setup_estimators()
        log(f"Estimators of Crazyflie {self.name} started.", self.logger)

        log(f"Crazyflie {self.name} initialized", self.logger)

        return True

    def destroy(self):
        # Destroy estimators
        for estimator in self.estimators.values():
            estimator.stop()

        # Destroy multiranger
        if not self.is_simulated:
            self.multiranger_sensor.stop()
            self.multiranger_attached_event.clear()
            self.__multiranger_attached = False

            # Destroy flow deck
            self.flow_deck_attached_event.clear()
            self.__flow_deck_attached = False

        # Close connection
        self.close_connection()
        log(f"Crazyflie {self.name} destroyed", self.logger)

    # Flow deck management
    def flow_deck_attached_callback(self, _, value_str):
        if int(value_str):
            self.flow_deck_attached_event.set()
            self.__flow_deck_attached = True
            log(f"Flow deck attached to {self.name}", self.logger)
        else:
            log(f"Flow deck is not attached to {self.name}", self.logger)

    def multiranger_deck_attached_callback(self, _, value_str) -> None:
        if int(value_str):
            self.multiranger_attached_event.set()
            self.__multiranger_attached = True
            self.multiranger_sensor.start()
            log(f"Multiranger is attached to {self.name}", self.logger)
        else:
            log(f"Multiranger is not attached to {self.name}", self.logger)

    # Connection management
    def open_connection(self):
        if self.__connection_opened:
            raise Exception("Connection already opened")
        try:
            log(f"Opening connection to {self.scf._link_uri} ...", self.logger)
            self.scf.open_link()
            self.__connection_opened = True
        except Exception as e:
            log(f"Error opening connection: {e}", self.logger)
            self.close_connection()
            raise e

    def close_connection(self):
        self.__connection_opened = False
        self.scf.close_link()

    # * Update
    def update(self):
        if self.flight_mode == "idle":
            return
        
        # * Multirange values (filled only if above a certain height)
        if self.state.z > 0.1:
            self.__buffer[RangeDirection.FRONT].append(self.multiranger_sensor.front)
            self.__buffer[RangeDirection.RIGHT].append(self.multiranger_sensor.right)
            self.__buffer[RangeDirection.BACK].append(self.multiranger_sensor.back)
            self.__buffer[RangeDirection.LEFT].append(self.multiranger_sensor.left)
            self.__buffer[RangeDirection.UP].append(self.multiranger_sensor.up)
            self.__mean_buffer[RangeDirection.FRONT] = self.__buffer[
                RangeDirection.FRONT
            ].compute_mean()
            self.__mean_buffer[RangeDirection.RIGHT] = self.__buffer[
                RangeDirection.RIGHT
            ].compute_mean()
            self.__mean_buffer[RangeDirection.BACK] = self.__buffer[
                RangeDirection.BACK
            ].compute_mean()
            self.__mean_buffer[RangeDirection.LEFT] = self.__buffer[
                RangeDirection.LEFT
            ].compute_mean()
            self.__mean_buffer[RangeDirection.UP] = self.__buffer[RangeDirection.UP].compute_mean()

        # * Handle take off and land
        z = self.state.z
        if self.flight_mode == "take_off" and self.state.z < self.hovering_height - 0.05:
            self.cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, self.hovering_height)
            self.is_flying = True
        elif self.flight_mode is not "flying" and self.state.z >= self.hovering_height - 0.05:
            log(f"Take off done for Crazyflie {self.name}", self.logger)
            self.take_off_done = True
            self.is_flying = True
            self.flight_mode = "flying"
            
        if self.flight_mode == "land":
            self.cf.commander.send_velocity_world_setpoint(0, 0, -0.05, 0)
        if self.take_off_done and z <= 0.05:
            log(f"Land done for Crazyflie {self.name}", self.logger)
            self.cf.commander.send_stop_setpoint()
            self.flight_mode = "idle"
            self.is_flying = False
            self.take_off_done = False

        # * Handle multirange
        if self.multiranger and self.take_off_done:
            if (
                self.__mean_buffer[RangeDirection.FRONT] < self.emergency_stop_distance
                or self.__mean_buffer[RangeDirection.RIGHT] < self.emergency_stop_distance
                or self.__mean_buffer[RangeDirection.BACK] < self.emergency_stop_distance
                or self.__mean_buffer[RangeDirection.LEFT] < self.emergency_stop_distance
            ):
                log(f"Emergency stop for Crazyflie {self.name}", self.logger)
                self.emergency_stop()

    # * Commands
    def take_off(self, absolute_height=None, duration=None):
        if not self.__connection_opened:
            raise Exception("Connection not opened")
        if absolute_height is None:
            raise Exception("Absolute height not provided")
        if not self.is_simulated:
            if not self.__flow_deck_attached:
                raise Exception("Flow deck not attached")
            if self.multiranger and not self.__multiranger_attached:
                raise Exception("Multiranger not attached")

        self.hovering_height = absolute_height + 0.1
        self.logger.info(f"Taking off {self.name} to {absolute_height} m")
        self.flight_mode = "take_off"

    def land(self, duration=None):
        if not self.is_flying:
            log(f"Not flying {self.name}", self.logger)
            return
        self.is_flying = False
        self.flight_mode = "land"

    def emergency_stop(self):
        self.cf.commander.send_stop_setpoint()

    def set_velocity(self, vx, vy, yaw_rate):
        if not self.__connection_opened:
            raise Exception("Connection not opened")
        if not self.is_simulated:
            if not self.__flow_deck_attached:
                raise Exception("Flow deck not attached")
            if self.multiranger and not self.__multiranger_attached:
                raise Exception("Multiranger not attached")
        if not self.take_off_done:
            log(f"Not flying {self.name}", self.logger)
            return
        self.cf.commander.send_hover_setpoint(vx, vy, yaw_rate, self.hovering_height)

    def set_attitude(self, roll, pitch, yaw_rate, thrust):
        if not self.__connection_opened:
            raise Exception("Connection not opened")
        if not self.is_simulated:
            if not self.__flow_deck_attached:
                raise Exception("Flow deck not attached")
            if self.multiranger and not self.__multiranger_attached:
                raise Exception("Multiranger not attached")
        self.cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)

    # * Setters
    def set_led(self, intensity):
        try:
            self.cf.param.set_value("led.bitmask", intensity)
        except Exception as e:
            log(f"Error setting led intensity: {e}", self.logger)

    # * Getters
    def get_state(self) -> CrazyState:
        if not self.is_simulated:
            self.get_multiranger_data()
        return self.state

    def get_multiranger_data(self):
        self.state.mr_front = self.__mean_buffer[RangeDirection.FRONT]
        self.state.mr_right = self.__mean_buffer[RangeDirection.RIGHT]
        self.state.mr_back = self.__mean_buffer[RangeDirection.BACK]
        self.state.mr_left = self.__mean_buffer[RangeDirection.LEFT]
        self.state.mr_up = self.__mean_buffer[RangeDirection.UP]

    def pos_estimator_callback(self, timestamp, data, logconf):
        self.state.x = data["stateEstimate.x"]
        self.state.y = data["stateEstimate.y"]
        self.state.z = data["stateEstimate.z"]

    def vel_estimator_callback(self, timestamp, data, logconf):
        self.state.vx = data["stateEstimate.vx"]
        self.state.vy = data["stateEstimate.vy"]
        self.state.vz = data["stateEstimate.vz"]

    def attitude_estimator_callback(self, timestamp, data, logconf):
        self.state.roll = data["stabilizer.roll"]
        self.state.pitch = data["stabilizer.pitch"]
        self.state.yaw = data["stabilizer.yaw"]

    def setup_estimators(self):
        pos_estimator = LogConfig(name="Position", period_in_ms=self.state_update_dt)
        pos_estimator.add_variable("stateEstimate.x", "float")
        pos_estimator.add_variable("stateEstimate.y", "float")
        pos_estimator.add_variable("stateEstimate.z", "float")
        self.cf.log.add_config(pos_estimator)
        self.estimators["pos"] = pos_estimator
        pos_estimator.data_received_cb.add_callback(self.pos_estimator_callback)
        pos_estimator.start()

        vel_estimator = LogConfig(name="Velocity", period_in_ms=self.state_update_dt)
        vel_estimator.add_variable("stateEstimate.vx", "float")
        vel_estimator.add_variable("stateEstimate.vy", "float")
        vel_estimator.add_variable("stateEstimate.vz", "float")
        self.cf.log.add_config(vel_estimator)
        self.estimators["vel"] = vel_estimator
        vel_estimator.data_received_cb.add_callback(self.vel_estimator_callback)
        vel_estimator.start()

        attitude_estimator = LogConfig(name="Attitude", period_in_ms=self.state_update_dt)
        attitude_estimator.add_variable("stabilizer.roll", "float")
        attitude_estimator.add_variable("stabilizer.pitch", "float")
        attitude_estimator.add_variable("stabilizer.yaw", "float")
        self.cf.log.add_config(attitude_estimator)
        self.estimators["attitude"] = attitude_estimator
        attitude_estimator.data_received_cb.add_callback(self.attitude_estimator_callback)
        attitude_estimator.start()

    # * Estimator Reset
    def reset_estimator(self):
        self.cf.param.set_value("kalman.resetEstimation", "1")
        time.sleep(0.1)
        self.cf.param.set_value("kalman.resetEstimation", "0")

        log_config = LogConfig(name="Kalman Variance", period_in_ms=self.state_update_dt)
        log_config.add_variable("kalman.varPX", "float")
        log_config.add_variable("kalman.varPY", "float")
        log_config.add_variable("kalman.varPZ", "float")

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.001

        with SyncLogger(self.scf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]

                var_x_history.append(data["kalman.varPX"])
                var_x_history.pop(0)
                var_y_history.append(data["kalman.varPY"])
                var_y_history.pop(0)
                var_z_history.append(data["kalman.varPZ"])
                var_z_history.pop(0)

                min_x = min(var_x_history)
                max_x = max(var_x_history)
                min_y = min(var_y_history)
                max_y = max(var_y_history)
                min_z = min(var_z_history)
                max_z = max(var_z_history)

                if (
                    (max_x - min_x) < threshold
                    and (max_y - min_y) < threshold
                    and (max_z - min_z) < threshold
                ):
                    break
