import numpy as np
from casadi import MX, cos, horzcat, sin, vertcat

from crazyflie_mpc_pkg.utils.configuration import QuadmodelConfig


class QuadrotorSimplified:

    x: MX  # state vector
    u: MX  # control vector
    f_expl: MX  # explicit dynamics function

    def __init__(self, config: QuadmodelConfig):
        self.mass = config.mass
        self.gravity = 9.80665
        self.arm_length = config.arm_length
        self.Ixx = config.Ixx
        self.Iyy = config.Iyy
        self.Izz = config.Izz
        self.cm = config.cm
        self.tau = config.tau
        self._init_dynamics()

    @staticmethod
    def euler_to_rotm(rpy):
        cpitch = cos(rpy[1])
        spitch = sin(rpy[1])
        croll = cos(rpy[0])
        sroll = sin(rpy[0])
        cyaw = cos(rpy[2])
        syaw = sin(rpy[2])
        Rotm = np.array(
            [
                [
                    cpitch * cyaw,
                    sroll * spitch * cyaw - croll * syaw,
                    croll * spitch * cyaw + sroll * syaw,
                ],
                [
                    cpitch * syaw,
                    sroll * spitch * syaw + croll * cyaw,
                    croll * spitch * syaw - sroll * cyaw,
                ],
                [-spitch, sroll * cpitch, croll * cpitch],
            ]
        )
        return Rotm

    def _init_dynamics(self) -> None:
        px = MX.sym("px")
        py = MX.sym("py")
        pz = MX.sym("pz")
        vx = MX.sym("vx")
        vy = MX.sym("vy")
        vz = MX.sym("vz")
        roll = MX.sym("roll")
        pitch = MX.sym("pitch")
        yaw = MX.sym("yaw")
        roll_c = MX.sym("roll_c")
        pitch_c = MX.sym("pitch_c")
        yaw_c = MX.sym("yaw_c")
        thrust = MX.sym("thrust")

        # Setup state and control vectors
        self.x = vertcat(px, py, pz, vx, vy, vz, roll, pitch, yaw)
        self.u = vertcat(roll_c, pitch_c, yaw_c, thrust)

        cpitch = cos(pitch)
        spitch = sin(pitch)
        croll = cos(roll)
        sroll = sin(roll)
        cyaw = cos(yaw)
        syaw = sin(yaw)

        # Define rotation matrix from quadrotor body to inertial reference frames
        Rotm = vertcat(
            horzcat(
                cpitch * cyaw,
                sroll * spitch * cyaw - croll * syaw,
                croll * spitch * cyaw + sroll * syaw,
            ),
            horzcat(
                cpitch * syaw,
                sroll * spitch * syaw + croll * cyaw,
                croll * spitch * syaw - sroll * cyaw,
            ),
            horzcat(-spitch, sroll * cpitch, croll * cpitch),
        )

        f_vec = vertcat(0.0, 0.0, thrust)

        # velocity dynamics
        vdot = vertcat(0.0, 0.0, -self.gravity) + Rotm @ f_vec / self.mass

        # Setup explicit ode equations
        pxdot = vx
        pydot = vy
        pzdot = vz
        vxdot = vdot[0]
        vydot = vdot[1]
        vzdot = vdot[2]
        rolldot = (roll_c - roll) / self.tau
        pitchdot = (pitch_c - pitch) / self.tau
        yawdot = (yaw_c - yaw) / self.tau

        # vector function of explicit dynamics
        self.f_expl = vertcat(pxdot, pydot, pzdot, vxdot, vydot, vzdot, rolldot, pitchdot, yawdot)
