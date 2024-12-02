import importlib
import pathlib
import sys
from typing import Tuple

import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from ament_index_python import get_package_share_directory
from scipy.linalg import block_diag

from crazyflie_mpc_pkg.mpc.quadrotor_model import QuadrotorSimplified
from crazyflie_mpc_pkg.utils.configuration import MpcConfig
from rclpy.impl.rcutils_logger import RcutilsLogger


class ModelPredictiveController:
    def __init__(
        self,
        name: str,
        quadrotor: QuadrotorSimplified,
        config: MpcConfig,
        to_generate: bool = True,
    ):
        self.name = name
        self.quad = quadrotor
        self.config = config
        self.to_generate = to_generate
        self.N = config.horizon
        self.tf = self.N * config.dt
        self.acados_gen_path = (
            pathlib.Path(get_package_share_directory("crazyflie_mpc_pkg")).resolve()
            / "acados_generated_files"
        )
        self.k = 0
        self.ocp_solver = None
        self.solver_locked = False
        self.hover_control = np.array([0.0, 0.0, 0.0, self.quad.gravity * self.quad.mass])

        if not self.to_generate:
            try:
                if self.acados_gen_path.is_dir():
                    sys.path.append(str(self.acados_gen_path))
                acados_ocp_solver_pyx = importlib.import_module(
                    "c_generated_code.acados_ocp_solver_pyx"
                )
                self.solver: AcadosOcpSolver = acados_ocp_solver_pyx.AcadosOcpSolverCython(
                    self.name, "SQP", self.N
                )
                print("Acados cython module imported successfully.")
            except ImportError:
                print("Acados cython code was not found. Generating it now...")
                self.to_generate = True
        if self.to_generate:
            self._generate_mpc()

    def _generate_mpc(self):

        # Create Acados model
        model = AcadosModel()
        model.name = self.name
        model.f_expl_expr = self.quad.f_expl
        model.x = self.quad.x
        model.u = self.quad.u

        # Define Optimal Control Problem
        self.ocp = AcadosOcp()
        self.ocp.model = model
        self.ocp.code_export_directory = self.acados_gen_path / ("c_generated_code")
        json_file = str(self.acados_gen_path / "acados_ocp.json")

        # Dimensions
        self.nx = self.quad.x.rows()  # number of states
        self.nu = self.quad.u.rows()  # number of controls
        self.ny = self.nx + self.nu  # number of optimization variables
        self.ny_e = self.nx  # number of optimization variables at the last stage

        # Horizon
        self.ocp.solver_options.N_horizon = self.N  # number of horizon stages
        self.ocp.solver_options.tf = self.tf  # prediction horizon

        # Cost function
        self.ocp.cost.cost_type = "LINEAR_LS"
        self.ocp.cost.cost_type_e = "LINEAR_LS"
        cost_weights = self.config.cost_weights
        Q = np.diag(
            [
                cost_weights.x,
                cost_weights.y,
                cost_weights.z,
                cost_weights.vx,
                cost_weights.vy,
                cost_weights.vz,
                cost_weights.roll,
                cost_weights.pitch,
                cost_weights.yaw,
            ]
        )

        R = np.diag(
            [
                cost_weights.roll_c,
                cost_weights.pitch_c,
                cost_weights.yaw_c,
                cost_weights.thrust,
            ]
        )

        self.ocp.cost.W = block_diag(Q, R)
        self.ocp.cost.W_e = Q

        # Mapping Matrices
        self.ocp.cost.Vx = np.vstack([np.eye(self.nx), np.zeros((self.nu, self.nx))])
        self.ocp.cost.Vx_e = np.eye(self.nx)
        self.ocp.cost.Vu = np.vstack([np.zeros((self.nx, self.nu)), np.eye(self.nu)])

        # Reference Stub
        self.ocp.cost.yref = np.zeros(self.ny)
        self.ocp.cost.yref_e = np.zeros(self.ny_e)

        # Constraints
        input_constraints = self.config.input_constraints
        state_constraints = self.config.state_constraints
        self.ocp.constraints.lbu = np.array(
            [
                input_constraints.roll_min,
                input_constraints.pitch_min,
                input_constraints.yaw_min,
                input_constraints.thrust_min,
            ]
        )
        self.ocp.constraints.ubu = np.array(
            [
                input_constraints.roll_max,
                input_constraints.pitch_max,
                input_constraints.yaw_max,
                input_constraints.thrust_max,
            ]
        )
        self.ocp.constraints.idxbu = np.arange(self.nu)

        self.ocp.constraints.lbx = np.array(
            [
                state_constraints.x_min,
                state_constraints.y_min,
                state_constraints.z_min,
                state_constraints.vx_min,
                state_constraints.vy_min,
                state_constraints.vz_min,
                state_constraints.roll_min,
                state_constraints.pitch_min,
                state_constraints.yaw_min,
            ]
        )
        self.ocp.constraints.ubx = np.array(
            [
                state_constraints.x_max,
                state_constraints.y_max,
                state_constraints.z_max,
                state_constraints.vx_max,
                state_constraints.vy_max,
                state_constraints.vz_max,
                state_constraints.roll_max,
                state_constraints.pitch_max,
                state_constraints.yaw_max,
            ]
        )
        self.ocp.constraints.idxbx = np.arange(self.nx)

        # Initial state
        self.ocp.constraints.x0 = np.zeros(self.nx)

        # Solver Options
        self.ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        self.ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        self.ocp.solver_options.nlp_solver_type = "SQP"
        self.ocp.solver_options.integrator_type = "ERK"
        self.ocp.solver_options.tol = 1e-3
        self.ocp.solver_options.qp_tol = 1e-3
        self.ocp.solver_options.nlp_solver_max_iter = 20
        self.ocp.solver_options.qp_solver_iter_max = 30
        self.ocp.solver_options.print_level = 0

        # Generate c code
        AcadosOcpSolver.generate(self.ocp, json_file)
        AcadosOcpSolver.build(self.ocp.code_export_directory, with_cython=True)
        if self.acados_gen_path.is_dir():
            sys.path.append(str(self.acados_gen_path))
        acados_ocp_solver_pyx = importlib.import_module("c_generated_code.acados_ocp_solver_pyx")
        self.solver: AcadosOcpSolver = acados_ocp_solver_pyx.AcadosOcpSolverCython(
            self.name, self.ocp.solver_options.nlp_solver_type, self.N
        )

    def solve(self, x0: np.ndarray, yref: np.ndarray, yref_e: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if self.solver_locked:
            return
        self.solver_locked = True

        for i in range(self.N):
            self.solver.set(i, 'yref', np.array([*yref[:,i], *self.hover_control]))
        self.solver.set(self.N, 'yref', yref_e)
        
        x_mpc = np.zeros((self.nx, self.N + 1))
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)
        u_mpc = np.zeros((self.nu, self.N))
        
        self.solver.solve()
        
        # extract state and control solution from solver
        for i in range(self.N):
            x_mpc[:, i] = self.solver.get(i, "x")
            u_mpc[:, i] = self.solver.get(i, "u")
        x_mpc[:, self.N] = self.solver.get(self.N, "x")
        
        self.solver_locked = False
        
        return x_mpc, u_mpc
        
        