from ament_index_python.packages import get_package_share_directory
import numpy as np

import yaml
from omegaconf import OmegaConf

import os

root = get_package_share_directory("crazyflie_simulation_pkg")
file_path = os.path.join(root, "config", "msg_config.yaml")

with open(file_path, 'r') as file:
    config = OmegaConf.create(yaml.safe_load(file))

from crazyflie_simulation_pkg.utils import Plotter
import crazyflie_simulation_pkg.utils.stringer as stringer

class World():
    def __init__(self, VIEW_2D, PLOT_GRAPHS):
        self.plotter = Plotter(VIEW_2D, PLOT_GRAPHS)

        self.obs_db = []

    def plot2d(self, drones, title = ''):
        plot = None

        for d in drones.values():
            if d[config.forces] == '':  # not ready yet
                continue

            yaw = d[config.scope_pose][-1]
            forces = stringer.get_forces(d[config.forces])
            v = d[config.scope_velocity]
            des_v = stringer.get_velocity(d[config.des_v])
            obj_detected = stringer.get_objects(d[config.obj_detected])

            plot, self.obs_db = self.plotter.plot_2d(d, yaw, forces, v, des_v, obj_detected, clear = d[config.name] == "cf1", keepMap=True)

        if plot is not None:
            plot.legend()

            plot.rc('grid', linestyle=':', color='black')

            plot.grid()

            plot.title(title)

            plot.pause(0.05)

    def plot_graphs(self, inter_force, obst_force, mig_force, forces, vx_hist, vy_hist, omega_hist):
        plot = self.plotter.plot_graphs(inter_force, obst_force, mig_force, forces, vx_hist, vy_hist, omega_hist)
        
        plot.legend()
        plot.pause(0.05)