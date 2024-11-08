from matplotlib import pyplot as plt, patches
from ament_index_python.packages import get_package_share_directory
import numpy as np

import yaml
from omegaconf import OmegaConf

import os

root = get_package_share_directory("crazyflie_plotter_pkg")
file_path = os.path.join(root, "config", "msg_config.yaml")

with open(file_path, 'r') as file:
    config = OmegaConf.create(yaml.safe_load(file))

NAME = config.name
COLOR = config.color
colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

OBSTACLE = config.obstacle
DRONE = config.drone
FLOOR = config.floor

obs_db = []

class Plotter():
    def __init__(self, plot2d, plotGraphs, flock_config):

        self.flockingConfig = flock_config

        if plot2d:
            #plt.ion()       
            self.fig_2d = plt.figure("2D View")
            #self.line, = self.ax.plot([], [])
            #self.ax.set_xlim(0, 100)
            #self.ax.set_ylim(-1, 1)
            plt.pause(0.01)
        if plotGraphs:
            self.fig_graphs = plt.figure("Graphs")
        pass

    def plot_2d(self, myDrone, yaw, forces, v, des_v, obj_detected, clear, keepMap = False):
        plt.figure(self.fig_2d.number)

        if clear:
            plt.gcf().clear()

        plt.gca().set_aspect('equal')
        side = 4
        plt.gca().set(xlim=(-side, side), ylim=(-side, side))

        d = myDrone

        p = d[config.scope_pose][:2]
        #drone = patches.Circle((p[0], p[1]), config.radius, fill=True, label = d[NAME], color=d[COLOR])
        drone_back = patches.Wedge((p[0], p[1]), self.flockingConfig.dimensions.radius, 90 + np.rad2deg(yaw), 270 + np.rad2deg(yaw), fill=True, label=d[NAME], color=d[COLOR])

        xy = get_triangle(p[0:2], yaw, self.flockingConfig.dimensions.radius)
        drone_front = patches.Polygon(xy, facecolor=d[COLOR], fill=True)

        bound = patches.Circle((p[0], p[1]), self.flockingConfig.dimensions.d_eq/2 + self.flockingConfig.dimensions.radius, fill = False, color=d[COLOR], ls='--')
        plt.gca().add_patch(drone_back)
        plt.gca().add_patch(drone_front)
        plt.gca().add_patch(bound)


        # Plot inter-robot force
        force = forces[:, 0]
        force_norm = np.linalg.norm(force)
        plt.gca().arrow(p[0], p[1], force[0], force[1], length_includes_head=True, head_width=force_norm/10, head_length=force_norm/10, color = 'turquoise')

        # Plot obstacle avoiding force
        force = forces[:, 1]
        force_norm = np.linalg.norm(force)
        plt.gca().arrow(p[0], p[1], force[0], force[1], length_includes_head=True, head_width=force_norm/10, head_length=force_norm/10, color = 'r')

        # Plot migration force
        force = forces[:, 2]
        force_norm = np.linalg.norm(force)
        plt.gca().arrow(p[0], p[1], force[0], force[1], length_includes_head=True, head_width=force_norm/10, head_length=force_norm/10, color = 'b')

        #Plot overall force
        overall_force = np.sum(forces, axis=1)
        force_norm = np.linalg.norm(overall_force)
        plt.gca().arrow(p[0], p[1], overall_force[0], overall_force[1], length_includes_head=True, head_width=force_norm/10, head_length=force_norm/10, color = 'k')

        # Plot actual velocity
        v_norm = np.linalg.norm(v)
        plt.gca().arrow(p[0], p[1], v[0], v[1], length_includes_head=True, head_width=v_norm/10, head_length=v_norm/10, color = d[COLOR])

        # Plot desired velocity
        des_v_norm = np.linalg.norm(des_v)
        plt.gca().arrow(p[0], p[1], des_v[0], des_v[1], ls='--', fill = False, length_includes_head=True, head_width=des_v_norm/10, head_length=des_v_norm/10, color = d[COLOR])

        for o in obj_detected:
            pos = o[0]
            if o[2] == OBSTACLE or o[2] == DRONE:
                obstacle = patches.Circle((pos[0], pos[1]), 0.1, fill= not o[2] == DRONE, color=d[COLOR])
                if keepMap:
                    if o[2] == OBSTACLE and pos[2] > 0.8 and pos[2] < 1.2:
                        x = np.round(pos[0], 1)
                        y = np.round(pos[1], 1)
                        if not (x,y) in obs_db:
                            obs_db.append((x, y))
            else:
                obstacle = patches.Rectangle((pos[0], pos[1]), 0.1, 0.1, fill= False, color=d[COLOR])
            plt.gca().add_patch(obstacle)

        if clear and len(obs_db) > 0:
            obs_x, obs_y = zip(*obs_db)
            plt.scatter(obs_x, obs_y, s = 1, c='gray')

        return [plt, obs_db]

    def plot_graphs(self, inter_force, obst_force, mig_force, forces, vx_hist, vy_hist, omega_hist):
        plt.figure(self.fig_graphs.number)
        plt.gcf().clear()

        plt.subplot(3,3,1)
        plt.title("Inter-robot force")
        plt.plot(inter_force, color = 'r')

        plt.subplot(3,3,4)
        plt.gca().clear()
        plt.title("Obstacle force")
        plt.plot(obst_force, color = 'k')

        plt.subplot(3,3,7)
        plt.title("Migration force")
        plt.plot(mig_force, color = 'c')

        ax = plt.gcf().add_axes((0.4, 0.1, 0.5, 0.5))
        ax.set_title("Forces")
        dx_r = forces[0, 0]
        dy_r = forces[1, 0]
        dx_o = forces[0, 1]
        dy_o = forces[1, 1]
        dx_m = forces[0, 2]
        dy_m = forces[1, 2]
        lim = np.max([np.abs(dx_r), np.abs(dy_r), np.abs(dx_o), np.abs(dy_o), np.abs(dx_m), np.abs(dy_m)])
        ax.set(xlim=(-lim, lim), ylim=(-lim, lim))
        ax.arrow(0, 0, dx_r, dy_r, length_includes_head=True, head_width=inter_force[-1]/10, head_length=inter_force[-1]/10, color = 'r', label = 'Inter-robot')
        ax.arrow(0, 0, dx_o, dy_o, length_includes_head=True, head_width=obst_force[-1]/10, head_length=obst_force[-1]/10, color = 'k', label = 'Obstacle')
        ax.arrow(0, 0, dx_m, dy_m, length_includes_head=True, head_width=mig_force[-1]/10, head_length=mig_force[-1]/10, color = 'c', label='Migration')
        ax.legend()

        plt.subplot(3,3,2)
        plt.title("v")
        plt.plot(vx_hist, label=r'$v_x$')
        plt.plot(vy_hist, label=r'$v_y$')

        plt.subplot(3,3,3)
        plt.title(r"$\omega$")
        plt.plot(omega_hist)

        return plt
    
def get_triangle(base_center, angle, radius):
    base_center = np.reshape(np.array(base_center), (2, 1))
    front = np.array([base_center[0, 0] + radius * np.cos(angle), base_center[1, 0] + radius * np.sin(angle)])
    rear1 = np.array([[0], [+radius]])
    rear2 = np.array([[0], [-radius]])
    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    rear1xy = R @ rear1
    rear2xy = R @ rear2

    return np.vstack((front, (base_center + rear1xy).T, (base_center + rear2xy).T))