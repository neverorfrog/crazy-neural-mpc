import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from dataclasses import dataclass

@dataclass
class CrazyState:
  # Position
  x: float = 0.0
  y: float = 0.0
  z: float = 0.0
  
  # Euler orientation
  roll: float = 0.0
  pitch: float = 0.0
  yaw: float = 0.0
  
  # Quaternion orientation
  qx: float = 0.0
  qy: float = 0.0
  qz: float = 0.0
  qw: float = 0.0
  
  # Linear Velocity
  vx: float = 0.0
  vy: float = 0.0
  vz: float = 0.0
  
  # Angular Velocity
  roll_rate: float = 0.0
  pitch_rate: float = 0.0
  yaw_rate: float = 0.0
  
  # Linear Acceleration
  ax: float = 0.0
  ay: float = 0.0
  az: float = 0.0
  
  def __str__(self):
    return f'Position: ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})\n' + \
           f'Euler Orientation: ({self.roll:.2f}, {self.pitch:.2f}, {self.yaw:.2f})\n' + \
           f'Quaternion Orientation: ({self.qx:.2f}, {self.qy:.2f}, {self.qz:.2f}, {self.qw:.2f})\n' + \
           f'Linear Velocity: ({self.vx:.2f}, {self.vy:.2f}, {self.vz:.2f})\n' + \
           f'Angular Velocity: ({self.roll_rate:.2f}, {self.pitch_rate:.2f}, {self.yaw_rate:.2f})\n' + \
           f'Linear Acceleration: ({self.ax:.2f}, {self.ay:.2f}, {self.az:.2f})'