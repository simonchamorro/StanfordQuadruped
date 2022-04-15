import numpy as np
import pybullet

class IMU:
    def __init__(self):
        pass

    def read_orientation(self):
        (_, q_scalar_last) = pybullet.getBasePositionAndOrientation(1)
        return np.array(
            [q_scalar_last[3], q_scalar_last[0], q_scalar_last[1], q_scalar_last[2]]
        )
        
    def read_lin_ang_vel(self):
        lin_vel = np.zeros(3)
        ang_vel = np.zeros(3)
        breakpoint()
        return lin_vel, ang_vel

