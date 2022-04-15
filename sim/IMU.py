import numpy as np
import pybullet

class IMU:
    
    def __init__(self, simulator=True):
        
        self.simulator = simulator
        self.prev_pos = None
        self.prev_angle = None

    def _simulator_observation(self, dt=1/240.0, urdf_id=1):
        # URDF ID

        base_position, base_orientation = pybullet.getBasePositionAndOrientation(urdf_id)

        projection_gravity = np.array([0, 0, -9.81]) @ self._compute_rotation_matrix(base_orientation)
        obj_euler = pybullet.getEulerFromQuaternion(base_orientation)
        
        if self.prev_pos == None:
            velocity = base_position
        else:
            velocity = (base_postion - self.prev_pos) / dt
        
        if self.prev_angle == None:
            ang_velocity = obj_euler
            self.prev_pos = base_position
        else:
            ang_velocity = (obj_euler - self.prev_angle) / dt
            self.prev_angle = obj_euler


        return velocity, ang_velocity, projection_gravity

    def read_orientation(self):        
        return pybullet.getBasePositionAndOrientation(1)[1]


    def _compute_rotation_matrix(self, Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix



        


