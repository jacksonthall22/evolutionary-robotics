import pybullet as pyb


class World:
    def __init__(self):
        self.plane_id = pyb.loadURDF('plane.urdf')
        pyb.loadSDF('world.sdf')
