import pybullet as pb


class World:
    def __init__(self):
        self.plane_id = pb.loadURDF('plane.urdf')
        pb.loadSDF('world.sdf')
