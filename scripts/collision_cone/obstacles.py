#!/usr/bin/env python3


class Obstacle:
    """
    This class helps keep track of all obstacles in the environment
    which have to be avoided
    """
    def __init__(self, is_long=True, r=1.0):
        self.c_x = 0
        self.c_y = 0
        self.c_z = 0
        self.r = r
        self.v_x = 0
        self.v_y = 0
        self.v_z = 0
        self.is_long = is_long

        return

    def update(self, x, y, z=0, v_x=0, v_y=0, v_z=0):
        self.c_x = x
        self.c_y = y
        self.c_z = z
        self.v_x = v_x
        self.v_y = v_y
        self.v_z = v_z

        return
