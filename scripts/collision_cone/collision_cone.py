#!/usr/bin/env python3
import sympy as sp
import numpy as np

class Obstacle:
    def __init__(self, is_long=True):
        self.c_x = 0
        self.c_y = 0
        self.is_long = is_long


def hello():
    print("hello world")
