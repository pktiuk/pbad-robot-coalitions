#!/usr/bin/env python3

from enum import Enum, auto
from typing import ClassVar, Tuple, Set

from __future__ import annotations


class Warehouse:
    coalitions: Set[RobotCoalition]

    def __init__(self, width=100, height=100):
        self.width = width
        self.height = height
        self.robots = set()
        self.boxes_left = set()
        self.coalitions = set()

    def generate_random_boxes(self, boxes_num):
        raise NotImplementedError

    def generate_random_robots(self, robots_num: int):
        raise NotImplementedError

    def visualize(self):
        raise NotImplementedError

    def step(self, time_step=0.1):
        # move robots being part of coalitions
        for c in self.coalitions:
            c.update_states()
        # TODO move robots outside of coallitions


class WarehouseObject:
    """represents any object which can be located inside of the Warehouse
    """

    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


class Robot(WarehouseObject):

    class RobotState(Enum):
        IDLE = auto()
        WAITING = auto()
        DRIVING_EMPTY = auto()
        DRIVING_LOADED = auto()
        CHARGING = auto()

    EMPTY_SPEED = 1
    LOADED_SPEED = 0.5

    def __init__(self,
                 x=None,
                 y=None,
                 parent: Warehouse = None,
                 capacity: int = 100,
                 battery_level: int = None):
        super().__init__(x, y, parent)
        self.battery_level = battery_level
        self.capacity = capacity
        self.state = self.RobotState.IDLE
        self.target = None  #Tuple[x,y]


class Box(WarehouseObject):

    def __init__(self,
                 x=None,
                 y=None,
                 parent: Warehouse = None,
                 mass: int = 100,
                 points_of_support: int = 1):
        super().__init__(x, y, parent)
        self.mass = mass
        self.points_of_support = points_of_support


class RobotCoalition:

    class State(Enum):
        ASSEMBLING = auto()
        DELIVERING = auto()
        DISASSEMBLED = auto()

    def __init__(self, robots: set, box: Box,
                 target_location: Tuple[int, int]) -> None:
        self.robots = robots
        self.box = box
        self.x_target, self.y_target = target_location
        self.x = self.box.x
        self.y = self.box.y
        self.state = self.ASSEMBLING

    def update_states(self):
        """Updates state of coallition (targets of robots)
        """
        raise NotImplementedError