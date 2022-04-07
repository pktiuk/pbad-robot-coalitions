#!/usr/bin/env python3

from enum import Enum, auto
from math import pow, sqrt
from typing import ClassVar, Tuple, Set

from __future__ import annotations


class Warehouse:
    coalitions: Set[RobotCoalition]
    robots: Set[Robot]

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
        for r in self.robots:
            r.move(time_step)
        # TODO move robots outside of coallitions


class WarehouseObject:
    """represents any object which can be located inside of the Warehouse
    """

    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


class Robot(WarehouseObject):
    target: Tuple[float, float]

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

    def move(self, movement_duration) -> None:
        """Moves robot in direction depending on current state and target.
        Does nothing when there is no need to move
        """
        speed = None
        if self.state is self.RobotState.DRIVING_EMPTY:
            speed = self.EMPTY_SPEED
        elif self.state is self.RobotState.DRIVING_LOADED:
            speed = self.LOADED_SPEED
        else:
            return

        covered_distance = movement_duration * speed
        target_x, target_y = self.target
        distance = self.get_distance_to(target_x, target_y)
        if covered_distance > distance:
            self._reach_target(target_x, target_y)
        else:
            percentage = covered_distance / distance
            self.x = self.x + percentage * (self.x - target_x)
            self.y = self.y + percentage * (self.y - target_y)

    def _reach_target(self, target_x, target_y):
        self.x = target_x
        self.y = target_y
        self.target = None
        if self.state is self.RobotState.DRIVING_EMPTY:
            self.state = self.RobotState.WAITING
        if self.state is self.RobotState.DRIVING_LOADED:
            self.state = self.RobotState.IDLE

    def get_distance_to(self, x, y):
        return sqrt((x - self.x)**2 + (y - self.y)**2)


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
    robots: Set[Robot]

    class State(Enum):
        ASSEMBLING = auto()
        DELIVERING = auto()
        DISASSEMBLED = auto()

    def __init__(self, robots: set, box: Box,
                 target_location: Tuple[int, int]) -> None:
        self.robots = robots
        self.box = box
        self.target = target_location
        self.state = self.State.ASSEMBLING
        self._set_robots_target((box.x, box.y), Robot.RobotState.DRIVING_EMPTY)

    def update_states(self):
        """Updates state and targets of coallition
        """
        if self.state is self.State.ASSEMBLING:
            assembled = True
            for r in self.robots:
                if r.state is r.RobotState.DRIVING_EMPTY:
                    assembled = False
                    break
            if assembled:
                self.state = self.State.DELIVERING
                self._set_robots_target(self.target,
                                        Robot.RobotState.DRIVING_LOADED)
        elif self.state is self.State.DELIVERING:
            #checking only one robot
            if self.robots[0].state is Robot.RobotState.IDLE:
                self.state = self.State.DISASSEMBLED

    def _set_robots_target(self, target: Tuple[float, float],
                           robot_state: Robot.RobotState):
        for r in self.robots:
            r.state = robot_state
            r.target = target