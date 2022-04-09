#!/usr/bin/env python3

from __future__ import annotations

import sys
from enum import Enum, auto
from math import sqrt
from random import randint, uniform
from typing import Tuple, Set

import pygame


class Warehouse:
    robots: Set[Robot]
    coalitions: Set[RobotCoalition]
    boxes_left: Set[Box]

    def __init__(self, width=100, height=100):
        self.width = width
        self.height = height
        self.robots = set()
        self.boxes_left = set()
        self.coalitions = set()

    def generate_random_boxes(self, boxes_num):
        self.boxes_left = set()
        for num in range(boxes_num):
            self.boxes_left.add(
                Box(uniform(0, self.width), uniform(0, self.height),
                    (uniform(0, self.width), uniform(0, self.height)),
                    randint(0, 400), randint(1, 4)))

    def generate_random_robots(self, robots_num: int):
        self.robots = set()
        for num in range(robots_num):
            self.robots.add(
                Robot(uniform(0, self.width),
                      uniform(0, self.height),
                      battery_level=uniform(0, 100)))

    def visualize(self):
        raise NotImplementedError

    def step(self, time_step=0.1):
        # move robots being part of coalitions
        for c in self.coalitions:
            c.update_states()
        for r in self.robots:
            r.move(time_step)
        # TODO move robots outside of coallitions


class WarehouseVisualizer:
    LIGHT_GRAY = (224, 224, 224)
    BLACK = (0, 0, 0)
    RED = (255, 0, 0)
    GREEN = (0, 200, 0)
    BLUE = (0, 0, 255)

    BOX_SIZE = 10
    ROBOT_SIZE = 4

    def __init__(self, warehouse: Warehouse, title: str = "Warehouse"):
        self.x = warehouse.width
        self.y = warehouse.height
        self.warehouse = warehouse
        self.display = pygame.display.set_mode((self.x, self.y), )
        self.display.fill(self.LIGHT_GRAY)
        pygame.display.set_caption(title)

    def show(self):
        self.display.fill(self.LIGHT_GRAY)
        self._draw_boxes()
        self._draw_robots()

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            pygame.display.update()
            keys = pygame.key.get_pressed()
            if keys[pygame.K_SPACE]:
                break

    def _draw_boxes(self):
        for box in self.warehouse.boxes_left:
            self._draw_square(box.x, box.y, self.GREEN)
            x2, y2 = box.target
            self._draw_line(box.x, box.y, x2, y2, self.GREEN)

    def _draw_robots(self):
        for r in self.warehouse.robots:
            self._draw_circle(r.x, r.y, self.ROBOT_SIZE, self.RED)
            if r.target is not None:
                x2, y2 = r.target
                self._draw_line(r.x, r.y, x2, y2, self.GREEN)

        #TODO add robots numbers when multiple robots in the same place

    def _draw_square(self, x_center: float, y_center: float, color):
        pygame.draw.rect(
            self.display, color,
            pygame.Rect(x_center - self.BOX_SIZE / 2,
                        y_center - self.BOX_SIZE / 2, self.BOX_SIZE,
                        self.BOX_SIZE))

    def _draw_circle(self, x, y, radius, color) -> None:
        pygame.draw.circle(self.display, color, (x, y), radius)

    def _draw_line(self, x1, y1, x2, y2, color) -> None:
        pygame.draw.line(self.display, color, (x1, y1), (x2, y2))


class WarehouseObject:
    """represents any object which can be located inside of the Warehouse
    """
    x: float
    y: float

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_distance_to(self, x, y):
        return sqrt((x - self.x)**2 + (y - self.y)**2)


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
    DISCHARGE_SPEED = 0.05

    def __init__(self,
                 x=None,
                 y=None,
                 capacity: int = 100,
                 battery_level: int = None):
        super().__init__(x, y)
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
            self._update_battery(distance)
        else:
            percentage = covered_distance / distance
            self.x = self.x + percentage * (self.x - target_x)
            self.y = self.y + percentage * (self.y - target_y)
            self._update_battery(covered_distance)

    def _reach_target(self, target_x, target_y):
        self.x = target_x
        self.y = target_y
        self.target = None
        if self.state is self.RobotState.DRIVING_EMPTY:
            self.state = self.RobotState.WAITING
        if self.state is self.RobotState.DRIVING_LOADED:
            self.state = self.RobotState.IDLE

    def _update_battery(self, covered_distance):
        self.battery_level = self.battery_level - covered_distance * self.DISCHARGE_SPEED


class Box(WarehouseObject):

    def __init__(self,
                 x,
                 y,
                 target_location: Tuple[int, int],
                 mass: int = 100,
                 points_of_support: int = 1):
        super().__init__(x, y)
        self.mass = mass
        self.points_of_support = points_of_support
        self.target = target_location


class RobotCoalition:
    robots: Set[Robot]

    class State(Enum):
        ASSEMBLING = auto()
        DELIVERING = auto()
        DISASSEMBLED = auto()

    def __init__(self, robots: set, box: Box) -> None:
        self.robots = robots
        self.box = box
        self.target = box.target
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