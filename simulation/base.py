#!/usr/bin/env python3

from __future__ import annotations

import sys
from enum import Enum, auto
from math import sqrt
from random import randint, uniform
from typing import Tuple, Set, List
import random

import pygame


class Warehouse:
    robots: List[Robot]
    coalitions: Set[RobotCoalition]
    boxes_left: Set[Box]
    boxes_done: Set[Box]

    def __init__(self, width=100, height=100, visualize: bool = False):
        self.width = width
        self.height = height
        self.robots = list()
        self.boxes_left = set()
        self.boxes_done = set()
        self.coalitions = set()
        self.passed_time = 0
        self.visualizer = None
        if visualize:
            self.visualizer = WarehouseVisualizer(self)
            self.update_visualization()

    def generate_random_boxes(self, boxes_num):
        self.boxes_left = set()
        i=0
        for num in range(boxes_num):
            random.seed(10+i)
            i=i+1
            self.boxes_left.add(
                Box(uniform(0, self.width), uniform(0, self.height),
                    (uniform(0, self.width), uniform(0, self.height)),
                    randint(0, 400), randint(1, 4)))
        self.update_visualization()

    def generate_random_robots(self, robots_num: int):
        self.robots = list()
        i=0
        for num in range(robots_num):
            random.seed(10+i)
            i=i+1
            self.robots.append(
                Robot(uniform(0, self.width),
                      uniform(0, self.height),
                      battery_level=uniform(0, 100)))
        self.update_visualization()

    def update_visualization(self):
        if self.visualizer is not None:
            self.visualizer.update()

    def step(self, time_step=0.1):
        for r in self.robots:
            r.move(time_step)
        coalitions_finished = set()
        for c in self.coalitions:
            c.update_states()
            if c.state is RobotCoalition.State.DISASSEMBLED:
                coalitions_finished.add(c)
                self.boxes_left.remove(c.box)
                self.boxes_done.add(c.box)
        self.coalitions = self.coalitions - coalitions_finished
        self.update_visualization()
        self.passed_time += time_step

    def is_done(self) -> bool:
        '''is simulation done'''
        return len(self.boxes_left) == 0

    def get_objects_by_loc(self,
                           x,
                           y,
                           max_radius=10
                           ) -> List[Tuple[float, WarehouseObject]]:
        '''gets objects in selected radius sorted from the nearest'''
        result = []
        objects = list(self.boxes_left) + self.robots
        for box in objects:
            dist = box.get_distance_to(x, y)
            if dist <= max_radius:
                result.append((round(dist, 2), box))

        sorting_fun = lambda x: x[0]
        result.sort(key=sorting_fun)
        return result


class WarehouseVisualizer:
    LIGHT_GRAY = (214, 214, 214)
    BLACK = (0, 0, 0)
    ROBOT_COLOR_CHARGED = (255, 50, 0)
    ROBOT_COLOR_DISCHARGED = (155, 30, 0)
    BOX_COLOR = (210, 190, 0)
    BLUE = (0, 0, 255)

    PX_PER_M = 6  #Pixels per meter ratio

    # Size in meters
    BOX_SIZE = 2
    ROBOT_SIZE = 1

    def __init__(self, warehouse: Warehouse, title: str = "Warehouse"):
        self.warehouse = warehouse
        self.display = pygame.display.set_mode(
            (warehouse.width * self.PX_PER_M,
             warehouse.height * self.PX_PER_M), )
        self.display.fill(self.LIGHT_GRAY)
        pygame.display.set_caption(
            f'{title} {warehouse.width}m x {warehouse.height}m')
        self.update()

    def update(self):
        self.display.fill(self.LIGHT_GRAY)
        self._draw_boxes()
        self._draw_robots()
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                x_px, y_px = event.pos
                x_m = x_px / self.PX_PER_M
                y_m = y_px / self.PX_PER_M
                print(f"Clicked:({x_m},{y_m})")
                nearest_objects = self.warehouse.get_objects_by_loc(
                    x_m, y_m, max_radius=3)
                for x in nearest_objects:
                    dist, obj = x
                    print(f"dist: {dist:.2} Obj: {obj}")

    def _draw_boxes(self):
        for box in self.warehouse.boxes_left:
            self._draw_square(box.x, box.y, self.BOX_COLOR)
            x2, y2 = box.target
            self._draw_line(box.x, box.y, x2, y2, self.BOX_COLOR)

    def _draw_robots(self):
        for r in self.warehouse.robots:
            if r.is_battery_critical():
                self._draw_circle(r.x, r.y, self.ROBOT_SIZE,
                                  self.ROBOT_COLOR_DISCHARGED)
            else:
                self._draw_circle(r.x, r.y, self.ROBOT_SIZE,
                                  self.ROBOT_COLOR_CHARGED)
            if r.target is not None:
                x2, y2 = r.target
                self._draw_line(r.x, r.y, x2, y2, self.ROBOT_COLOR_CHARGED)

        #TODO add robots numbers when multiple robots in the same place

    def _draw_square(self, x_center: float, y_center: float, color):
        pygame.draw.rect(
            self.display, color,
            pygame.Rect((x_center - self.BOX_SIZE / 2) * self.PX_PER_M,
                        (y_center - self.BOX_SIZE / 2) * self.PX_PER_M,
                        self.BOX_SIZE * self.PX_PER_M,
                        self.BOX_SIZE * self.PX_PER_M))

    def _draw_circle(self, x, y, radius, color) -> None:
        pygame.draw.circle(self.display, color,
                           (x * self.PX_PER_M, y * self.PX_PER_M),
                           radius * self.PX_PER_M)

    def _draw_line(self, x1, y1, x2, y2, color) -> None:
        pygame.draw.line(self.display, color,
                         (x1 * self.PX_PER_M, y1 * self.PX_PER_M),
                         (x2 * self.PX_PER_M, y2 * self.PX_PER_M))


class WarehouseObject:
    """represents any object which can be located inside of the Warehouse
    """
    x: float
    y: float

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return str(self.__dict__)

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
    DISCHARGE_SPEED_EMPTY = 0.05
    DISCHARGE_SPEED_PER_KG = 0.001
    BATTERY_CRITICAL_LEVEL = 20

    def __init__(self,
                 x=None,
                 y=None,
                 capacity: int = 100,
                 battery_level: int = None,
                 critical_battery_level: int = 20):
        super().__init__(x, y)
        self.battery_level = battery_level
        self.capacity = capacity
        self.state = self.RobotState.IDLE
        self.target = None  #Tuple[x,y]
        self.passed_distance = 0
        self.carried_mass = 0
        self.critical_battery_level = critical_battery_level

    def __str__(self):
        printed_dict = self.__dict__
        for k, v in printed_dict.items():
            if type(v) is float:
                printed_dict[k] = round(v, 3)
        return f"Robot {printed_dict}"


    def move(self, movement_duration) -> bool:
        """Moves robot in direction depending on current state and target.
        Does nothing when there is no need to move
        """
        if self.target is None:
            return
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
            self.passed_distance += distance
            self._update_battery(distance)
        else:
            percentage = covered_distance / distance
            self.x = self.x - percentage * (self.x - target_x)
            self.y = self.y - percentage * (self.y - target_y)
            self.passed_distance += covered_distance
            self._update_battery(covered_distance)

    def _reach_target(self, target_x, target_y):
        self.x = target_x
        self.y = target_y
        self.target = None
        if self.state is self.RobotState.DRIVING_EMPTY:
            self.state = self.RobotState.WAITING
        elif self.state is self.RobotState.DRIVING_LOADED:
            self.state = self.RobotState.IDLE
        self.carried_mass = 0
        #TODO add going to charging place

    def _update_battery(self, covered_distance):
        self.battery_level -= covered_distance * self.DISCHARGE_SPEED_EMPTY - covered_distance * self.carried_mass * self.DISCHARGE_SPEED_PER_KG

    def is_battery_critical(self) -> bool:
        return self.battery_level <= self.BATTERY_CRITICAL_LEVEL


class Box(WarehouseObject):
    #profitability: int

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
        self.profitability = random.randint(1, 10)
        self.num_of_robot_needed = None

    def __str__(self):
        return f"Box {self.__dict__}"



class RobotCoalition:
    robots: Set[Robot]

    class State(Enum):
        ASSEMBLING = auto()
        DELIVERING = auto()
        DISASSEMBLED = auto()

    def __init__(self, robots: set, box: Box) -> None:
        self.robots = set(robots)
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
            any_robot = list(self.robots)[0]
            if any_robot.state is Robot.RobotState.IDLE:
                self.state = self.State.DISASSEMBLED
            self.box.x = any_robot.x
            self.box.y = any_robot.y

    def _set_robots_target(self, target: Tuple[float, float],
                           robot_state: Robot.RobotState):
        for r in self.robots:
            r.state = robot_state
            r.target = target
            if robot_state is Robot.RobotState.DRIVING_LOADED:
                r.carried_mass = self.box.mass / len(self.robots)
