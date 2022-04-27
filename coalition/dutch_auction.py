#!/usr/bin/env python3

from __future__ import annotations

from simulation.base import Robot, Box
from typing import Set, List


class RobotCost():

    ''' Robot wrapper class 
    '''

    # TODO: Temporary energy coefficients hardcoded; fix this!

    COF_ENE = 2
    COF_LOADED = 1

    robot_energy_cost: float

    def __init__(self, robot: Robot, initial_cost, epsilon):
        self.robot = robot
        self.cost = initial_cost + epsilon

    def update_cost(self) -> None:
        pass

    ''' Calculate  '''
    def energy_cost(self, box: Box) -> None:
        x_target, y_target = box.target
        self.robot_energy_cost = self.robot.get_distance_to(box.x, box.y)*self.COF_ENE + \
            box.get_distance_to(x_target, y_target) * \
            (self.COF_ENE+box.points_of_support*self.COF_LOADED)


class DutchAuction:
    robots: List[Robot]
    profitability_box_list: List[Box]

    robots_costs = []

    '''epsilon - auction's constant
    '''

    def __init__(self, robots: List[Robot], boxes: Set[Box], epsilon=1.0, robots_capacity=100):
        self.robots = robots
        self.epsilon = epsilon
        self.robots_capacity = robots_capacity

        self.profitability_box_list = sorted(
            boxes, key=lambda box: box.profitability, reverse=True)

    def start_auction(self):
        ''' Remove robot whose battery level is below or equal critical'''
        self.discar_flat_robots()

        for box_task in self.profitability_box_list:
            ''' Calculate every robot's cost '''
            self.create_robots_queue_list(
                box_task.profitability)

            ''' Calculate how many robots are needed to move the box '''
            # self.calculate_robot_needed()

            # while(True):
            ''' Lower the cost of the most expensive robot '''
            self.change_robot_cost()

            ''' Create precoalition and check if precoalition is suitable for the box/task'''
            pre_coalition = self.create_pre_coalition(box_task)
            pass

    def discar_flat_robots(self) -> None:
        for robot in self.robots:
            if robot.battery_level <= robot.critical_battery_level:
                self.robots.remove(robot)

    def create_robots_queue_list(self, profitability: float) -> None:
        for robot in self.robots:
            self.robots_costs.append(
                RobotCost(robot, profitability, self.epsilon))

    def calculate_robot_needed(self) -> None:
        """ Calculate how many robots are needed to complete given task (push particular box to the desired destination) """
        for box in self.profitability_box_list:
            box.num_of_robot_needed = int(box.mass/self.robots_capacity) + \
                1 if box.mass % self.robots_capacity != 0 else 0

    def change_robot_cost(self) -> None:
        self.robots_costs = sorted(
            self.robots_costs, key=lambda robot: robot.cost, reverse=True)

        self.robots_costs[0].cost = self.robots_costs[0].cost-self.epsilon

    def create_pre_coalition(self, box: Box) -> List:
        if len(self.robots) > box.points_of_support:
            for robot in self.robots_costs:
                robot.energy_cost(box)

            energy_cost_list = sorted(
                self.robots_costs, key=lambda robot_energy_c: robot_energy_c.robot_energy_cost)

            pre_coalition = energy_cost_list[0:box.points_of_support]

            pre_coalition_cost = 0
            for coalition_member in pre_coalition:
                pre_coalition_cost = pre_coalition_cost+coalition_member.cost

            if box.profitability >= pre_coalition_cost:
                # TODO: Add robots removing
                return pre_coalition

            return None
