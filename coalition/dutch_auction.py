#!/usr/bin/env python3

from __future__ import annotations

from simulation.base import Robot, Box
from typing import Set, List


class RobotTaskPair():
    summary_coalition_cost_energy = 0
    summary_coalition_cost_value = 0

    def __init__(self, robot_cost: List[RobotCost], box: Box):
        self.robot_cost = robot_cost
        self.robot = [r.robot for r in self.robot_cost]
        self.box = box
        self.id = 2  # int
        self.sum_cost = 0

        for r in self.robot_cost:
            self.sum_cost = self.sum_cost + \
                r.robot_energy_cost

    def __str__(self):
        text = f"Coalition id: {self.id}\n"

        worst_time = 0
        for r in self.robot_cost:
            text = text + \
                f'Energy cost: {r.robot_energy_cost}, Cost: {r.cost}\n'
            self.summary_coalition_cost_energy = self.summary_coalition_cost_energy + \
                r.robot_energy_cost
            self.summary_coalition_cost_value = self.summary_coalition_cost_value + r.cost
            try:
                if worst_time < r.task_time:
                    worst_time = r.task_time
            except:
                pass

        text = text + \
            f'Summary energy cost: {self.summary_coalition_cost_energy}, Cummary cost: {self.summary_coalition_cost_value}\n'
        text = text + f'Box profitability: {self.box.profitability}\n'
        text = text + f'Task time: {worst_time}\n'

        return text

    def calculate_energy(self):
        temp = 0
        for r in self.robot_cost:
            temp = temp + \
                r.robot_energy_cost

        return temp


class RobotCost():

    ''' Robot wrapper class 
    '''

    # TODO: Temporary energy coefficients hardcoded; fix this!

    COF_ENE = 0.05
    COF_LOADED = 0.001

    EMPTY_SPEED = 1
    LOADED_SPEED = 0.5

    ''' 
    Weighted cost of a robot; 
    
    robot_cost * [WEIGHT_1] + robot_energy_cost * [WEIGHT_2]'''
    #robot_weighted_cost: float

    def __init__(self, robot: Robot, initial_cost, epsilon):
        self.robot = robot
        self.cost = initial_cost + epsilon
        self.robot_weighted_cost = float
        self.robot_energy_cost = float
        self.robot_available = True
        self.task_time = float

    def update_cost(self) -> None:
        pass

    ''' Calculate  energy cost'''

    def energy_cost(self, box: Box) -> None:
        x_target, y_target = box.target
        self.robot_energy_cost = self.robot.get_distance_to(box.x, box.y)*self.COF_ENE + \
            box.get_distance_to(x_target, y_target) * \
            (self.COF_ENE+box.points_of_support*self.COF_LOADED)

    def time_cost(self, box: Box) -> None:
        x_target, y_target = box.target
        self.task_time = (self.robot.get_distance_to(box.x, box.y)*self.EMPTY_SPEED +
                          box.get_distance_to(x_target, y_target) * self.LOADED_SPEED)

    def weighted_robot_cost(self, weight: List[float, float]) -> None:
        self.robot_weighted_cost = self.cost * \
            weight[0] + self.robot_energy_cost * weight[1]


class DutchAuction:
    robots: List[Robot]
    profitability_box_list: List[Box]

    robots_costs = []

    task_pairs = []

    robot_recharged = []

    '''epsilon - auction's constant
    '''

    def __init__(self, robots: List[Robot], boxes: Set[Box], cost_weights: List[float, float], epsilon=1.0, robots_capacity=100):
        self.robots = robots
        self.epsilon = epsilon
        self.robots_capacity = robots_capacity

        self.profitability_box_list = sorted(
            boxes, key=lambda box: box.profitability, reverse=True)

        self.cost_weights = cost_weights

    def start_auction(self):
        ''' Remove robot whose battery level is below or equal critical'''
        self._discar_flat_robots()

        ''' Calculate every robot's cost '''
        self.robots_costs = [RobotCost(
            robot, self.profitability_box_list[0].profitability, self.epsilon) for robot in self.robots]

        #copy_robots_cost = self.robots_costs

        while True:
            # Updating Robot Cost per Service
            if self._cost_decrement():
                break

            # Perform Bidding
            for box_task in self.profitability_box_list:
                ''' Lower the cost of the most expensive robot '''

                for robot in self.robots_costs:
                    robot.energy_cost(box_task)
                    robot.time_cost(box_task)

                # while True:
                ''' Create precoalition and check if precoalition is suitable for the box/task'''
                pre_coalition = self._create_pre_coalition(box_task)

                if pre_coalition is not None:
                    robot_temp_class = [
                        wrapper_class.robot for wrapper_class in pre_coalition]

                    for robot in pre_coalition:
                        self.robots_costs.remove(robot)

                    self.profitability_box_list.remove(box_task)

                    self.task_pairs.append(
                        RobotTaskPair(pre_coalition, box_task))

                    if len(self.robots_costs) <= 4 or len(self.profitability_box_list) == 0:
                        return self.task_pairs


    def _discar_flat_robots(self) -> None:
        for robot in self.robot_recharged:
            self.robots.append(robot)
        
        # if len(self.robots) != 20:
        #     print("aa")

        self.robot_recharged = []

        for robot in self.robots:
            if robot.battery_level <= robot.critical_battery_level:
                robot.battery_level = 100
                self.robot_recharged.append(robot)
                self.robots.remove(robot)
        
        

    def _calculate_robot_needed(self) -> None:
        """ Calculate how many robots are needed to complete given task (push particular box to the desired destination) """
        for box in self.profitability_box_list:
            box.num_of_robot_needed = int(box.mass/self.robots_capacity) + \
                1 if box.mass % self.robots_capacity != 0 else 0


    def _change_robot_cost(self, weights) -> None:
        for robot in self.robots_costs:
            robot.weighted_robot_cost(weights)

        self.robots_costs = sorted(
            self.robots_costs, key=lambda robot: robot.robot_weighted_cost, reverse=True)

        self.robots_costs[0].cost = self.robots_costs[0].cost-self.epsilon

        if self.robots_costs[0].cost == 0:
            self.robots_costs.remove(self.robots_costs[0])

    def _cost_decrement(self):
        # Updateing Robot Cost per Servise
        self.robots_costs = sorted(
            self.robots_costs, key=lambda robot: robot.cost, reverse=True)

        index_number = 0
        while True:
            if index_number<len(self.robots_costs)-1:
                if self.robots_costs[index_number].robot_available is True:
                    break
                else:
                    index_number = index_number+1
            else:
                index_number=0
                for robot in self.robots_costs:
                    robot.cost = 11 # hardcoded
                return True

        # Price decrement
        self.robots_costs[index_number].cost = self.robots_costs[index_number].cost-self.epsilon

        if self.robots_costs[index_number].cost == 0:
            self.robots_costs[index_number].robot_available = False

        return False

    def _create_pre_coalition(self, box: Box) -> List:
        if len(self.robots) > box.points_of_support:

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
