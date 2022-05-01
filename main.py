#!/usr/bin/env python3

from time import sleep

from simulation import Warehouse, RobotCoalition

from coalition.dutch_auction import DutchAuction


print(1)
w1 = Warehouse(200, 100, True)
w1.generate_random_boxes(10)
w1.generate_random_robots(14)
r1 = list(w1.robots)[0:2]
box1 = list(w1.boxes_left)[0]
#r2 = list(w1.robots)[3:5]
#box2 = list(w1.boxes_left)[1]

test_robot = list(w1.robots)[0]
test_robot.battery_level = 0

list(w1.robots)[0] = test_robot

cost_weights = [0.3, 0.7]

coalition = DutchAuction(w1.robots, w1.boxes_left, cost_weights=cost_weights)

col, box = coalition.start_auction()

coal1 = {RobotCoalition(set(col), box)}
w1.coalitions = coal1

while len(w1.boxes_done) == 0:
    w1.step(0.1)
    sleep(0.01)

print(f'Finished in: {w1.passed_time}s')
print("press Enter to exit")
c = input()
