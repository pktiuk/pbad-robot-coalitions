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
r2 = list(w1.robots)[3:5]
box2 = list(w1.boxes_left)[1]

test_robot = list(w1.robots)[0]
test_robot.battery_level=0

list(w1.robots)[0] = test_robot

coalition = DutchAuction(w1.robots, w1.boxes_left)

coalition.start_auction()


coal1 = {RobotCoalition(set(r1), box1), RobotCoalition(set(r2), box2)}
w1.coalitions = coal1

while len(w1.boxes_done) == 0:
    w1.step(0.1)
    sleep(0.01)

print(f'Finished in: {w1.passed_time}s')
print("press Enter to exit")
c = input()