#!/usr/bin/env python3

from time import sleep
from simulation import Warehouse, RobotCoalition

print(1)
w1 = Warehouse(200, 100, True)
w1.generate_random_boxes(10)
w1.generate_random_robots(14)
r = list(w1.robots)[0:2]
box = list(w1.boxes_left)[0]
coal = {RobotCoalition(set(r), box)}
w1.coalitions = coal

while len(w1.boxes_done) == 0:
    w1.step(0.1)
    sleep(0.01)

print(f'Finished in: {w1.passed_time}s')
print("press Enter to exit")
c = input()