#!/usr/bin/env python3

from simulation.base import *

print(1)
w1 = Warehouse(200, 100, True)
w1.generate_random_boxes(10)
w1.generate_random_robots(14)
print("press Enter to exit")
c = input()