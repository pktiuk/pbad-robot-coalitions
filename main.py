#!/usr/bin/env python3

from time import sleep

from simulation import Warehouse, RobotCoalition

from coalition.dutch_auction import DutchAuction, RobotTaskPair


w1 = Warehouse(200, 100, visualize=True)
w1.generate_random_boxes(50)
w1.generate_random_robots(100)
r1 = list(w1.robots)[0:2]
box1 = list(w1.boxes_left)[0]

cost_weights = [0.2, 0.8]

coalition = DutchAuction(w1.robots, w1.boxes_left, cost_weights=cost_weights)

tasks_pairs = coalition.start_auction()

for pair in tasks_pairs:
    print(pair)

pair_set = [RobotCoalition(set(pair.robot), pair.box) for pair in tasks_pairs]


coal1 = set(pair_set)
w1.coalitions = coal1

while True:  # len(w1.boxes_done) == 0:
    w1.step(0.1)
    sleep(0.01)

print(f'Finished in: {w1.passed_time}s')
print("press Enter to exit")
c = input()
