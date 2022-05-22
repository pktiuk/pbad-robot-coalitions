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

temp_pairs_sum = 0

counter = 0

while len(w1.boxes_left) > 0:
    coalition = DutchAuction(w1.robots, w1.boxes_left, cost_weights=cost_weights)

    coalition.task_pairs = []

    tasks_pairs = coalition.start_auction()

    print(f"############# ZADANIE NUMER: {counter}########################")
    counter = counter +1

    for pair in tasks_pairs:
        print(pair)

    pair_set = [RobotCoalition(set(pair.robot), pair.box) for pair in tasks_pairs]

    temp_pairs_sum = temp_pairs_sum + len(tasks_pairs)

    coal1 = set(pair_set)
    w1.coalitions = coal1


    while len(w1.boxes_done) < temp_pairs_sum:
        w1.step(6)
        sleep(0.01)

    

print(f'Finished in: {w1.passed_time}s')
print("press Enter to exit")
c = input()
