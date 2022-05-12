#!/usr/bin/env python3

from re import sub
from simulation.base import Warehouse, RobotCoalition
import numpy as np
from time import sleep
import matplotlib.pyplot as plt

counter=0
data=[]
data_time=[]
cur_data=[]
cur_time=[]
h=10
for step in range(50):
    size= step/h + 1
    size=int(size)
    w1 = Warehouse(200, 100, True)
    w1.generate_random_boxes(20*size)
    w1.generate_random_robots(40 *size)
    print(size)

    chargedRobots=[]
    r = list(w1.robots)[0:len(w1.robots)]
    box = list(w1.boxes_left)[0:len(w1.boxes_left)]

    totalEnergy=0
    while len(chargedRobots) < len(w1.robots):
        chargedRobots=[]
        for i in range(0,len(w1.robots)):
            if r[i].battery_level>=20:
                chargedRobots.append(r[i])
            else:
                r[i].battery_level += 20

    globaltasks=0
    step=0
    while len(w1.boxes_left)>0:
        step += 1
        chargedRobots=[]
        r = list(w1.robots)[0:len(w1.robots)]
        box = list(w1.boxes_left)[0:len(w1.boxes_left)]
        for i in range(0,len(w1.robots)):
            if r[i].battery_level>=20:
                chargedRobots.append(r[i])
            else:
                r[i].battery_level += 20
        leader=np.empty((len(w1.boxes_left),2))
        potentialCoalition=np.zeros((len(w1.boxes_left),3))
        potential=[]
        tasks=0
        for i in range(0,len(w1.boxes_left)):
            minDistance=10000
            for j in range(0,len(chargedRobots)):
                if box[i].get_distance_to(chargedRobots[j].x,chargedRobots[j].y)<minDistance:
                    if j not in leader:
                        minDistance=box[i].get_distance_to(chargedRobots[j].x,chargedRobots[j].y)
                        leader[i][0]=i
                        leader[i][1]=j
        leaders=[row[1] for row in leader]
        leaders = list(dict.fromkeys(leaders))
        robotsEnergy=np.zeros(((len(chargedRobots)-len(leaders)),2))
        preferableCoalition=np.zeros(((len(chargedRobots)-len(leaders)),2))
        for i in range(0,len(w1.boxes_left)):
            sublist=[leader[i][1]]
            k=0
            for j in range(0,len(chargedRobots)):
                if j not in leaders:
                    robotsEnergy[k][0]=j
                    robotsEnergy[k][1]=box[i].get_distance_to(chargedRobots[j].x,chargedRobots[j].y)*0.05+box[i].get_distance_to(box[i].target[0],box[i].target[1])*0.001*box[i].mass/box[i].points_of_support
                    k += 1
            if len(robotsEnergy)>=box[i].points_of_support-1:
                energies=[row[1] for row in robotsEnergy]
                for i in range(0,box[i].points_of_support-1):
                    value=min(energies)
                    idx=energies.index(value)
                    sublist.append(robotsEnergy[idx][0])
                    energies[idx]=100
                sublist.sort()
            potential.append(sublist)

        value=0
        for i in range(0,len(chargedRobots)):
            if i not in leaders:
                if not any(i in sublist for sublist in potential):
                    value += 1
        preferableCoalition=np.zeros(((len(chargedRobots)-len(leaders)-value),2))
        z=0
        
        for i in range(0,len(chargedRobots)):
            gain=0
            if i not in leaders:
                for j in range(0,len(w1.boxes_left)):
                    if i in potential[j]:
                        if box[j].utility/box[j].points_of_support>=gain:
                            gain=box[j].utility/box[j].points_of_support
                            preferableCoalition[z][0]=i
                            preferableCoalition[z][1]=leader[j][0]
                if gain != 0:
                    z += 1
        finalCoalition = set()
        for j in range(0,len(w1.boxes_left)):
            coalition=[]
            coalition.append(leader[j][1])
            for i in range(0,(len(chargedRobots)-len(leaders)-value)):
                if preferableCoalition[i][1]==j:
                    coalition.append(preferableCoalition[i][0])
                    coalition.sort()
            if coalition in potential:
                task=list(leader[j])[0]
                box1 = list(w1.boxes_left)[int(task)]
                r1=[]
                for i in coalition:
                    r1.append(chargedRobots[int(i)])
                energyUsed=0
                for k in range(len(r1)):
                    energyUsed += box1.get_distance_to(r1[k].x,r1[k].y)*0.05+box1.get_distance_to(box1.target[0],box1.target[1])*0.001*box1.mass/box1.points_of_support
                totalEnergy += energyUsed
                finalCoalition.add(RobotCoalition(set(r1), box1))
                tasks += 1
        globaltasks += tasks
        
        w1.coalitions = finalCoalition
        print(f'tasks done: {globaltasks}')
        while len(w1.boxes_done) != globaltasks:
            w1.step(10)
            sleep(0.00001)

        print(f'Finished in: {w1.passed_time}s')
        print("press Enter to exit")

    else:
        print(f"Calkowita energia: {totalEnergy}")
        print('koniec')
        if counter!=h:
            cur_data.append(totalEnergy)
            cur_time.append(w1.passed_time)
            counter += 1
            print(len(cur_data))
        else:
            data.append(cur_data)
            data_time.append(cur_time)
            cur_data=[]
            cur_time=[]
            cur_data.append(totalEnergy)
            cur_time.append(w1.passed_time)
            counter=1

data.append(cur_data)
data_time.append(cur_time)
fig = plt.figure(figsize =(10, 7))
ax = fig.add_axes([0, 0, 1, 1])
bp = ax.boxplot(data)
plt.show()

print(data_time)
fig2 = plt.figure(figsize =(10, 7))
ax2 = fig2.add_axes([0, 0, 1, 1])
bp2 = ax2.boxplot(data_time)
plt.show()


