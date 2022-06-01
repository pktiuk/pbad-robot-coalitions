import numpy as np

def save_energy(data):
    arr = np.array(data)
    np.savetxt("energy.txt", arr)

def save_time(data):
    arr = np.array(data)
    np.savetxt("time.txt", arr)

def load_energy():
    y = np.loadtxt("energy.txt")
    return y

def load_time():
    y = np.loadtxt("time.txt")
    return y