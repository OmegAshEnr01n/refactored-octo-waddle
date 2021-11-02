import os
import glob
import sys
import random
import matplotlib.pyplot as plt
import time
import numpy as np
import argparse

try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

def main():

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    # print(client.get_available_maps())
    world = client.load_world('Town03')

    blueprintLibrary = world.get_blueprint_library()
    # vehicleL = blueprintLibrary.filter('vehicle.*.*')
    # for v in vehicleL:
    #     print(v)
    car = blueprintLibrary.filter('vehicle.mercedes.coupe_2020')[0]
    transform = carla.Transform(carla.Location(x=120, y=120, z=40), carla.Rotation(yaw=180))
    car_spn = world.spawn_actor(car, transform)



if __name__ == '__main__':
    main()