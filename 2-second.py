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

IM_WIDTH = 640
IM_HEIGHT = 480

def main():
    actor_list = []
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10)
        world = client.load_world('Town04')
        map = world.get_map()

        blueprintLib = world.get_blueprint_library()
        carBp = blueprintLib.filter('vehicle.mercedes.coupe_2020')[0]
        spawnpt = carla.Transform(carla.Location(x=0, y=0, z=10), carla.Rotation(pitch = 0, yaw = 180, roll = 0))
        car = world.spawn_actor(carBp, spawnpt)
        actor_list.append(car)

        car.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        blueprint = blueprintLib.find('sensor.camera.rgb')
        # change the dimensions of the image
        blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
        blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
        blueprint.set_attribute('fov', '110')

        spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
        sensor = world.spawn_actor(blueprint, spawn_point, attach_to=car)
        actor_list.append(sensor)


    finally:
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])



