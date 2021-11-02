import glob
import os
import sys
try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

import random
import time
import numpy as np
import imageio

import cv2

IM_WIDTH = 1920
IM_HEIGHT = 1080

# the problem here was the cv2 display. It was not displaying and causing the lag. Once removed code works fine
def process_img(image):
    i = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    i3 = i3[:, :, ::-1]
    print(i3)
    image.save_to_disk('_out/%08d' % image.frame)
    return i3/255.0

w = imageio.get_writer('_out/my_video.mp4', format='FFMPEG', mode='I', fps=30)

def process_img1(image):
    i = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    i3 = i3[:, :, ::-1]
    w.append_data(i3)
    return i3/255.0


actor_list = []
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter('model3')[0]
    print(bp)

    spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
    vehicle.set_autopilot(True)  # if you just wanted some NPCs to drive.

    actor_list.append(vehicle)

    # https://carla.readthedocs.io/en/latest/cameras_and_sensors
    # get the blueprint for this sensor
    blueprint = blueprint_library.find('sensor.camera.rgb')
    # change the dimensions of the image
    blueprint.set_attribute('image_size_x', f'{IM_WIDTH}')
    blueprint.set_attribute('image_size_y', f'{IM_HEIGHT}')
    blueprint.set_attribute('fov', '110')

    # Adjust sensor relative to vehicle
    spawn_point = carla.Transform(carla.Location(x=-5.5, z=1.7))

    # spawn the sensor and attach to vehicle.
    sensor = world.spawn_actor(blueprint, spawn_point, attach_to=vehicle)

    # add sensor to list of actors
    actor_list.append(sensor)

    # do something with this sensor
    
    sensor.listen(lambda data: process_img1(data))

    time.sleep(30)

finally:
    print('destroying actors')
    
    for actor in actor_list:
        actor.destroy()
    print('done.')
    time.sleep(5)
    w.close()