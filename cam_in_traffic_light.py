import glob
import os
import sys
import random
from time import sleep
from numpy import array
from cv2 import imshow, waitKey
from geohash import encode, decode
import carla

actor_list = []
IM_WIDTH = 640
IM_HEIGHT = 480

def process_img(image):
    i = array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    imshow("", i3)
    waitKey(1)
    return i3 / 255.0

try:
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)

    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter("model3")[0]
    print(bp)

    spawn_point = random.choice(world.get_map().get_spawn_points())

    traffic_light = world.get_actors()[47]

    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle.set_autopilot(True)

    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))

    cam_bp = blueprint_library.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    cam_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
    cam_bp.set_attribute("fov", "110")

    traffic_light_location = traffic_light.get_location()
    tl_latitude, tl_longitude = traffic_light_location

    # Check if both objects are close
    traffic_light_hash = encode(tl_latitude, tl_longitude)

    # spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
    spawn_point = carla.Transform(traffic_light_location)
    print(spawn_point)

    sensor = world.spawn_actor(cam_bp, spawn_point, attach_to=traffic_light)
    actor_list.append(sensor)
    sensor.listen(lambda data: process_img(data))

    sleep(120)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
