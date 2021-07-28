#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import time
from queue import Queue
from queue import Empty

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def sensor_callback(sensor_data, sensor_name):

    print("sensor data", sensor_data.raw_data)
    sensor_data.save_to_disk("pic1.jpeg")

    
    

def main():
    client = carla.Client('192.168.0.14', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()
    print('spawn points', spawn_points)

    blueprint_library = world.get_blueprint_library()
    

    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '1920')
    cam_bp.set_attribute('image_size_y', '1080')
    cam_bp.set_attribute('fov', '110')
    
    x = 36.346519470214844
    y = 4.937343120574951
    z = 1.0885553359985352
    cam01 = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=x, y=y, z=z)))
    cam01.listen(lambda data: sensor_callback(data, "camera01"))
	

    while(True):
        t = world.get_spectator().get_transform()
        # coordinate_str = "(x,y) = ({},{})".format(t.location.x, t.location.y)
        coordinate_str = "(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z)
        print (coordinate_str)
        time.sleep(5)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
