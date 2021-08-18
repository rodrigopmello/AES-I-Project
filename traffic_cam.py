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
    sys.path.append("../carla/")
except IndexError:
    pass

import carla

from agents.navigation.controller import VehiclePIDController

actor_list = []

def sensor_callback(sensor_data, sensor_name):

    
    ts = str(time.time())    
    
    # sensor_data.save_to_disk(sensor_name+'-'+ts+".jpeg")
    time.sleep(0.05)
    sensor_data.save_to_disk(sensor_name+'-'+ts+".jpeg")

    


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()




    blueprint_library = world.get_blueprint_library()    
    actors_list = world.get_actors()    

    map = world.get_map()

    traffic_light = actors_list[47]
    print(traffic_light.get_transform())
    x = traffic_light.get_transform().location.x 
    y = traffic_light.get_transform().location.y
    z = traffic_light.get_transform().location.z + 2 
    
    cam_bp = blueprint_library.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', '1920')
    cam_bp.set_attribute('image_size_y', '1080')
    cam_bp.set_attribute('fov', '110')
    waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)
    filtered_waypoints = []
    current_road = 0
    for waypoint in waypoints:        
        print(waypoint.road_id)
        if current_road != waypoint.road_id:
            # time.sleep(1)
            pass
        # world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
        #                            color=carla.Color(r=255, g=0, b=0), life_time=50,
        #                            persistent_lines=True)

        current_road = waypoint.road_id
        if(waypoint.road_id == 8):
            filtered_waypoints.append(waypoint)
            world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=50,
                                   persistent_lines=True)

    # cam01 = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=x, y=y, z=z)))
    
    size = len(filtered_waypoints)
    transform = filtered_waypoints[size-1].transform.location
    print(transform)
    transform.x -= 10
    transform.z = 10.59
    transform.y -= 2.5

    # print('waypoint len' + str(len(filtered_waypoints))) 75
    cam01 = world.spawn_actor(cam_bp, carla.Transform(transform))
    #231.79937744140625,7.489245414733887,4.150000095367432
    cam02 = world.spawn_actor(cam_bp, carla.Transform(carla.Location(x=40, y=10.8, z=10.59)))
    cam01.listen(lambda data: sensor_callback(data, "camera01"))
    # cam02.listen(lambda data: sensor_callback(data, "camera02"))
    
    bp = blueprint_library.filter('model3')[0]
    
    
    world = client.get_world()
    spawnPoint=carla.Transform(carla.Location(x=38.6,y=5.8, z=0.598),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))
    spawnPoint2=carla.Transform(carla.Location(x=58.6,y=5.8, z=0.598),carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))
    spawn_point = filtered_waypoints[size-1].transform
    spawn_point.location.z += 2
    vehicle = client.get_world().spawn_actor(bp, spawn_point)
    

    #vehicle = world.spawn_actor(bp, spawnPoint)
    print(map.get_waypoint(vehicle.get_location()).road_id)
    waypoint = map.get_waypoint(vehicle.get_location())
    world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                   color=carla.Color(r=0, g=255, b=0), life_time=50,
                                   persistent_lines=True)

    
    # vehicle = world.spawn_actor(bp, spawnPoint2)

    custom_controller = VehiclePIDController(vehicle, args_lateral = {'K_P': 1, 'K_D': 0.0, 'K_I': 0}, args_longitudinal = {'K_P': 1, 'K_D': 0.0, 'K_I': 0.0})

    ticks_to_track = 10
    for i in range(ticks_to_track):
	    control_signal = custom_controller.run_step(1, filtered_waypoints[0])
	    vehicle.apply_control(control_signal)

    

    while(True):
        t = world.get_spectator().get_transform()
        # coordinate_str = "(x,y) = ({},{})".format(t.location.x, t.location.y)
        coordinate_str = "(x,y,z) = ({},{},{})".format(t.location.x, t.location.y,t.location.z)
        print (coordinate_str)
        time.sleep(10)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        for actor in actor_list:
            actor.destroy()
        print("All cleaned up!")
        print(' - Exited by user.')
