#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
from carla import Client, VehicleControl, Location, Transform
from random import choice, sample, seed
from time import sleep, time
from numpy import array, max as npmax, random, float32, ones, argmax
from cv2 import imshow, waitKey
from math import sqrt
from collections import deque
from keras.applications.xception import Xception
from keras.layers import Dense, GlobalAveragePooling2D
from tensorflow.keras.optimizers import Adam
from keras.models import Model

import tensorflow as tf
from tensorflow.compat.v1.keras.backend import set_session
from tensorflow.random import set_seed
from tensorflow.compat.v1 import GPUOptions, Session, ConfigProto
# from keras.backend.tensorflow_backend import set_session
# from threading import Threading
from tqdm import tqdm


SHOW_PREVIEW = False
IM_WIDTH = 640
IM_HEIGHT = 480
SECONDS_PER_EPISODE = 10
REPLAY_MEMORY_SIZE = 5_000
MIN_REPLAY_MEMORY_SIZE = 1_000
MINIBATCH_SIZE = 16
PREDICTION_BATCH_SIZE = 1
TRAINING_BATCH_SIZE = MINIBATCH_SIZE // 4
UPDATE_TARGET_EVERY = 5
MODEL_NAME = "Xception"

MEMORY_FRACTION = 0.6
MIN_REWARD = -200

EPISODES = 100

DISCOUNT = 0.99
epsilon = 1
EPSILON_DECAY = 0.95
MIN_EPSILON = 0.001

AGGREGATE_STATS_EVERY = 10


class CarEnv:
    SHOW_CAM = SHOW_PREVIEW
    STEER_AMT = 1.0
    im_width = IM_WIDTH
    im_height = IM_HEIGHT
    front_camera = None

    def __init__(self):
        self.client = Client("localhost", 2000)
        self.client.set_timeout(2.0)
        self.world = self.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.model_3 = blueprint_library.filter("model3")[0]

    def reset(self):
        self.collision_hist = []
        self.actor_list = []

        self.transform = choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(self.model_3, self.transform)
        self.actor_list.append(self.vehicle)

        self.rgb_cam = self.blueprint_library.find("sensor.camera.rgb")
        self.rgb.set_attribute("image_size_x", f"{self.im_width}")
        self.rgb.set_attribute("image_size_y", f"{self.im_height}")
        self.rgb.set_attribute("fov", "100")

        transform = Transform(Location(x=2.5, z=0.7))
        self.sensor = self.world.spawn_actor(self.rgb_cam, transform, attach_to=self.vehicle)
        self.sensor.listen(lambda data: self.process_img(data))

        self.vehicle.apply_control(VehicleControl(throttle=0.0, brake=0.0))
        sleep(4)

        colsensor = self.blueprint_library.find("sensor.other.collision")
        self.colsensor = self.world.spawn_actor(colsensor, transform, attach_to=self.vehicle)
        self.actor_list.append(self.colsensor)
        self.colsensor.listen(lambda event: self.collision_data(event))

        while self.front_camera is None:
            sleep(0.01)

        self.episode_start = time()
        self.vehicle.apply_control(VehicleControl(throttle=0.0, brake=0.0))

        return self.front_camera

    def collision_data(self, event):
        self.collision_hist.append(event)
        

    def process_img(image):
        i = array(image.raw_data)
        i2 = i.reshape((self.im_height, self.im_width, 4))
        i3 = i2[:, :, :3]
        if self.SHOW_CAM:
            imshow("", i3)
            waitKey(1)

        self.front_camera = i3

    def step(self, action):
        if action == 0:
            self.vehicle.apply_control(VehicleControl(throttle=1.0, steer=-1 * self.STEER_AMT))
        elif action == 1:
            self.vehicle.apply_control(VehicleControl(throttle=1.0, steer=0))
        elif action == 2:
            self.vehicle.apply_control(VehicleControl(throttle=1.0, steer=1 * self.STEER_AMT))

        v = self.VEHICLE.GET_VELOCITY()
        kmh = int(3.6 * sqrt(v.x**2 + v.y**2 + v.z**2))

        if len(self.collision_hist) != 0:
            done = True
            reward = -200
        elif kmh < 50:
            done = False
            reward = -1
        else:
            done = False
            reward = 1


        if self.episode_start + SECONDS_PER_EPISODE < time():
            done = True

        return self.front_camera, reward, done, None


class DQAgent:
    def __init__(self):
        self.model = self.create_model()
        self.target_model = self.create_model()
        self.target_model.set_weights(self.model.get_weights())

        self.replay_memory = deque(maxlen=REPLAY_MEMORY_SIZE)

        self.tensorboard = ModifiedTensorBoard(log_dir=f"logs/{MODEL_NAME}-{int(time())}")
        self.target_update_counter = 0
        self.graph = tf.get_default_graph()

        self.terminate = False
        self.last_logged_episode = 0

        self.training_initialized = False

    def create_model(self):
        base_model = Xception(weights=None, include_top=False, input_shape=(IM_HEIGHT, IM_WIDTH, 3))

        x = base_model.output
        x = GlobalAveragePooling2D()(x)

        predictions = Dense(3, activation="linear")(x)
        model = Model(input=base_model.input, outputs=predictions)
        model.compile(loss="mse", optimizer=Adam(lr=0.001), metrics=["accuracy"])

        return model

    def update_replay_memory(self, transition):
        self.replay_memory.append(transition)

    def train(self):
        if len(self.replay_memory) < MIN_REPLAY_MEMORY_SIZE:
            return

        minibatch = sample(self.replay_memory, MINIBATCH_SIZE)
        new_current_states = array([transition[0] for transition in minibatch]) / 255
        with self.grath.as_default():
            future_qs_lsit = self.target_model.predict(new_current_states, PREDICTION_BATCH_SIZE)

        x = y = []

        for index, (current_state, action, reward, new_state, done) in enumerate(minibatch):
            if not done:
                max_future_q = npmax(future_qs_list[index])
                new_q = reward + DISCOUNT * max_future_q
            else:
                new_q = reward

            current_qs = current_qs_list[index]
            current_qs[action] = new_q

            x.append(current_state)
            y.append(current_qs)

        log_this_step = False
        if self.tensorboard.step > self.last_logged_episode:
            log_this_step = True
            self.last_log_episode = self.tensorboard.step

        with self.graph.as_default():
            self.model.fit(array(X) / 255, array(y), batch_size=TRAINING_BATCH_SIZE, verbose=0, shuffle=False, callbacks=[self.tensorboard] if log_this_step else None)

        if log_this_step:
            self.target_update_counter += 1

        if self.target_update_counter > UPDATE_TARGET_EVERY:
            self.target_model.set_weights(self.model.get_weights())
            self.target_update_counter = 0

    def get_qs(self, state):
        return self.model.predict(array(state).reshape(-1 * state.shape) / 255)[0]

    def train_in_loop(self):
        x = random.uniform(size=(1, IM_HEIGHT, IM_WIDTH, 3)).astype(float32)
        y = random.uniform(size=(1, 3)).astype(float32)

        with self.graph.as_default():
            self.model.fit(X, y, verbose=False, batch_size=1)

        self.training_initialized = True

        while True:
            if self.terminate:
                return
            self.train()
            sleep(0.01)

if __name__ == "__main__":
    FPS = 20
    ep_rewards = [-200]
    seed(1)
    random.seed(1)
    set_seed(1)

    gpu_options = GPUOptions(per_process_gpu_memory_fraction=MEMORY_FRACTION)
    set_session(Session(config=ConfigProto(gpu_options=gpu_options)))

    if not os.path.isdir("models"):
        os.makedirs("models")

    agent = DQAgent()
    env = CarEnv()

    trainer_thread = Thread(target=agent.train_in_loop, daemon=True)
    trainer_thread.start()

    while not agent.training_initialized:
        sleep(0.01)

    agent.get_qs(ones((env.IM_HEIGHT, env.IM_WIDTH, 3)))

    for episody in tqdm(range(1, EPSISODES + 1), ascii=True, unit="episodes"):
        env.collision_hist = []
        agent.tensorboard.step = episode
        episode_reward = 0
        step = 1
        current_state = env.reset()
        done = False
        episode_start = time()

        while True:
            if random.random() > epsilon:
                action = argmax(agent.get_qs(current_state))
            else:
                action = random.randint(0, 3)
                sleep(1/FPS)

            new_state, reward, done, _ = env.step(action)

            epsode_reward += reward

            agent.update_replay_memory((current_state, action, reward, new_state, done))
            step += 1

            if done:
                break

            for actor in env.actor_list:
                actor.destroy()

            # Append episode reward to a list and log stats (every given number of episodes)
            ep_rewards.append(episode_reward)
            if not episode % AGGREGATE_STATS_EVERY or episode == 1:
                average_reward = sum(ep_rewards[-AGGREGATE_STATS_EVERY:])/len(ep_rewards[-AGGREGATE_STATS_EVERY:])
                min_reward = min(ep_rewards[-AGGREGATE_STATS_EVERY:])
                max_reward = max(ep_rewards[-AGGREGATE_STATS_EVERY:])
                agent.tensorboard.update_stats(reward_avg=average_reward, reward_min=min_reward, reward_max=max_reward, epsilon=epsilon)

                # Save model, but only when min reward is greater or equal a set value
                if min_reward >= MIN_REWARD:
                    agent.model.save(f'models/{MODEL_NAME}__{max_reward:_>7.2f}max_{average_reward:_>7.2f}avg_{min_reward:_>7.2f}min__{int(time.time())}.model')

            # Decay epsilon
            if epsilon > MIN_EPSILON:
                epsilon *= EPSILON_DECAY
                epsilon = max(MIN_EPSILON, epsilon)

        # Set termination flag for training thread and wait for it to finish
        agent.terminate = True
        trainer_thread.join()
        agent.model.save(f'models/{MODEL_NAME}__{max_reward:_>7.2f}max_{average_reward:_>7.2f}avg_{min_reward:_>7.2f}min__{int(time.time())}.model')

    
