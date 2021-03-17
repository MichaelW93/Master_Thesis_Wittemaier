#!/usr/bin/env python3

import carla
import random
import sys

from implementation.configuration_parameter import *
from implementation.carla_client.carla_steering_algorithm import CarlaSteeringAlgorithm
from implementation.monitor.carla_monitor import CarlaMonitor

class CarlaControlClient(object):

    def __init__(self):
        self.client = None
        self.carla_world = None
        self.settings = None
        self.raining = False

        self.leader_vehicle = None
        self.ego_vehicle = None
        self.leader_control_agent = None
        self.carla_steering_algorithm_ego = None
        self.carla_steering_algorithm_leader = None
        self.carla_ego_control = carla.VehicleControl()
        self.leader_control = carla.VehicleControl()

        self.carla_monitor = None

        self.scene_camera = None

        self.initialize_carla_client()
        self.spawn_leader()
        self.spawn_ego_vehicle()
        self.setup_camera()
        self.move_spectator()
        self.carla_world.tick()  # tick once, to update actors

        self.setup_lead_vehicle()
        self.setup_ego_vehicle()

        self.game_loop()

    def initialize_carla_client(self) -> None:
        """Creates the carla client, loads the world and sets the simulation to synchronous mode.
        :return:
        """

        self.client = carla.Client(CARLA_SERVER_HOST, CARLA_SERVER_PORT)
        self.client.load_world("Town05")
        self.carla_world = self.client.get_world()

        self.settings = self.carla_world.get_settings()
        self.settings.synchronous_mode = True
        self.settings.fixed_delta_seconds = 1 / CARLA_SERVER_FPS

        self.carla_world.apply_settings(self.settings)

    def setup_lead_vehicle(self) -> None:
        """Creates the steering algorithm for the leader vehicle"""
        self.carla_steering_algorithm_leader = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.leader_vehicle)

    def setup_ego_vehicle(self) -> None:
        self.carla_steering_algorithm_ego = CarlaSteeringAlgorithm(self.carla_world.get_map(), self.ego_vehicle)

    def move_spectator(self) -> None:
        """Sets the spectator to the same transform as the scene camera."""

        spectator = self.carla_world.get_spectator()
        spectator.set_transform(self.scene_camera.get_transform())

    def setup_camera(self) -> None:
        blueprint = self.create_camera_blueprint()
        transform = carla.Transform(
            carla.Location(SCENE_CAMERA_LOCATION_X, SCENE_CAMERA_LOCATION_Y, SCENE_CAMERA_LOCATION_Z),
            carla.Rotation(SCENE_CAMERA_ROTATION_PITCH, SCENE_CAMERA_ROTATION_YAW, SCENE_CAMERA_ROTATION_ROLL)
        )
        self.scene_camera = self.carla_world.spawn_actor(blueprint, transform, self.ego_vehicle)

    def create_camera_blueprint(self) -> carla.Blueprint:
        """Creates the carla blueprint for the scene camera
        :return carla.Blueprint
        """
        bp_library = self.carla_world.get_blueprint_library()
        blueprint = bp_library.find("sensor.camera.rgb")
        blueprint.set_attribute('image_size_x', str(PYGAME_WINDOW_WIDTH))
        blueprint.set_attribute('image_size_y', str(PYGAME_WINDOW_HEIGHT))

        blueprint.set_attribute('role_name', "scene_camera")
        blueprint.set_attribute('blur_amount', str(0.5))
        blueprint.set_attribute('motion_blur_intensity', str(0.225))
        blueprint.set_attribute('motion_blur_max_distortion', str(0.175))
        blueprint.set_attribute('motion_blur_min_object_screen_size', str(0.05))
        return blueprint

    def switch_weather(self) -> None:
        """Changes the weather between sunshine and rain
        :return:
        """
        if self.raining:
            self.carla_world.set_weather(carla.WeatherParameters.ClearNoon)
        else:
            self.carla_world.set_weather(carla.WeatherParameters.WetNoon)

    def spawn_leader(self) -> None:

        blueprint_library = self.carla_world.get_blueprint_library()
        spawn_point = carla.Transform()
        spawn_point.location.x = LEADER_SPAWN_LOCATION_X
        spawn_point.location.y = LEADER_SPAWN_LOCATION_Y
        spawn_point.location.z = LEADER_SPAWN_LOCATION_Z

        spawn_point.rotation.yaw = LEADER_SPAWN_ROTATION_YAW

        blueprint = random.choice(blueprint_library.filter('vehicle.bmw.*'))

        while self.leader_vehicle is None:
            self.leader_vehicle = self.carla_world.try_spawn_actor(blueprint, spawn_point)

    def spawn_ego_vehicle(self) -> None:
        blueprint_library = self.carla_world.get_blueprint_library()
        spawn_point = carla.Transform()
        spawn_point.location.x = EGO_SPAWN_LOCATION_X
        spawn_point.location.y = EGO_SPAWN_LOCATION_Y
        spawn_point.location.z = EGO_SPAWN_LOCATION_Z

        spawn_point.rotation.yaw = EGO_SPAWN_ROTATION_YAW

        blueprint = random.choice(blueprint_library.filter('vehicle.audi.*'))

        while self.ego_vehicle is None:
            self.ego_vehicle = self.carla_world.try_spawn_actor(blueprint, spawn_point)

    def game_loop(self) -> None:

        running = True
        while running:
            try:
                world_snapshot = self.carla_world.tick()

                if MOVE_SPECTATOR:
                    self.move_spectator()

                leader_control_command = None
                if self.leader_control_agent:
                    leader_control_command = self.leader_control_agent.run_step()
                if leader_control_command:
                    self.leader_vehicle.apply_control(leader_control_command)

                self.leader_control.steer = self.carla_steering_algorithm_leader.goToNextTargetLocation()
                self.leader_control.throttle = 0.5
                self.leader_vehicle.apply_control(self.leader_control)

                self.carla_ego_control.steer = self.carla_steering_algorithm_ego.goToNextTargetLocation()
                self.carla_ego_control.throttle = 0.6
                self.ego_vehicle.apply_control(self.carla_ego_control)

            except KeyboardInterrupt:
                running = False
                self.exit_client()
            except Exception as error:
                running = False
                import traceback
                print(error)
                traceback.print_exc()

        self.exit_client()

    def reset_simulation(self):
        if self.scene_camera is not None:
            self.scene_camera.destroy()
            self.scene_camera = None
        if self.ego_vehicle is not None:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None
        if self.leader_vehicle is not None:
            self.leader_vehicle.destroy()
            self.leader_vehicle = None
        if self.leader_control_agent is not None:
            self.leader_control_agent = None

    def exit_client(self):

        print("Exiting client")

        self.reset_simulation()

        settings = self.carla_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0  # set to 0 -> server default
        settings.no_rendering_mode = False
        self.carla_world.apply_settings(settings)
        self.carla_world.wait_for_tick()


if __name__ == '__main__':
    carla_control_client = None
    try:
        carla_control_client = CarlaControlClient()
    except Exception as e:
        print("Error executing Control Client.", e)
        import traceback
        import logging
        traceback.print_exc()
        logging.error(e)
    except KeyboardInterrupt as e:
        if carla_control_client:
            carla_control_client.exit_client()
        sys.exit()
    finally:
        if carla_control_client:
            carla_control_client.exit_client()
