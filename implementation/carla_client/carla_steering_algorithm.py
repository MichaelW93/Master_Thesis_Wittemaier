import math
import sys

import carla


class CarlaSteeringAlgorithm(object):

    def __init__(self, map, vehicle):
        self.__map = map
        self.__vehicle = vehicle
        self.__next_waypoint = None

    def goToNextTargetLocation(self):
        ######
        # calculate front center position between the wheels
        ######

        # physic instance of the wheels has the position attribute from CARLA 0.9.6 onwards
        # wheels = self.__vehicle.get_physics_control().wheels
        # frontLeftWheel = wheels[0]
        # frontRightWheel = wheels[1]
        # current_location = (frontLeftWheel.position + frontRightWheel.position) / 2.0

        center_location = self.__vehicle.get_location()
        rotation = self.__vehicle.get_transform().rotation

        direction_x = math.cos(math.radians(rotation.yaw)) * math.cos(math.radians(rotation.pitch))
        direction_y = math.sin(math.radians(rotation.yaw)) * math.cos(math.radians(rotation.pitch))
        direction_z = math.sin(math.radians(rotation.pitch))
        forward_direction = carla.Vector3D(direction_x, direction_y, direction_z)
        normalized_forward_direction = self.__normalize(forward_direction)

        current_location = center_location
        # go 90% from the self.__vehicle center to the self.__vehicle front (last 10% would mean the bumper position)
        offset_factor = self.__vehicle.bounding_box.extent.x * 0.9
        current_location.x += normalized_forward_direction.x * offset_factor
        current_location.y += normalized_forward_direction.y * offset_factor
        current_location.z += normalized_forward_direction.z * offset_factor

        ######
        # get next target location
        ######

        # project to road is required to be True
        # False can result in none object if no corresponding waypoint is found
        current_waypoint = self.__map.get_waypoint(current_location, project_to_road=True,
                                                   lane_type=carla.LaneType.Driving)
        # print(current_waypoint)
        if self.__next_waypoint is not None:
            if current_waypoint == self.__next_waypoint:
                target = current_waypoint.next(8.0)
                self.__next_waypoint = None
            else:
                target = self.__next_waypoint
        else:
            target = current_waypoint.next(8.0)
        # for i in target:
        #     print(i)
        target_position = target[0].transform.location

        ######
        # calculate steering value based on the angle between target and current location
        ######

        target_diff_current = carla.Vector3D(target_position.x - current_location.x,
                                             target_position.y - current_location.y,
                                             target_position.z - current_location.z)
        direction = self.__normalize(target_diff_current)

        # print(direction)
        # print(normalized_forward_direction)

        direction_angle = self.__cartesian_to_spherical(direction).z
        self.__vehicle_angle = self.__cartesian_to_spherical(normalized_forward_direction).z
        # print(direction_angle)
        # print(self.__vehicle_angle)
        direction_angle = math.degrees(direction_angle)
        self.__vehicle_angle = math.degrees(self.__vehicle_angle)

        angle = direction_angle - self.__vehicle_angle
        # print("angle to adjust to", angle)
        if angle > 180:
            angle -= 360
        elif angle < -180:
            angle += 360

        steering = 0.0
        maximum_steering_angle = self.__vehicle.get_physics_control().wheels[0].max_steer_angle

        # it may be an improvement to include the decreased steering possibilities in higher speed ranges
        # steering_curve = self.__vehicle.get_physics_control().steering_curve
        # for i in steering_curve:
        #     print(i)

        if angle < -maximum_steering_angle:
            steering = -1.0
        elif angle > maximum_steering_angle:
            steering = 1.0
        else:
            steering += angle / maximum_steering_angle

        return steering

    def __normalize(self, vector):
        direction_value = math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)
        normalized_vector = carla.Vector3D(vector.x / direction_value,
                                           vector.y / direction_value,
                                           vector.z / direction_value)
        return normalized_vector

    def __cartesian_to_spherical(self, normalized_vector):
        ######
        # Check octant position (2D quadrant & above 0/0/0 layer or below)
        ######

        quadrant = -1
        if normalized_vector.x >= 0 and normalized_vector.y >= 0:
            quadrant = 1
        elif normalized_vector.x >= 0 and normalized_vector.y < 0:
            quadrant = 4
        elif normalized_vector.x < 0 and normalized_vector.y >= 0:
            quadrant = 2
        elif normalized_vector.x < 0 and normalized_vector.y < 0:
            quadrant = 3

        above = True
        if normalized_vector.z < 0:
            above = False

        ######
        # Calculate spherical coordinates for a normalized unit vector in the first quadrant
        ######
        if math.fabs(normalized_vector.x) < sys.float_info.epsilon:
            spherical_vector = carla.Vector3D(1, math.acos(math.fabs(normalized_vector.z)),
                                              math.copysign(1, normalized_vector.y) * math.pi / 2.0)
        else:
            spherical_vector = carla.Vector3D(1, math.acos(math.fabs(normalized_vector.z)),
                                              math.atan(math.fabs(normalized_vector.y) /
                                                        math.fabs(normalized_vector.x)))

        ######
        # Adjust the spherical coordinates to the correct octant value
        ######

        spherical_vector.y *= -1 if not above else 1

        if quadrant == 1:
            spherical_vector.z += 0.0
        elif quadrant == 2:
            spherical_vector.z += 2 * (0.5 * math.pi - spherical_vector.z)
        elif quadrant == 3:
            spherical_vector.z += math.pi
        elif quadrant == 4:
            spherical_vector.z += 2 * (0.5 * math.pi - spherical_vector.z) + math.pi

        return spherical_vector

    def set_next_waypoint(self, waypoint):
        self.__next_waypoint = waypoint
