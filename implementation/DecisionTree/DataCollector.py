import csv
from typing import TYPE_CHECKING,  Dict
from implementation.data_classes import AdaptationTechnique, FailureType
from implementation.vehicle.controller import ControllerType
from matplotlib import pyplot as pyp
if TYPE_CHECKING:
    from implementation.data_classes import EnvironmentKnowledge, OtherVehicle, SimulationState
    from implementation.platoon_controller.knowledge.knowledge import Knowledge
    from implementation.vehicle.vehicles import ManagedVehicle

class DataCollector(object):

    def __init__(self, knowledge: "Knowledge", ego_vehicle: "ManagedVehicle"):

        self.header = []
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle

        self.knowledge: "Knowledge" = knowledge
        self.data_file = open(f"decision_tree_data_set_{self.ego_vehicle.role_name}.csv", "w", encoding="UTF8")
        print(self.data_file)
        self.writer = csv.writer(self.data_file)
        print("Opened data file")
        self.distance = []
        self.desired_distance = []
        self.distance_error = []

        self.ego_speed = []
        self.front_speed = []
        self.leader_speed = []

        self.leader_brake = []
        self.leader_throttle = []
        self.front_brake = []
        self.front_throttle = []
        self.ego_brake = []
        self.ego_throttle = []

        self.ego_acceleration = []
        self.front_acceleration = []
        self.leader_acceleration = []

        self.current_controller = []
        self.is_safe = []

        self.timestamp = []

        self.record_data: bool = False

        header = ["Controller", "CMACC", "CMDEC",
                 "Ego acc", "SpeedDifF", "SpeedDifL",
                 "OverLimit",
                 "DistError", "EDF",
                 "FSF", "Front acc", "FAF", "Front throttle", "Front brake", "FrOvLi",
                 "LSF", "Leader acc", "LAF", "Leader throttle", "Leader brake",
                 "Max acc", "max dec", "max throttle", "max brake",
                 "Technique"]
        self.writer.writerow(header)

    def run_step(self, data: "EnvironmentKnowledge", sim_state: "SimulationState") -> AdaptationTechnique:

        technique = ""
        technique_result = self.classify_data(data)
        if sim_state.classify:
            if sim_state.no_adap:
                technique = "NO"
            elif sim_state.par_adap:
                technique = "PA"
            elif sim_state.struc_adap:
                technique = "SA"
            elif sim_state.com_adap:
                technique = "CA"
        else:
            if technique_result == AdaptationTechnique.NO_ADAPTATION:
                technique = "NO"
            elif technique_result == AdaptationTechnique.PARAMETER:
                technique = "PA"
            elif technique_result == AdaptationTechnique.STRUCTURAL:
                technique = "SA"
            elif technique_result == AdaptationTechnique.CONTEXT:
                technique = "CA"
        if self.record_data:
            self.write_data(data, technique)
        print(f"{self.ego_vehicle.role_name}, classified technique: {technique}")
        return technique_result

    def write_data(self, data: "EnvironmentKnowledge", technique: str) -> None:

        timestamp = data.timestamp

        max_dec = "%.4f" % data.max_dec
        max_acc = "%.4f" % data.max_acc
        max_throttle = "%.4f" % data.max_throttle
        max_brake = "%.4f" % data.max_brake

        if self.knowledge.front_vehicle_id in data.other_vehicles:
            front_vehicle = data.other_vehicles[self.knowledge.front_vehicle_id]
            if front_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                front_vehicle_acc = float("%.4f" % front_vehicle.acceleration_tuple[0])
            else:
                front_vehicle_acc = float("%.4f" % front_vehicle.measured_acceleration_tuple[0])
            if front_vehicle.speed_tuple[1] == FailureType.no_failure:
                front_vehicle_speed = float("%.4f" % front_vehicle.speed_tuple[0])
            else:
                front_vehicle_speed = float("%.4f" % front_vehicle.measured_speed_tuple[0])
            front_vehicle_throttle = float("%.4f" % front_vehicle.throttle)
            front_vehicle_brake = float("%.4f" % front_vehicle.brake)
            fvs_failure = front_vehicle.speed_tuple[1]
            fva_failure = front_vehicle.acceleration_tuple[1]
        else:
            front_vehicle_acc = 0
            front_vehicle_speed = 100
            front_vehicle_throttle = -1
            front_vehicle_brake = -1
            fvs_failure = FailureType.omission
            fva_failure = FailureType.omission

        front_over_limit = data.front_over_limit

        if self.knowledge.leader_id in data.other_vehicles:
            leader_vehicle = data.other_vehicles[self.knowledge.leader_id]
            if leader_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                leader_acc = float("%.4f" % leader_vehicle.acceleration_tuple[0])
            else:
                leader_acc = float("%.4f" % leader_vehicle.measured_acceleration_tuple[0])
            if leader_vehicle.speed_tuple[1] == FailureType.no_failure:
                leader_speed = float("%.4f" % leader_vehicle.speed_tuple[0])
            else:
                leader_speed = float("%.4f" % leader_vehicle.measured_speed_tuple[0])
            leader_throttle = float("%.4f" % leader_vehicle.throttle)
            leader_brake = float("%.4f" % leader_vehicle.brake)
            lvs_failure = leader_vehicle.speed_tuple[1]
            lva_failure = leader_vehicle.acceleration_tuple[1]
        else:
            leader_acc = -1
            leader_speed = -1
            leader_throttle = -1
            leader_brake = -1
            lvs_failure = FailureType.omission
            lva_failure = FailureType.omission

        ego_speed = float("%.4f" % data.ego_speed_tuple[0])
        ego_acc = float("%.4f" % data.ego_acceleration_tuple[0])

        speed_diff_front = float("%.4f" % data.speed_diff_to_front)
        speed_diff_leader = float("%.4f" % data.speed_diff_to_leader)
        speed_over_limit = float("%.4f" % data.speed_over_limit)

        distance_to_front = float("%.4f" % data.ego_distance_tuple[0])
        current_controller: "ControllerType" = self.knowledge.current_controller
        speed_limit = data.speed_limit

        con_max_acc = self.knowledge.cont_max_acc
        con_max_dec = self.knowledge.cont_max_dec

        des_distance = float("%.4f" % data.desired_distance)
        distance_error = float("%.4f" % data.distance_error)
        edf = data.ego_distance_tuple[1]

        safety_space = 2 * distance_to_front * 11
        speed_square = ego_speed ** 2 - front_vehicle_speed ** 2

        is_safe = safety_space > speed_square

        ego_control = self.ego_vehicle.ego_vehicle.get_control()

        self.distance.append(distance_to_front)
        self.desired_distance.append(des_distance)
        self.distance_error.append(des_distance - distance_to_front)

        self.ego_speed.append(ego_speed)
        self.leader_speed.append(leader_speed)
        self.front_speed.append(front_vehicle_speed)

        self.front_brake.append(front_vehicle_brake)
        self.front_throttle.append(front_vehicle_throttle)
        self.leader_brake.append(leader_brake)
        self.leader_throttle.append(leader_throttle)
        self.ego_throttle.append(ego_control.throttle)
        self.ego_brake.append(ego_control.brake)

        self.ego_acceleration.append(ego_acc)
        self.leader_acceleration.append(leader_acc)
        self.front_acceleration.append(front_vehicle_acc)

        self._append_controller(current_controller)
        self._append_safety_state(is_safe)
        print(f"{self.ego_vehicle.role_name} is safe: ", is_safe)

        self.timestamp.append(timestamp.elapsed_seconds)

        if current_controller == ControllerType.SPEED:
            current_controller: str = "Speed"
        elif current_controller == ControllerType.BRAKE:
            current_controller: str = "Brake"
        elif current_controller == ControllerType.DISTANCE:
            current_controller: str = "Distance"

        write_data = [current_controller, con_max_acc, con_max_dec,
                      ego_acc, speed_diff_front, speed_diff_leader, speed_over_limit,
                      distance_error, edf,
                      fvs_failure, front_vehicle_acc, fva_failure, front_vehicle_throttle, front_vehicle_brake, front_over_limit,
                      lvs_failure, leader_acc, lva_failure, leader_throttle, leader_brake,
                      max_acc, max_dec, max_throttle, max_brake,
                      technique]
        self.writer.writerow(write_data)

    def classify_data(self, data: "EnvironmentKnowledge") -> AdaptationTechnique:

        current_controller: "ControllerType" = self.knowledge.current_controller
        over_limit = data.speed_over_limit
        edf = data.ego_distance_tuple[1]
        speed_dif = data.speed_diff_to_front
        controller_max_acc = self.knowledge.cont_max_acc
        front_over_limit = data.front_over_limit
        dist_error = data.distance_error

        if self.knowledge.front_vehicle_id in data.other_vehicles:
            front_vehicle = data.other_vehicles[self.knowledge.front_vehicle_id]
        else:
            front_vehicle = None

        if front_vehicle is not None:
            fsf = front_vehicle.speed_tuple[1]
            faf = front_vehicle.acceleration_tuple[1]
            front_brake = front_vehicle.brake
            if data.other_vehicles[data.front_vehicle_id].acceleration_tuple[1] == FailureType.no_failure:
                acc_front = data.other_vehicles[data.front_vehicle_id].acceleration_tuple[0]
            else:
                acc_front = data.other_vehicles[data.front_vehicle_id].measured_acceleration_tuple[0]
        else:
            fsf = FailureType.omission
            faf = FailureType.omission
            acc_front = 0
            front_brake = -1

        technique: AdaptationTechnique = None

        if current_controller == ControllerType.SPEED:
            # PS2
            if edf == FailureType.no_front_vehicle:
                technique = AdaptationTechnique.NO_ADAPTATION
            elif dist_error < 2 and (front_over_limit < 3 or over_limit < 3):
                # front vehicle is driving in acceptable speed limit deviation
                technique = AdaptationTechnique.STRUCTURAL
            else:
                # front vehicle is driving too fast
                technique = AdaptationTechnique.NO_ADAPTATION

        elif current_controller == ControllerType.DISTANCE:

            # no front vehicle
            if edf == FailureType.no_front_vehicle:
                technique = AdaptationTechnique.STRUCTURAL

            # all data available, but in ACC mode
            elif controller_max_acc == 2:
                if (fsf == FailureType.no_failure) and \
                        (faf == FailureType.no_failure):
                    technique = AdaptationTechnique.PARAMETER
                # missing parameter
            elif controller_max_acc == 2.75:
                if (fsf == FailureType.omission) or \
                        (faf == FailureType.omission):
                    technique = AdaptationTechnique.PARAMETER

            if over_limit > 3:
                technique = AdaptationTechnique.STRUCTURAL
            elif front_over_limit > 3:
                technique = AdaptationTechnique.STRUCTURAL

            # S0
            if speed_dif >= 0:
                # C3
                if dist_error < -0.75:
                    print("S0, C3, switch to brake")
                    technique = AdaptationTechnique.STRUCTURAL
                # C0 - C2
            # S1
            elif 0 > speed_dif >= -(5 / 3.6):
                # C0, C1
                if dist_error >= -0.1:
                    if acc_front <= -2:
                        technique = AdaptationTechnique.STRUCTURAL
                # C2
                elif -0.1 > dist_error >= -0.2:
                    print("S1, C2")
                    technique = AdaptationTechnique.STRUCTURAL
                # C3
                else:
                    print("S1, C3")
                    technique = AdaptationTechnique.STRUCTURAL
            # S2
            elif (-5 / 3.6) > speed_dif >= (-10 / 3.6):
                # C0
                if dist_error >= 0:
                    # no adaptation
                    pass
                # C1
                elif 0 > dist_error >= -0.1:
                    print("S2, C1")
                    technique = AdaptationTechnique.STRUCTURAL
                # C2
                elif -0.1 > dist_error >= -0.3:
                    print("S2, C2")
                    technique = AdaptationTechnique.STRUCTURAL
                # C3
                else:
                    technique = AdaptationTechnique.CONTEXT
            # S3
            elif speed_dif < -10 / 3.6:
                # C0, C1
                if dist_error > -0.4:
                    print("S3, C01")
                    technique = AdaptationTechnique.STRUCTURAL
                elif dist_error > 0.2:
                    technique = AdaptationTechnique.NO_ADAPTATION
                # C2, C3
                else:
                    technique = AdaptationTechnique.CONTEXT

            if dist_error < 0 and acc_front < -9:
                technique = AdaptationTechnique.CONTEXT

            if dist_error > 2 and front_over_limit < 0:
                technique = AdaptationTechnique.STRUCTURAL

        elif current_controller == ControllerType.BRAKE:
            if edf == FailureType.no_front_vehicle:
                technique = AdaptationTechnique.STRUCTURAL
            elif dist_error > 0.2 and speed_dif > 3 and front_brake <= 0:
                technique = AdaptationTechnique.STRUCTURAL
            elif dist_error < 0 and front_brake > 0.9 or acc_front < -9:
                technique = AdaptationTechnique.PARAMETER
        else:
            technique = AdaptationTechnique.NO_ADAPTATION

        if technique is None:
            technique = AdaptationTechnique.NO_ADAPTATION

        return technique

    def terminate(self):
        self.data_file.close()
        print("Closed data file")
        self.plot_distance()

    def plot_distance(self):

        self.ego_speed = [float(line) for line in self.ego_speed]
        self.front_speed = [float(l) for l in self.front_speed]
        self.leader_speed = [float(l) for l in self.leader_speed]

        self.desired_distance = [float(dis) for dis in self.desired_distance]
        self.distance = [float(dis) for dis in self.distance]
        self.distance_error = [float(dis) for dis in self.distance_error]

        self.front_brake = [float(val) for val in self.front_brake]
        self.front_throttle = [float(val) for val in self.front_throttle]
        self.leader_brake = [float(val) for val in self.leader_brake]
        self.leader_throttle = [float(val) for val in self.leader_throttle]

        self.ego_acceleration = [float(val) for val in self.ego_acceleration]
        self.front_acceleration = [float(val) for val in self.front_acceleration]
        self.leader_acceleration = [float(val) for val in self.leader_acceleration]


        """
        pyp.plot(self.timestamp, self.ego_speed, label="Ego Speed")
        pyp.plot(self.timestamp, self.front_speed, label="Front speed")
        pyp.plot(self.timestamp, self.leader_speed, label="Leader Speed")
        pyp.xlabel("Timestamp")
        pyp.ylabel("Speed")
        pyp.legend()
        pyp.gcf().autofmt_xdate()
        """

        fig1, ax = pyp.subplots(3 , 2) # figsize=(20, 20))
        pyp.subplots_adjust(hspace=0.35)
        ax[0, 0].plot(self.timestamp, self.desired_distance, label="Desired distance")
        ax[0, 0].plot(self.timestamp, self.distance, label="Distance")
        ax[0, 0].plot(self.timestamp, self.distance_error, label="Distance error")
        ax[0, 0].set_xlabel("Time (s)")
        ax[0, 0].set_ylabel("Distance [m]")
        ax[0, 0].set_title(f"Distance comparison {self.ego_vehicle.role_name}")
        ax[0, 0].legend()

        ax[0, 1].plot(self.timestamp, self.ego_speed, label=f"Speed {self.ego_vehicle.role_name}")
        ax[0, 1].plot(self.timestamp, self.leader_speed, label="Speed Leader")
        if self.ego_vehicle.role_name == "Follower_1":
            ax[0, 1].plot(self.timestamp, self.front_speed, label="Speed Front")
        ax[0, 1].set_xlabel("Time (s)")
        ax[0, 1].set_ylabel("Speed [m/s]")
        ax[0, 1].set_title(f"Speed comparison {self.ego_vehicle.role_name}")
        ax[0, 1].legend()

        if self.ego_vehicle.role_name == "Follower_1":
            ax[1, 0].plot(self.timestamp, self.front_brake, label="Brake value front")
            ax[1, 0].plot(self.timestamp, self.front_throttle, label="Throttle value front")
        ax[1, 0].plot(self.timestamp, self.leader_brake, label="Brake value Leader")
        ax[1, 0].plot(self.timestamp, self.leader_throttle, label="Throttle value Leader")
        ax[1, 0].plot(self.timestamp, self.ego_brake, label="Brake value ego vehicle")
        ax[1, 0].plot(self.timestamp, self.ego_throttle, label="Throttle value ego vehicle")
        ax[1, 0].set_xlabel("Time (s)")
        ax[1, 0].set_ylabel("Throttle/Brake value")
        ax[1, 0].set_title(f"Throttle/ Brake comparison {self.ego_vehicle.role_name}")
        ax[1, 0].legend()

        ax[1, 1].plot(self.timestamp, self.ego_acceleration, label=f"Acceleration {self.ego_vehicle.role_name}")
        if self.ego_vehicle.role_name == "Follower_1":
            ax[1, 1].plot(self.timestamp, self.front_acceleration, label="Acceleration Front")
        ax[1, 1].plot(self.timestamp, self.leader_acceleration, label="Acceleration Leader")
        ax[1, 1].set_xlabel("Time (s)")
        ax[1, 1].set_ylabel("Acceleration in [m/s²]")
        ax[1, 1].set_title(f"Acceleration comparison {self.ego_vehicle.role_name}")
        ax[1, 1].legend()

        ax[2, 0].set_yticklabels(["Speed", "Distance", "Brake"])
        ax[2, 0].plot(self.timestamp, self.current_controller)
        ax[2, 0].set_xlabel("Time (s)")
        ax[2, 0].set_ylabel("Current Controller")
        ax[2, 0].set_title(f"Current controller {self.ego_vehicle.role_name}")
        ax[2, 0].set_yticks([0, 1, 2])

        ax[2, 1].set_yticklabels(["", "Safe", "Unsafe", ""])
        ax[2, 1].set_yticks([-1, 0, 1, 2])
        ax[2, 1].plot(self.timestamp, self.is_safe)
        ax[2, 1].set_xlabel("Time (s)")
        ax[2, 1].set_title(f"Safety state {self.ego_vehicle.role_name}")


        pyp.savefig(f"data_plot_{self.ego_vehicle.role_name}")

        pyp.show()


    def __get_max_throttle(self, other_vehicles: Dict[int, "OtherVehicle"]) -> float:

        max_throttle = 0
        for vehicle_data in other_vehicles.values():
            if vehicle_data.throttle is not None:
                if vehicle_data.throttle > max_throttle:
                    max_throttle = vehicle_data.throttle
        return max_throttle

    def __get_max_brake(self, other_vehicles: Dict[int, "OtherVehicle"]) -> float:

        max_brake = 0
        for vehicle_data in other_vehicles.values():
            if vehicle_data.brake is not None:
                if vehicle_data.brake > max_brake:
                    max_brake = vehicle_data.brake
        return max_brake

    def __get_max_deceleration(self, other_vehicles: Dict[int, "OtherVehicle"]) -> float:

        max_dec = 0
        for vehicle_data in other_vehicles.values():
            if vehicle_data.acceleration_tuple[0] is not None:
                if vehicle_data.acceleration_tuple[0] < max_dec:
                    max_dec = vehicle_data.acceleration_tuple[0]
            elif vehicle_data.measured_acceleration_tuple[0]:
                if vehicle_data.measured_acceleration_tuple[0] < max_dec:
                    max_dec = vehicle_data.measured_acceleration_tuple[0]
        return max_dec

    def __get_max_acceleration(self, other_vehicles: Dict[int, "OtherVehicle"]) -> float:

        max_acc = 0
        for vehicle_data in other_vehicles.values():
            if vehicle_data.acceleration_tuple[0] is not None:
                if vehicle_data.acceleration_tuple[0] > max_acc:
                    max_acc = vehicle_data.acceleration_tuple[0]
            elif vehicle_data.measured_acceleration_tuple[0]:
                if vehicle_data.measured_acceleration_tuple[0] > max_acc:
                    max_acc = vehicle_data.measured_acceleration_tuple[0]
        return max_acc

    def _append_controller(self, controller: ControllerType) -> None:
        if controller == ControllerType.SPEED:
            self.current_controller.append(0)
        elif controller == ControllerType.DISTANCE:
            self.current_controller.append(1)
        elif controller == ControllerType.BRAKE:
            self.current_controller.append(2)

    def _append_safety_state(self, state: bool) -> None:
        if state:
            self.is_safe.append(0)
        else:
            self.is_safe.append(1)
