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
        self.timestamp = []

        self.record_data: bool = False

        header = ["Controller", "CMACC", "CMDEC",
                 "Ego speed", "Ego acc", "Ego dist", "SpeedDifF", "SpeedDifL",
                 "OverLimit",
                 "DesDist", "DistError",
                 "Front speed", "FSF", "Front acc", "FAF", "Front throttle", "Front brake",
                 "Leader speed", "LSF", "Leader acc", "LAF", "Leader throttle", "Leader brake",
                 "Max acc", "max dec", "max throttle", "max brake",
                 "Speed limit", "Technique"]
        self.writer.writerow(header)

    def run_step(self, data: "EnvironmentKnowledge", sim_state: "SimulationState") -> None:

        technique_result = ""
        if sim_state.classify:
            if sim_state.no_adap:
                technique_result = "NO"
            elif sim_state.par_adap:
                technique_result = "PA"
            elif sim_state.struc_adap:
                technique_result = "SA"
            elif sim_state.com_adap:
                technique_result = "CA"
        else:
            technique_result = self.classify_data(data)
            if technique_result == AdaptationTechnique.NO_ADAPTATION:
                technique_result = "NO"
            elif technique_result == AdaptationTechnique.PARAMETER:
                technique_result = "PA"
            elif technique_result == AdaptationTechnique.STRUCTURAL:
                technique_result = "SA"
            elif technique_result == AdaptationTechnique.CONTEXT:
                technique_result = "CA"
        if self.ego_vehicle.role_name == "Follower_1":
            print(technique_result)
            print(self.knowledge.current_controller)
        if self.record_data:
            self.write_data(data, technique_result)
        else:
            return

    def write_data(self, data: "EnvironmentKnowledge", technique: str) -> None:

        timestamp = data.timestamp

        max_dec = "%.4f" % data.max_dec
        max_acc = "%.4f" % data.max_acc
        max_throttle = "%.4f" % data.max_throttle
        max_brake = "%.4f" % data.max_brake

        if self.knowledge.front_vehicle_id in data.other_vehicles:
            front_vehicle = data.other_vehicles[self.knowledge.front_vehicle_id]
            if front_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                front_vehicle_acc = "%.4f" % front_vehicle.acceleration_tuple[0]
            else:
                front_vehicle_acc = "%.4f" % front_vehicle.measured_acceleration_tuple[0]
            if front_vehicle.speed_tuple[1] == FailureType.no_failure:
                front_vehicle_speed = "%.4f" % front_vehicle.speed_tuple[0]
            else:
                front_vehicle_speed = "%.4f" % front_vehicle.measured_speed_tuple[0]
            front_vehicle_throttle = "%.4f" % front_vehicle.throttle
            front_vehicle_brake = "%.4f" % front_vehicle.brake
            fvs_failure = front_vehicle.speed_tuple[1]
            fva_failure = front_vehicle.acceleration_tuple[1]
        else:
            front_vehicle_acc = 0
            front_vehicle_speed = 100
            front_vehicle_throttle = 0
            front_vehicle_brake = 0
            fvs_failure = FailureType.omission
            fva_failure = FailureType.omission

        if self.knowledge.leader_id in data.other_vehicles:
            leader_vehicle = data.other_vehicles[self.knowledge.leader_id]
            if leader_vehicle.acceleration_tuple[1] == FailureType.no_failure:
                leader_acc = "%.4f" % leader_vehicle.acceleration_tuple[0]
            else:
                leader_acc = "%.4f" % leader_vehicle.measured_acceleration_tuple[0]
            if leader_vehicle.speed_tuple[1] == FailureType.no_failure:
                leader_speed = "%.4f" % leader_vehicle.speed_tuple[0]
            else:
                leader_speed = "%.4f" % leader_vehicle.measured_speed_tuple[0]
            leader_throttle = "%.4f" % leader_vehicle.throttle
            leader_brake = "%.4f" % leader_vehicle.brake
            lvs_failure = leader_vehicle.speed_tuple[1]
            lva_failure = leader_vehicle.acceleration_tuple[1]
        else:
            leader_acc = -1
            leader_speed = -1
            leader_throttle = -1
            leader_brake = -1
            lvs_failure = FailureType.omission
            lva_failure = FailureType.omission

        ego_speed = "%.4f" % data.ego_speed_tuple[0]
        ego_acc = "%.4f" %  data.ego_acceleration_tuple[0]

        speed_diff_front = "%.4f" % data.speed_diff_to_front
        speed_diff_leader = "%.4f" % data.speed_diff_to_leader
        speed_over_limit = "%.4f" % data.speed_over_limit

        distance_to_front = "%.4f" % data.ego_distance_tuple[0]
        current_controller: "ControllerType" = self.knowledge.current_controller
        speed_limit = data.speed_limit

        con_max_acc = self.knowledge.cont_max_acc
        con_max_dec = self.knowledge.cont_max_dec

        des_distance = "%.4f" % data.desired_distance
        distance_error = "%.4f" % data.distance_error

        self.distance.append(distance_to_front)
        self.desired_distance.append(des_distance)
        self.timestamp.append(timestamp.elapsed_seconds)

        if current_controller == ControllerType.SPEED:
            current_controller: str = "Speed"
        elif current_controller == ControllerType.BRAKE:
            current_controller: str = "Brake"
        elif current_controller == ControllerType.DISTANCE:
            current_controller: str = "Distance"

        write_data = [current_controller, con_max_acc, con_max_dec,
                      ego_speed, ego_acc, distance_to_front, speed_diff_front, speed_diff_leader, speed_over_limit,
                      des_distance, distance_error,
                      front_vehicle_speed, fvs_failure, front_vehicle_acc, fva_failure, front_vehicle_throttle, front_vehicle_brake,
                      leader_speed, lvs_failure, leader_acc, lva_failure, leader_throttle, leader_brake,
                      max_acc, max_dec, max_throttle, max_brake,
                      speed_limit, technique]
        self.writer.writerow(write_data)

    def classify_data(self, data: "EnvironmentKnowledge") -> AdaptationTechnique:

        current_controller: "ControllerType" = self.knowledge.current_controller
        ego_speed = data.ego_speed_tuple[0]
        speed_limit = data.speed_limit
        distance_to_front = data.ego_distance_tuple[0]
        des_distance = data.desired_distance
        front_vehicle_speed = -1

        front_vehicle = None
        if self.knowledge.front_vehicle_id in data.other_vehicles:
            front_vehicle = data.other_vehicles[self.knowledge.front_vehicle_id]
            if front_vehicle.speed_tuple[1] == FailureType.no_failure:
                front_vehicle_speed = front_vehicle.speed_tuple[0]
            else:
                front_vehicle_speed = front_vehicle.measured_speed_tuple[0]

        if front_vehicle_speed is None:
            speed_dif = 100
        else:
            speed_dif = front_vehicle_speed - ego_speed

        technique = AdaptationTechnique.NO_ADAPTATION

        if current_controller == ControllerType.SPEED:
            # PS2
            if distance_to_front == -1:
                return AdaptationTechnique.NO_ADAPTATION
            if ego_speed < front_vehicle_speed > ((speed_limit + 5) / 3.6):
                return AdaptationTechnique.NO_ADAPTATION
            else:
                return AdaptationTechnique.STRUCTURAL

        elif current_controller == ControllerType.DISTANCE:

            # no front vehicle
            if distance_to_front == -1:
                return AdaptationTechnique.STRUCTURAL

            # PD0 is already covered from S0, same as PS0
            # PD1
            elif (2 * des_distance) > distance_to_front > des_distance:
                # PS1
                if ego_speed < front_vehicle_speed <= ((speed_limit + 4) / 3.6):
                    return AdaptationTechnique.STRUCTURAL
                # PS2
                elif ego_speed < front_vehicle_speed > ((speed_limit + 4)/ 3.6):
                    return AdaptationTechnique.STRUCTURAL
            # PD2
            elif distance_to_front > 2 * des_distance:
                # PS0
                if (speed_limit - 3)/3.6 < ego_speed < ((speed_limit + 3) / 3.6) and \
                        (front_vehicle_speed - 3) < ego_speed < (front_vehicle_speed + 3):
                    return AdaptationTechnique.NO_ADAPTATION
                if ego_speed < front_vehicle_speed - 3 <= ((speed_limit + 4) / 3.6):
                    return AdaptationTechnique.STRUCTURAL

            elif speed_limit + 5 < (ego_speed * 3.6):
                return AdaptationTechnique.STRUCTURAL
            # S0
            elif speed_dif >= 0:
                # C3
                if distance_to_front < (des_distance/2):
                    return AdaptationTechnique.STRUCTURAL
                # C0 - C2
                else:
                    return AdaptationTechnique.NO_ADAPTATION
            # S1
            elif 0 > speed_dif >= -(5/3.6):
                # C0, C1
                if distance_to_front >= (des_distance - 4):
                    return AdaptationTechnique.NO_ADAPTATION
                # C2
                elif des_distance > distance_to_front >= (des_distance/2):
                    return AdaptationTechnique.STRUCTURAL
                # C3
                else:
                    return AdaptationTechnique.STRUCTURAL
            # S2
            elif (-5/3.6) > speed_dif >= (-10/3.6):
                # C0
                if distance_to_front > des_distance:
                    return AdaptationTechnique.NO_ADAPTATION
                # C1
                elif (des_distance + 2) > distance_to_front > (des_distance - 2):
                    return AdaptationTechnique.STRUCTURAL
                # C2
                elif des_distance > distance_to_front > (des_distance/2):
                    return AdaptationTechnique.STRUCTURAL
                # C3
                else:
                    return AdaptationTechnique.CONTEXT
            # S3
            elif speed_dif < -10:
                # C0, C1
                if distance_to_front >= (des_distance - 5):
                    return AdaptationTechnique.STRUCTURAL
                # C2, C3
                else:
                    return AdaptationTechnique.CONTEXT

            # all data available, but in ACC mode
            elif front_vehicle.speed_tuple[1] == FailureType.no_failure and \
                    front_vehicle.acceleration_tuple[1] == FailureType.no_failure and \
                    self.knowledge.cont_max_acc == 2:
                return AdaptationTechnique.PARAMETER
                # missing parameter
            elif front_vehicle.speed_tuple[1] != FailureType.no_failure or \
                    front_vehicle.acceleration_tuple[1] != FailureType.no_failure and \
                    self.knowledge.cont_max_acc == 2.75:
                return AdaptationTechnique.PARAMETER

        elif current_controller == ControllerType.BRAKE:
            if distance_to_front == -1:
                return AdaptationTechnique.STRUCTURAL
            if distance_to_front > des_distance:
                return AdaptationTechnique.STRUCTURAL
        else:
            return AdaptationTechnique.NO_ADAPTATION

    def terminate(self):
        self.data_file.close()
        print("Closed data file")
        #self.plot_distance()

    def plot_distance(self):
        fig1, ax = pyp.subplots(2)
        ax[0].plot(self.timestamp, self.desired_distance, label="Desired distance")
        ax[0].plot(self.timestamp, self.distance, label="Distance")
        ax[0].set_xlabel("Time (s)")
        ax[0].set_ylabel("Distance [m]")
        ax[0].set_title(f"Distance comparison {self.ego_vehicle.role_name}")
        ax[0].legend()

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
