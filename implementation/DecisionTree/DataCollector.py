import csv
from typing import TYPE_CHECKING,  Dict
from implementation.data_classes import AdaptationTechnique
from implementation.vehicle.controller import ControllerType
from matplotlib import pyplot as pyp
if TYPE_CHECKING:
    from implementation.data_classes import EnvironmentKnowledge, OtherVehicle
    from implementation.platoon_controller.knowledge.knowledge import Knowledge
    from implementation.vehicle.vehicles import ManagedVehicle

class DataCollector(object):

    def __init__(self, knowledge: "Knowledge", ego_vehicle):

        self.header = []
        self.ego_vehicle: "ManagedVehicle" = ego_vehicle

        self.knowledge: "Knowledge" = knowledge
        self.data_file = open(f"decision_tree_data_set_{self.ego_vehicle.role_name}.csv", "a", encoding="UTF8")
        self.writer = csv.writer(self.data_file)
        self.distance = []
        self.desired_distance = []
        self.timestamp = []

        header = ["Current Controller", "Contr. max acc", "Contr. max dec",
                  "Ego speed", "Ego acc", "Ego distance to front",
                  "Front speed", "Front acc", "Front throttle", "Front brake",
                  "Leader speed", "Leader acc", "Leader throttle", "Leader brake",
                  "Max acc", "max dec", "max throttle", "max brake",
                  "Speed limit", "Technique"]

    def run_step(self, data: "EnvironmentKnowledge"):

        technique_result = self.classify_data(data)

    def classify_data(self, data: "EnvironmentKnowledge"):

        timestamp = data.timestamp

        max_dec = self.__get_max_deceleration(data.other_vehicles)
        max_acc = self.__get_max_acceleration(data.other_vehicles)
        max_throttle = self.__get_max_throttle(data.other_vehicles)
        max_brake = self.__get_max_brake(data.other_vehicles)
        front_vehicle_acc = -1
        front_vehicle_speed = -1
        front_vehicle_throttle = -1
        front_vehicle_brake = -1

        if self.knowledge.front_vehicle_id != -1:
            front_vehicle_acc = data.other_vehicles[self.knowledge.front_vehicle_id].acceleration[0]
            front_vehicle_speed = data.other_vehicles[self.knowledge.front_vehicle_id].speed[0]
            front_vehicle_throttle = data.other_vehicles[self.knowledge.front_vehicle_id].throttle
            front_vehicle_brake = data.other_vehicles[self.knowledge.front_vehicle_id].brake

        leader_acc = data.other_vehicles[self.knowledge.leader_id].acceleration[0]
        leader_speed = data.other_vehicles[self.knowledge.leader_id].speed[0]
        leader_throttle = data.other_vehicles[self.knowledge.leader_id].throttle
        leader_brake =  data.other_vehicles[self.knowledge.leader_id].brake

        ego_speed =  data.ego_speed[0]
        ego_acc = data.ego_acceleration[0]

        speed_dif = front_vehicle_speed - ego_speed

        distance_to_front = data.ego_distance[0]
        current_controller: "ControllerType" = self.knowledge.current_controller
        speed_limit = data.speed_limit

        con_max_acc = self.knowledge.cont_max_acc
        con_max_dec = self.knowledge.cont_max_dec

        des_distance = self.knowledge.timegap * ego_speed

        self.distance.append(distance_to_front)
        self.desired_distance.append(des_distance)
        self.timestamp.append(timestamp.elapsed_seconds)

        technique = AdaptationTechnique.NO_ADAPTATION

        if current_controller == ControllerType.SPEED:
            # PS2
            if ego_speed < speed_limit:
                technique = AdaptationTechnique.NO_ADAPTATION
            elif distance_to_front > (des_distance * 2):
                technique = AdaptationTechnique.NO_ADAPTATION
            else:
                technique = AdaptationTechnique.STRUCTURAL

        elif current_controller == ControllerType.DISTANCE:

            if front_vehicle_speed is None or front_vehicle_acc is None \
                    or front_vehicle_throttle is None or front_vehicle_brake is None:
                technique = AdaptationTechnique.PARAMETER
            elif distance_to_front == -1:
                technique = AdaptationTechnique.STRUCTURAL
                # PD0 is already covered from S0, same as PS0
                # PD1
                if (2 * des_distance) > distance_to_front > des_distance:
                    # PS1
                    if ego_speed < front_vehicle_speed <= (speed_limit / 3.6):
                        technique = AdaptationTechnique.PARAMETER
                    # PS2
                    elif ego_speed < front_vehicle_speed > (speed_limit / 3.6):
                        technique = AdaptationTechnique.STRUCTURAL
                # PD2
                elif distance_to_front > 2 * des_distance:
                    # PS0
                    if (speed_limit / 3.6 * 0.98) < ego_speed < (speed_limit / 3.6 * 1.02):
                        technique = AdaptationTechnique.NO_ADAPTATION
                    elif ego_speed < (speed_limit * 0.95) and ego_speed <= front_vehicle_speed:
                        technique = AdaptationTechnique.STRUCTURAL

            elif speed_limit < (ego_speed * 3.6) <= speed_limit + 5:
                technique = AdaptationTechnique.PARAMETER
            elif (ego_speed * 3.6) > speed_limit + 5:
                technique = AdaptationTechnique.STRUCTURAL
            # S0
            elif speed_dif >= 0:
                # C3
                if distance_to_front < (des_distance / 2):
                    technique = AdaptationTechnique.STRUCTURAL
                # C0 - C2
                else:
                    technique = AdaptationTechnique.NO_ADAPTATION
            # S1
            elif 0 > speed_dif >= -(5/3.6):
                # C0, C1
                if distance_to_front >= (des_distance * 0.95):
                    technique = AdaptationTechnique.NO_ADAPTATION
                # C2
                elif des_distance > distance_to_front >= (des_distance / 2):
                    technique = AdaptationTechnique.PARAMETER
                # C3
                else:
                    technique = AdaptationTechnique.STRUCTURAL
            # S2
            elif (-5/3.6) > speed_dif >= (-10/3.6):
                # C0
                if distance_to_front > des_distance:
                    technique = AdaptationTechnique.NO_ADAPTATION
                # C1
                elif (des_distance * 1.03) > distance_to_front > (des_distance * 0.97):
                    technique = AdaptationTechnique.PARAMETER
                # C2
                elif des_distance > distance_to_front > (des_distance /2):
                    technique = AdaptationTechnique.STRUCTURAL
                # C3
                else:
                    technique = AdaptationTechnique.CONTEXT
            # S3
            else:
                # C0, C1
                if distance_to_front >= (des_distance * 0.97):
                    technique = AdaptationTechnique.STRUCTURAL
                # C2, C3
                else:
                    technique = AdaptationTechnique.CONTEXT

        elif current_controller == ControllerType.BRAKE:
            if distance_to_front == -1:
                technique = AdaptationTechnique.STRUCTURAL
            if distance_to_front > des_distance:
                technique = AdaptationTechnique.STRUCTURAL

        if technique == AdaptationTechnique.NO_ADAPTATION:
            technique = "NA"
        elif technique == AdaptationTechnique.PARAMETER:
            technique = "PA"
        elif technique == AdaptationTechnique.STRUCTURAL:
            technique = "SA"
        elif technique == AdaptationTechnique.CONTEXT:
            technique = "CA"

        if current_controller == ControllerType.SPEED:
            current_controller = "Speed"
        elif current_controller == ControllerType.BRAKE:
            current_controller = "Brake"
        elif current_controller == ControllerType.DISTANCE:
            current_controller = "Distance"

        write_data =  [current_controller, con_max_acc, con_max_dec,
                       ego_speed, ego_acc, distance_to_front,
                       front_vehicle_speed, front_vehicle_acc, front_vehicle_throttle, front_vehicle_brake,
                       leader_speed, leader_acc, leader_throttle, leader_brake,
                       max_acc, max_dec, max_throttle, max_brake,
                       speed_limit, technique]
        self.writer.writerow(write_data)
        print(technique)
        return technique

    def terminate(self):
        self.data_file.close()
        self.plot_distance()

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
            if vehicle_data.acceleration[0] is not None:
                if vehicle_data.acceleration[0] < max_dec:
                    max_dec = vehicle_data.acceleration[0]
            elif vehicle_data.measured_acceleration[0]:
                if vehicle_data.measured_acceleration[0] < max_dec:
                    max_dec = vehicle_data.measured_acceleration[0]
        return max_dec

    def __get_max_acceleration(self, other_vehicles: Dict[int, "OtherVehicle"]) -> float:

        max_acc = 0
        for vehicle_data in other_vehicles.values():
            if vehicle_data.acceleration[0] is not None:
                if vehicle_data.acceleration[0] > max_acc:
                    max_acc = vehicle_data.acceleration[0]
            elif vehicle_data.measured_acceleration[0]:
                if vehicle_data.measured_acceleration[0] > max_acc:
                    max_acc = vehicle_data.measured_acceleration[0]
        return max_acc
