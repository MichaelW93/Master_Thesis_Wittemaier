from implementation.platoon_controller.knowledge.knowledge import Knowledge
from implementation.data_classes import *
from implementation.vehicle.controller import ControllerType
from joblib import load
import os
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from sklearn import tree
    from sklearn.tree import DecisionTreeClassifier

class Analyzer(object):

    def __init__(self, knowledge: Knowledge):
        self.knowledge: Knowledge = knowledge

        self.switched_once = False

        print(os.getcwd())
        self.decision_tree: "DecisionTreeClassifier" = load("../platoon_controller/analyzer/Speeding_Leader_Tree.joblib")

    def run_step(self, analyzer_input: EnvironmentKnowledge) -> AdaptationTechnique:

        current_controller = self.knowledge.current_controller

        if current_controller == ControllerType.SPEED:
            if not self.switched_once:
                if (analyzer_input.ego_speed_tuple[0] * 3.6) > analyzer_input.speed_limit - 3:
                    self.switched_once = True
                    return AdaptationTechnique.STRUCTURAL
        else:
            return AdaptationTechnique.NO_ADAPTATION
        if self.knowledge.leader_id in analyzer_input.other_vehicles:
            if (analyzer_input.other_vehicles[self.knowledge.leader_id].brake > 0.1):
                if current_controller == ControllerType.BRAKE:
                    return AdaptationTechnique.NO_ADAPTATION
                else:
                    return AdaptationTechnique.STRUCTURAL
            elif current_controller == ControllerType.SPEED:
                if not self.switched_once:
                    if (analyzer_input.ego_speed_tuple[0] * 3.6) > analyzer_input.speed_limit - 3:
                        self.switched_once = True
                        return AdaptationTechnique.STRUCTURAL
                elif (analyzer_input.other_vehicles[self.knowledge.leader_id].brake > 0.15):
                    return AdaptationTechnique.STRUCTURAL
            else:
                if current_controller == ControllerType.BRAKE:
                    return AdaptationTechnique.STRUCTURAL
                else:
                    return AdaptationTechnique.NO_ADAPTATION
        else:
            return AdaptationTechnique.STRUCTURAL

        if current_controller == ControllerType.DISTANCE:
            if (analyzer_input.ego_distance_tuple == -1):
                return AdaptationTechnique.STRUCTURAL
            elif (analyzer_input.other_vehicles[self.knowledge.leader_id].brake > 0.15):
                return AdaptationTechnique.STRUCTURAL
        else:
            return AdaptationTechnique.NO_ADAPTATION
            data = []

            if self.knowledge.front_vehicle_id in analyzer_input.other_vehicles:
                front_vehicle = analyzer_input.other_vehicles[self.knowledge.front_vehicle_id]
            else:
                front_vehicle = None
            if self.knowledge.leader_id in analyzer_input.other_vehicles:
                leader_vehicle = analyzer_input.other_vehicles[self.knowledge.leader_id]
            else:
                leader_vehicle = None

            current_controller = 0
            if self.knowledge.current_controller == ControllerType.DISTANCE:
                current_controller = 0
            elif self.knowledge.current_controller == ControllerType.SPEED:
                current_controller = 1
            else:
                current_controller = 2

            contr_max_acc = self.knowledge.cont_max_acc
            contr_max_dec = self.knowledge.cont_max_dec

            ego_speed = analyzer_input.ego_speed_tuple[0]
            ego_acc = analyzer_input.ego_acceleration_tuple[0]
            ego_distance = analyzer_input.ego_distance_tuple[0]
            speed_diff_to_front = analyzer_input.speed_diff_to_front
            speed_diff_to_leader = analyzer_input.speed_diff_to_leader
            speed_over_limit = analyzer_input.speed_over_limit
            des_dist = analyzer_input.desired_distance
            dist_error = analyzer_input.distance_error

            if front_vehicle is not None:
                front_speed = front_vehicle.speed_tuple[0]
                fvs_failure = self.__convert_failure_type(front_vehicle.speed_tuple[1])
                front_acc = front_vehicle.acceleration_tuple[0]
                fva_failure = self.__convert_failure_type(front_vehicle.acceleration_tuple[1])
                front_throttle = front_vehicle.throttle
                front_brake = front_vehicle.brake
            else:
                front_speed = -1
                fvs_failure = self.__convert_failure_type(FailureType.omission)
                front_acc = -1
                fva_failure = self.__convert_failure_type(FailureType.omission)
                front_throttle = -1
                front_brake = -1

            if leader_vehicle is not None:
                leader_speed = leader_vehicle.speed_tuple[0]
                lvs_failure = self.__convert_failure_type(leader_vehicle.speed_tuple[1])
                leader_acc = leader_vehicle.acceleration_tuple[0]
                lva_failure = self.__convert_failure_type(leader_vehicle.acceleration_tuple[1])
                leader_throttle = leader_vehicle.throttle
                leader_brake = leader_vehicle.brake
            else:
                leader_speed = -1
                lvs_failure = self.__convert_failure_type(FailureType.omission)
                leader_acc = -1
                lva_failure = self.__convert_failure_type(FailureType.omission)
                leader_throttle = -1
                leader_brake = -1

            max_acc = analyzer_input.max_acc
            max_dec = analyzer_input.max_dec
            max_throttle = analyzer_input.max_throttle
            max_brake = analyzer_input.max_brake

            speed_limit = analyzer_input.speed_limit

            data = [
                [current_controller, contr_max_acc, contr_max_dec,
                 ego_speed, ego_acc, ego_distance, speed_diff_to_front,
                 speed_diff_to_leader, speed_over_limit,
                 des_dist, dist_error,
                 front_speed, fvs_failure, front_acc, fva_failure, front_throttle, front_brake,
                 leader_speed, lvs_failure, leader_acc, lva_failure, leader_throttle, leader_brake,
                 max_acc, max_dec, max_throttle, max_brake,
                 speed_limit]
            ]
            print(data)
            technique = self.decision_tree.predict(data)
            print("Technique from tree: ", technique)
            return self.__convert_technique(technique)

    @staticmethod
    def __convert_failure_type(failure: FailureType) -> float:
        if failure == FailureType.no_failure:
            return 0
        elif failure == FailureType.omission:
            return 1
        elif failure == FailureType.faulty_value:
            return 2
        elif failure == FailureType.delay:
            return 3
        elif failure == FailureType.faulty_delayed:
            return 4
        elif failure == FailureType.no_front_vehicle:
            return 5
        elif failure == FailureType.wrong_front_vehicle:
            return 6

    @staticmethod
    def __convert_technique(technique: str) -> AdaptationTechnique:
        if technique[0] == 0:
            return AdaptationTechnique.NO_ADAPTATION
        elif technique[0] == 1:
            return  AdaptationTechnique.PARAMETER
        elif technique[0] == 2:
            return  AdaptationTechnique.STRUCTURAL
        elif technique[0] == 3:
            return AdaptationTechnique.CONTEXT
