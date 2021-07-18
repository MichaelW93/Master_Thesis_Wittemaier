import pandas
from sklearn import tree, metrics
import  pydotplus
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import matplotlib.image as pltimg
import csv
from joblib import dump, load
from implementation.data_classes import AdaptationTechnique, FailureType
from implementation.vehicle.controller import ControllerType

class TreeTrainer(object):

    def __init__(self):

        self.data_file = pandas.read_csv("Combined.csv")
        self.pa: float = 0
        self.no: float = 0
        self.sa: float = 0
        self.ca: float = 0

        self.classify_data()

        file = pandas.read_csv("cleaned_file.csv")
        #self.test_file = pandas.read_csv("Connection_Failure_Data_Set.csv")
        weights = self.balance_classes()
        self.decision_tree = DecisionTreeClassifier(min_samples_leaf=25, class_weight=weights, min_samples_split=25, min_weight_fraction_leaf=0.001,
                                                    min_impurity_decrease=0.001, criterion="entropy")
        #self.decision_tree = DecisionTreeClassifier()
        file = self.map_data(file)
        #self.test_file = self.map_data(self.test_file)
        self.features = None
        x, y = self.prepare_data(file)
        x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.3)

        self.train_tree(x_train, y_train)

        y_pred = self.decision_tree.predict(x_test)

        #x_test_2, y_test_2 = self.prepare_data(self.test_file)
        # y_pred_2 = self.decision_tree.predict(x_test_2)
        print("Accuracy:", metrics.accuracy_score(y_test, y_pred))
        #print("Accuracy:", metrics.accuracy_score(y_test_2, y_pred_2))
        data = tree.export_graphviz(self.decision_tree, filled=True, out_file=None, feature_names=self.features, class_names=["NO", "PA", "SA", "CA"])
        graph = pydotplus.graph_from_dot_data(data)
        graph.write_png("Connection_Failure_Tree.png")

        img = pltimg.imread("Connection_Failure_Tree.png")
        imgplot = plt.imshow(img)
        plt.show()

    def train_tree(self, x, y):

        self.decision_tree = self.decision_tree.fit(x, y)

        dump(self.decision_tree, "../platoon_controller/analyzer/Communication_Failuretree.joblib")
    def prepare_data(self, file):
        self.features = ["Controller", "CMACC", "CMDEC",
                         "Ego acc", "SpeedDifF", "SpeedDifL",
                         "OverLimit",
                         "DistError", "EDF",
                         "FSF", "Front acc", "FAF", "Front throttle", "Front brake", "FrOvLi",
                         "LSF", "Leader acc", "LAF", "Leader throttle", "Leader brake",
                         "Max acc", "max dec", "max throttle", "max brake"]
        x = file[self.features]
        y = file["Technique"]

        return x, y

    def map_data(self, file):

        technique = {"NO": 0, "PA": 1, "SA": 2, "CA": 3}
        file["Technique"] = file["Technique"].map(technique)
        controller = {"Distance": 0, "Speed": 1, "Brake": 2}
        file["Controller"] = file["Controller"].map(controller)
        failure = {"FailureType.no_failure": 0, "FailureType.omission": 1, "FailureType.faulty_value": 2,
                   "FailureType.delay": 3, "FailureType.faulty_delayed": 4, "FailureType.no_front_vehicle": 5,
                   "FailureType.wrong_front_vehicle": 6}
        file["FSF"] = file["FSF"].map(failure)
        file["FAF"] = file["FAF"].map(failure)
        file["LAF"] = file["LAF"].map(failure)
        file["LSF"] = file["LSF"].map(failure)
        file["EDF"] = file["EDF"].map(failure)
        return file

    def balance_classes(self):

        sum_ = self.no + self.pa + self.sa + self.ca

        max_ = max(self.no, self.ca, self.sa, self.pa)

        weights = {0:(max_/self.no), 1:(max_/self.pa), 2:(max_/self.sa), 3:(max_/self.ca)}
        print(weights)
        return weights

    def classify_data(self):

        counter = 0

        with open("Combined.csv") as csv_file:

            new_file = open("cleaned_file.csv", "w", encoding="UTF8")
            writer = csv.writer(new_file)
            header = ["Controller", "CMACC", "CMDEC",
                         "Ego acc", "SpeedDifF", "SpeedDifL",
                         "OverLimit",
                         "DistError", "EDF",
                         "FSF", "Front acc", "FAF", "Front throttle", "Front brake", "FrOvLi",
                         "LSF", "Leader acc", "LAF", "Leader throttle", "Leader brake",
                         "Max acc", "max dec", "max throttle", "max brake", "Technique"]

            writer.writerow(header)

            csv_reader = csv.reader(csv_file, delimiter=",")
            line_count = 0

            for row in csv_reader:
                if line_count == 0:
                    line_count += 1
                else:
                    if not row[0]:
                        continue
                    current_controller: "ControllerType" = self.__create_controller(row[0])
                    controller_max_acc = float(row[1])
                    controller_max_dec = float(row[2])
                    if current_controller == ControllerType.SPEED:
                        controller_max_acc = 10
                        controller_max_dec = -3.5
                    elif current_controller == ControllerType.BRAKE:
                        controller_max_acc = 0
                        controller_max_dec = -13
                    speed_dif = float(row[4])
                    over_limit = float(row[6])
                    dist_error = float(row[7])
                    edf = self.__create_failure_type(row[8])
                    fsf = self.__create_failure_type(row[9])
                    faf = self.__create_failure_type(row[11])
                    lsf = self.__create_failure_type(row[15])
                    laf = self.__create_failure_type(row[17])
                    front_throttle = float(row[12])
                    front_brake = float(row[13])
                    old_technique = self.__create_adaptation_technique(row[24])

                    if old_technique == AdaptationTechnique.NO_ADAPTATION:
                        self.no += 1
                    elif old_technique == AdaptationTechnique.PARAMETER:
                        self.pa += 1
                    elif old_technique == AdaptationTechnique.STRUCTURAL:
                        self.sa += 1
                    elif old_technique == AdaptationTechnique.CONTEXT:
                        self.ca += 1

                    acc_front = float(row[10])
                    front_over_limit = float(row[14])

                    if FailureType.faulty_value in [edf, fsf, faf, lsf, laf]:
                        counter += 1
                        continue
                    elif front_over_limit > 15:
                        pass
                        #counter += 1
                        #continue

                    if edf == FailureType.no_front_vehicle:
                        dist_error = 0

                    if fsf == FailureType.omission or faf == FailureType.omission:
                        front_brake = -1
                        front_throttle = -1
                        fsf = FailureType.omission
                        faf = FailureType.omission
                        counter += 1
                    else:
                        front_throttle = float(row[12])

                    max_throttle = max(front_throttle, float(row[18]), 0)
                    max_brake = max(front_brake, float(row[19]), 0)
                    max_acc = max(acc_front, float(row[16]), 0)
                    max_dec = min(acc_front, float(row[16]), 0)

                    if speed_dif == 100 and edf == FailureType.no_failure:
                        counter += 1
                        continue

                    technique: AdaptationTechnique = None

                    if current_controller == ControllerType.SPEED:
                        # PS2
                        if edf == FailureType.no_front_vehicle:
                            technique = AdaptationTechnique.NO_ADAPTATION
                        elif dist_error < 1 and (front_over_limit < 3 and over_limit < 3):
                            technique = AdaptationTechnique.STRUCTURAL
                        else:
                            technique = AdaptationTechnique.NO_ADAPTATION

                    elif current_controller == ControllerType.DISTANCE:

                        # no front vehicle
                        if edf == FailureType.no_front_vehicle:
                            technique = AdaptationTechnique.STRUCTURAL

                            # all data available, but in ACC mode
                        if controller_max_acc == 2:
                            if fsf == FailureType.no_failure:
                                if faf == FailureType.no_failure:
                                    technique = AdaptationTechnique.PARAMETER
                            # missing parameter
                        elif controller_max_acc == 2.75:
                            if fsf == FailureType.omission:
                                technique = AdaptationTechnique.PARAMETER
                            elif faf == FailureType.omission:
                                technique = AdaptationTechnique.PARAMETER

                        if over_limit > 3 or front_over_limit > 3:
                            technique = AdaptationTechnique.STRUCTURAL

                        # S0
                        if speed_dif >= 0:
                            # C3
                            if dist_error < -0.75:
                                technique = AdaptationTechnique.STRUCTURAL
                            # C0 - C2
                            else:
                                # no adaptation
                                pass
                        # S1
                        elif 0 > speed_dif >= -(5 / 3.6):
                            # C0, C1
                            if dist_error >= -0.2:
                                if acc_front <= -2:
                                    technique = AdaptationTechnique.STRUCTURAL
                            # C2
                            elif -0.2 > dist_error >= -0.4:
                                technique = AdaptationTechnique.STRUCTURAL
                            # C3
                            else:
                                technique = AdaptationTechnique.STRUCTURAL
                        # S2
                        elif (-5 / 3.6) > speed_dif >= (-10 / 3.6):
                            # C0
                            if dist_error >= 0.1:
                                # no adaptation
                                pass
                            # C1
                            elif 0.1 > dist_error >= -0.2:
                                technique = AdaptationTechnique.STRUCTURAL
                            # C2
                            elif -0.2 > dist_error >= -0.4:
                                technique = AdaptationTechnique.STRUCTURAL
                            # C3
                            else:
                                technique = AdaptationTechnique.CONTEXT

                        # S3
                        elif speed_dif < -10 / 3.6:
                            # C0, C1
                            if dist_error > -0.2:
                                technique = AdaptationTechnique.STRUCTURAL
                            #elif dist_error > 0.2:
                            #   technique = AdaptationTechnique.NO_ADAPTATION
                            # C2, C3
                            else:
                                technique = AdaptationTechnique.CONTEXT

                        if dist_error < 0 and acc_front < -10.5 or front_brake > 0.9:
                            technique = AdaptationTechnique.CONTEXT

                        if dist_error > 1:
                            technique = AdaptationTechnique.STRUCTURAL

                    elif current_controller == ControllerType.BRAKE:
                        if edf == FailureType.no_front_vehicle:
                            technique = AdaptationTechnique.STRUCTURAL
                        elif dist_error > 0.1 and speed_dif > 1 and front_brake <= 0:
                            technique = AdaptationTechnique.STRUCTURAL
                        elif dist_error < 0 and (front_brake > 0.9 or acc_front < -10.5 or speed_dif < -15/3.6):
                            technique = AdaptationTechnique.PARAMETER

                    if technique is None:
                        technique = AdaptationTechnique.NO_ADAPTATION

                    if technique != old_technique:
                        counter += 1

                    current_controller: str = self.__convert_controller_to_string(current_controller)
                    technique: str = self.__convert_technique_to_string(technique)

                    data = [current_controller, controller_max_acc, controller_max_dec,
                            row[3], speed_dif, row[5], over_limit,
                            dist_error, edf,
                            fsf, row[10], faf, front_throttle, front_brake, front_over_limit,
                            lsf, row[16], laf, row[18], row[19],
                            max_acc, max_dec, max_throttle, max_brake,
                            technique]

                    writer.writerow(data)
            print("Errors in classification: ", counter)
            return new_file

    def __convert_controller_to_string(self, controller: ControllerType) -> str:
        if controller == ControllerType.DISTANCE:
            return "Distance"
        elif controller == ControllerType.SPEED:
            return "Speed"
        elif controller == ControllerType.BRAKE:
            return "Brake"

    def __convert_technique_to_string(self, technique: AdaptationTechnique) -> str:
        if technique == AdaptationTechnique.NO_ADAPTATION:
            return "NO"
        elif technique == AdaptationTechnique.PARAMETER:
            return "PA"
        elif technique == AdaptationTechnique.STRUCTURAL:
            return "SA"
        elif technique == AdaptationTechnique.CONTEXT:
            return "CA"

    def __create_controller(self, controller_string) -> ControllerType:

        if controller_string == "Distance":
            return ControllerType.DISTANCE
        elif controller_string == "Speed":
            return  ControllerType.SPEED
        elif controller_string == "Brake":
            return ControllerType.BRAKE

    def __create_failure_type(self, failure_string) -> FailureType:
        if failure_string == "FailureType.no_failure":
            return FailureType.no_failure
        elif failure_string == "FailureType.omission":
            return FailureType.omission
        elif failure_string == "FailureType.faulty_value":
            return FailureType.faulty_value
        elif failure_string == "FailureType.no_front_vehicle":
            return FailureType.no_front_vehicle

    def __create_adaptation_technique(self, technique_string: str) -> AdaptationTechnique:
        if technique_string == "NO":
            return AdaptationTechnique.NO_ADAPTATION
        elif technique_string == "PA":
            return AdaptationTechnique.PARAMETER
        elif technique_string == "SA":
            return AdaptationTechnique.STRUCTURAL
        elif technique_string == "CA":
            return AdaptationTechnique.CONTEXT

if __name__ == "__main__":

    tree_trainer = TreeTrainer()

"""
                        
                        """