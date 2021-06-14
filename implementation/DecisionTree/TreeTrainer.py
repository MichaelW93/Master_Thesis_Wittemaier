import pandas
from sklearn import tree, metrics
import  pydotplus
from sklearn.tree import DecisionTreeClassifier
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import matplotlib.image as pltimg
import csv
from joblib import dump, load

class TreeTrainer(object):

    def __init__(self):

        self.data_file = pandas.read_csv("Connection_Failure_Data_Set (Kopie).csv")
        self.test_file = pandas.read_csv("Connection_Failure_Data_Set.csv")
        self.decision_tree = DecisionTreeClassifier()
        self.data_file = self.map_data(self.data_file)
        self.test_file = self.map_data(self.test_file)
        self.features = None
        x, y = self.prepare_data(self.data_file)
        x_train, x_test, y_train, y_test = train_test_split(x, y, test_size=0.1, random_state=1)

        self.train_tree(x, y)

        y_pred = self.decision_tree.predict(x_test)

        x_test_2, y_test_2 = self.prepare_data(self.test_file)
        y_pred_2 = self.decision_tree.predict(x_test_2)
        print("Accuracy:", metrics.accuracy_score(y_test, y_pred))
        print("Accuracy:", metrics.accuracy_score(y_test_2, y_pred_2))
        data = tree.export_graphviz(self.decision_tree, filled=True, out_file=None, feature_names=self.features, class_names=["NO", "PA", "SA", "CA"])
        graph = pydotplus.graph_from_dot_data(data)
        graph.write_png("my_tree.png")

        img = pltimg.imread("my_tree.png")
        imgplot = plt.imshow(img)
        plt.show()

    def train_tree(self, x, y):

        self.decision_tree = self.decision_tree.fit(x, y)

        dump(self.decision_tree, "../platoon_controller/analyzer/Communication_Failuretree.joblib")
    def prepare_data(self, file):
        self.features = ["Controller", "CMACC", "CMDEC",
                         "Ego speed", "Ego acc", "Ego dist", "SpeedDifF", "SpeedDifL",
                         "OverLimit",
                         "DesDist", "DistError",
                         "Front speed", "FSF", "Front acc", "FAF", "Front throttle", "Front brake",
                         "Leader speed", "LSF", "Leader acc", "LAF", "Leader throttle", "Leader brake",
                         "Max acc", "max dec", "max throttle", "max brake",
                         "Speed limit"]
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
        print(file)
        return file

if __name__ == "__main__":

    tree_trainer = TreeTrainer()