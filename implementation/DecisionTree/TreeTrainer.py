import pandas
from sklearn import tree
import  pydotplus
from sklearn.tree import DecisionTreeClassifier
import matplotlib.pyplot as plt
import matplotlib.image as pltimg
import csv

class TreeTrainer(object):

    def __init__(self):

        self.data_file = pandas.read_csv("decision_tree_data_set.csv")
        self.decision_tree = DecisionTreeClassifier()
        self.map_data()
        self.features = None
        x, y = self.prepare_data()

        self.train_tree(x, y)

        data = tree.export_graphviz(self.decision_tree, out_file=None, feature_names=self.features)
        graph = pydotplus.graph_from_dot_data(data)
        graph.write_png("mytree.png")

        img = pltimg.imread("mytree.png")
        imgplot = plt.imshow(img)
        plt.show()

    def train_tree(self, x, y):

        self.decision_tree = self.decision_tree.fit(x, y)
    def prepare_data(self):
        self.features = ["Timestamp", "Current Controller", "Contr. max acc", "Contr. max dec",
                    "Ego speed", "Ego acc", "Ego distance to front",
                    "Front speed", "Front acc", "Front throttle", "Front brake",
                    "Leader speed", "Leader acc", "Leader throttle", "Leader brake",
                    "Max acc", "max dec", "max throttle", "max brake",
                    "Speed limit"]
        x = self.data_file[self.features]
        y = self.data_file["Technique"]

        return x, y

    def map_data(self):

        technique = {"NA": 0, "PA": 1, "SA": 2, "CA": 3}
        self.data_file["Technique"] = self.data_file["Technique"].map(technique)
        controller = {"Distance": 0, "Speed": 1, "Brake": 2}
        self.data_file["Current Controller"] = self.data_file["Current Controller"].map(controller)