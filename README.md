# Master_Thesis_Wittemaier
Building an adaptive System which uses Machine Learning

Installation:
1. Install all requirements specified in the requirements.txt:

pip3 install -r requirements.txt
sudo apt-get install graphviz

2. Make sure you have an installation of Carla (version >=0.9.9)
3. Make sure your PYTHONPATH variable includes the carla PythonAPI
4. add the folder /DAM/dam/implementation to your PYTHONPATH
	export PYTHONPATH=$PYTHONPATH:~/DAM/dam

Execution:
1. Launch the Carla Server
2. Launch DAM by executing the "carla_control_client.py", located in /DAM/dam/implementation/carla_client

python3 carla_control_client.py

Note:
In case that the carla server takes too long to load the new town, a timeout error will come. Just wait until the server has load the town, and then restart the carla client

Training the Tree:

1. Collect data:
2. To collect data, you have activate the "Record Data" Checkbox in the User Control Window. Then, the simulation will store labeled data for each simulation step inside the "decision_tree_data_set_Follower_x.csv" file located in the carla_client folder
3. After stopping the execution, you have to select the desired data samples and copy them into the "Combined.csv" file located in the "DecisionTree" folder.
4. To train the DT, execute the "TreeTrainer.py" also located inside the "DecisionTree" folder:

python3 DecisionTree.py

5. This will then load the Combinded.csv file, reclassify and filter the data and store them in the "cleaned_file.csv", which is then used by the Tree Trainer to train the Decision Tree.
6. Changing the training parameters is done inside "TreeTrainer.py"
7. After training the DT, TreeTrainer stores the trained tree inside the "Analyzer" folder as "Communication_Failuretree.joblib". When rerunning the simulation, the new DT will be used during the execution.
