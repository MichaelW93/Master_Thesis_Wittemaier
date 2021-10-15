# Master_Thesis_Wittemaier
Building an adaptive System which uses Machine Learning

Installation:
1. Install all requirements specified in the requirements.txt:

pip3 install -r requirements.txt

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
