# BiG-MRTA
Multi-Robot Task Allocation in Disaster Response: Addressing Dynamic Tasks with Deadlines and Robots with Range and Payload Constraints

This repository contains the implementation of three multi-robot task allocation algorithms, namely BiG-MRTA, Feas-RND, and EDF. It also contains the flood simulator to generate the data for test scenarios and the disaster response simulator to run each algorithm on a pre-computed flood scenario. For further information on this, please refer to our paper: [Ghassemi, P., and Chowdhury, S., Multi-Robot Task Allocation in Disaster Response: Addressing Dynamic Tasks with Deadlines and Robots with Range and Payload Constraints, Robotics and Autonomous Systems, 2021, 103905, https://doi.org/10.1016/j.robot.2021.103905.](https://www.sciencedirect.com/science/article/abs/pii/S0921889021001901)

# How to Use the Code 
This section provides further information on the usage of this code:
- bigmrta.py: This contains the main algorithm that each robot calls to select a task based on BiG-MRTA. To see how use this method, you can check two following example files that they run the BiG-MRTA method for huge static and dynamic case over multiple swarm size (scalability analysis). You can modify Line 24 in this code to change the swarm size. 
Example files: main_bigmrta_StaticHugeProblem_nRobot.py and main_bigmrta_DynamicHugeProblem_SA_nRobot.py

- bigmrta_wLatency.py: This contains the main algorithm that each robot calls to select a task based on BiG-MRTA by simulating latency in receiving information. To see how use this method, you can check the following example file. You can modify Line 24 in these codes to change the swarm size. Example file: main_bigmrta_StaticHugeProblem_nRobot_Latency.py

- bigmrta_taptc.py: This contains the main algorithm that each robot calls to select a task based on BiG-MRTA for the TAPTC problem. To see how use this method, you can check this example file: main_bigmrta_TAPTC.py

- edf_taptc.py: This contains the Earliest Deadline First (EDF) algorithm that each robot calls to select a task for the TAPTC problem. To see how use this method, you can check this example file: main_edf_TAPTC.py

- feas_rnd.py: This contains the feasibility-preserving random-walk (Feas-RND) baseline algorithm that each robot calls to select a task. To see how use this method, you can check this example file: main_feas_rnd_StaticHugeProblem_nRobot.py. You can modify Line 24 in these codes to change the swarm size.

# Dependencies
numpy, matplotlib, scipy, networkx
