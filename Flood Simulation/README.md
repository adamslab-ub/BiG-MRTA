# Flood Simulation
A simple flood simulation is considered here. An aggressive rate of floodwater rise is considered here, which is comparable to extreme flash flooding scenarios, and allows posing challenging dynamic task scenarios where 100% completion rate may not necessarily be achievable. The following set of assumptions is used in creating the flood simulation: i) there are two floodwater levels; ii) the first level is a horizontal plane, which starts at the elevation of the ocean water and rises uniformly at a user-defined rate (4 meters per hour in our case studies); iii) the second level is an inclined plane underneath the river drainage of Hilo ($0\leq x \leq 10$ and $14\leq y \leq 20$), where the water level rises at a higher averaged rate of 6 meters per hour; and iv) floodwater does not recede during the mission.


## Tasks -- Flood Victims
Tasks are defined by their location and time deadline. In this environment, the task locations are specified to be initially above the water level, and at least 1 km away from each other. It is assumed that a different process, e.g., a team of scouting UAVs identifies the task locations, and passes on this information to the multi-UAV response team without any delay or loss of information (i.e., immediate and complete observation of the dynamic task space is assumed). For our case studies, a distribution based on the population density of each region and the [FEMA flood zones](http://gis.hawaiinfip.org/fhat/) is used by a random generator to create the representative task scenarios. After generating the location of tasks, the flood simulation is executed, and the time deadline of each task is defined in a deterministic manner to be the time when the water level reaches 0.5 meters above ground level at the task location. The tasks with missed deadlines are not allowed to be selected and they are thus removed from the set of available tasks. Tasks, once generated, can take three different statuses: active, completed, and missed (i.e., deadline is passed).

For further information on this, please refer to our paper: [Ghassemi, P., and Chowdhury, S., Multi-Robot Task Allocation in Disaster Response: Addressing Dynamic Tasks with Deadlines and Robots with Range and Payload Constraints, Robotics and Autonomous Systems, 2021, 103905, https://doi.org/10.1016/j.robot.2021.103905.](https://www.sciencedirect.com/science/article/abs/pii/S0921889021001901)

# How to Use the Code 
To have a computationally efficient flood simulator and data samples, we separated the whole simulation in two main stages:
1. Generating data points; 2. Adding task information based on scenario settings (dynamic/static task scenario, tight/wide deadlines, etc.).

- To generate data points based on fool rate and the area, you can use this file: mainFloodSimulation_DOE.m. If you want to have a flood simulation for a different region (any place on the world that you have its elevation data) or different flood rate, you can use this file to generate the data points.

- To add task information that can be used by the BiG-MRTA, and Feas-RND algorithms, you need to run one of these files: mainGenerateDataset.m, mainGenerateDataset_Difficult.m, and mainGenerateDataset_Dynamic.m.

Note: You need to download bil file to execute it. Due to large size, the file has not been added to this Git repository.
