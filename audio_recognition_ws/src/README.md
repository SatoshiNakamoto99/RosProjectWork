# Requirements:
Check the requirements.txt for the dependency.

# How to run:
1. First of all, navigate in the folder scripts under the package project_work and set:
   ON_CHATBOT=True and ON_PEPPER=True and verify that VIDEO_GHOST_FRAME=10 for the system execution. 
   After that execute the following command in the folder of the workspace.
2. ``` catkin build ```
3. ``` source devel/setup.bash ```
4. ``` roslaunch project_work all.launch ```

# How to run the unit Test
1. First of all, navigate in the folder scripts under the package project_work and set:
   ON_CHATBOT=False and ON_PEPPER=False and verify that VIDEO_GHOST_FRAME=5 for the test execution. 
   After that execute the following command in the folder of the workspace
2.  ``` catkin build ```
3. ``` source devel/setup.bash ```
4. ``` roslaunch project_work <name_of_test_launch>.launch ```

# How to run the system in simulation mode
You can execute all the system in a simulation version. 

1. Execution of the system on PC without Pepper and without Chatbot 
        Navigate in the folder scripts under the package project_work and set:
        ON_CHATBOT=False and ON_PEPPER=False
        After that execute the following command in the folder of the workspace
         ``` catkin build ```
         ``` source devel/setup.bash ```
         ``` roslaunch project_work project_simulation_no_chatbot.launch ```
2. Execution of the system on PC without Pepper and with Chatbot 
        Navigate in the folder scripts under the package project_work and set:
        ON_CHATBOT=True and ON_PEPPER=False
        After that execute the following command in the folder of the workspace
         ``` catkin build ```
         ``` source devel/setup.bash ```
         ``` roslaunch project_work project_exec_chatbot_simulation.launch ```
3. Execution of the system on  Pepper and without Chatbot 
        Navigate in the folder scripts under the package project_work and set:
        ON_CHATBOT=False and ON_PEPPER=True
        After that execute the following command in the folder of the workspace
         ``` catkin build ```
         ``` source devel/setup.bash ```
         ``` roslaunch project_work all_no_chatbot.launch ```


