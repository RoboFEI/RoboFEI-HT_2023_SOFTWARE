# RoboFEI-HT_2023_SOFTWARE

This repository contains the code written using ROS2, by RoboFEI team, which is currently used in our physical robots. 

It is divided in 8 packages: 

* control: contains the code related to the robots motion and its parameters;
* custom_interfaces: contains all the custom interfaces used in the code;
* decision: contains the code responsible for the robots decision;
* GC: contains the code responsible for the robots communication with the game controller;
* localization_pkg: contains the code responsible for the robots localization;
* robotis_ws: contains the code responsible for the robots communication with its motors;
* start: contains the launch file to run all the nodes at once;
* um7: contains the code responsible for getting IMU measurements;
* vision_pkg: contains the code responsible for the robots vision.

## Installation:
1. First, download this repo from github:

    ```$ git clone https://github.com/RoboFEI/RoboFEI-HT_2023_SOFTWARE.git```

2. Then, install ROS2 Humble and all the libraries that are used in our code:

    ```$ ./comandos.sh```

3. Compile all the packages, in the source folder (*if there are more folders besides src delete them*):

   ```$ colcon build --symlink-install ```

4. Setup the environment:

   ```$ source install/setup.bash```
   
5. Run all codes at once:

    ```$ ros2 launch start start.launch.py```

6. Run the codes separately:

    - Control: 
    
      ```$ ros2 launch robot_bringup control_bringup.launch.py```

    - Decision: 
    
      ```$ ros2 run decision_pkg decision_node```

    - GC: 
    
      ```$ ros2 run game_controller connect```

    - IMU: 
    
      ```$ ros2 run um7 um7_node ```

    - Motors: 

      ```$ ros2 run motors_pkg motors_communication```
      
    - Vision: 
    
      ```$ ros2 launch robot_bringup vision_bringup.launch.py```


