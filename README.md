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

3. Compile all the packages, in the source folder (*if there are more folders besides src delete them*) and setup the environment:

        ```$ compile```
   
5. Run all codes at once:

    ```$ ros2 launch start start.launch.py```

6. Run the codes separately:

    - Control: 
    
          ```$ control```

    - Decision: 
    
          ```$ decision```

    - GC: 
    
          ```$ gamecontroler```

    - IMU: 
    
          ```$ imu```

    - Motors: 

           ```$ motors```
      
    - Vision: 
    
          ```$ vision```

    - Neck:
          ```$ neck```
   

