#font colors:
#Black        0;30     Dark Gray     1;30
#Blue         0;34     Light Blue    1;34
#Green        0;32     Light Green   1;32
#Cyan         0;36     Light Cyan    1;36
#Red          0;31     Light Red     1;31
#Purple       0;35     Light Purple  1;35
#Brown/Orange 0;33     Yellow        1;33
#Light Gray   0;37     White         1;37

blue='\e[0;34m'
NC='\e[0m' #No Color

echo -e "${blue} Remove Sudo password${NC}"
    username=$(whoami)
    echo "$username ALL=(ALL) NOPASSWD:ALL" | sudo EDITOR='tee -a' visudo


#ROS2 Humble Installing
echo -e "${blue} Instalação do ROS2...${NC}"

    #Setup Locale
    sudo apt update && sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8 
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
    export LANG=en_US.UTF-8

    #Enable Ubuntu Universe Repository
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe 

    #Add ROS2 GPG key
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    #Install ROS2 Humble
    sudo apt update && sudo apt upgrade -y
    sudo apt install ros-humble-desktop -y
    sudo apt install ros-dev-tools -y
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    sudo apt install python3-colcon-common-extensions -y

    #Install rqt and plugins 
    sudo apt update
    sudo apt install ros-humble-rqt*

    #install ros2 dynamixel
    sudo apt-get install ros-humble-dynamixel-sdk

    # Setup colcon and colcon_cd
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
    echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

echo -e "${blue} Instalação das bibliotecas necessárias...${NC}"
    sudo apt install python3-pip -y
    sudo pip3 install opencv-python 
    pip install -r src/vision_yolov7/vision_yolov7/requirements.txt
    sudo pip install construct

    #instalação da yolov8
    pip install ultralytics

echo -e "${blue} setup new rules for usb names${NC}"
    sudo rm /etc/udev/rules.d/robot-usb-ports.rules
    sudo touch /etc/udev/rules.d/robot-usb-ports.rules && sudo chmod 777 /etc/udev/rules.d/robot-usb-ports.rules
    echo -e "KERNEL==\"ttyUSB*\", ATTRS{product}==\"USB-Serial Controller\", SYMLINK+=\"imu\"\n" >> /etc/udev/rules.d/robot-usb-ports.rules 
    echo -e "KERNEL==\"ttyUSB*\", ATTRS{serial}==\"A50285BI\", SYMLINK+=\"motors\"\n" >> /etc/udev/rules.d/robot-usb-ports.rules 
    echo -e "KERNEL==\"video*\", ATTRS{product}==\"HD Pro Webcam C920\",ATTR{index}==\"0\", SYMLINK+=\"camera\"" >> /etc/udev/rules.d/robot-usb-ports.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    # udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep ATTRS{serial}

#Bind Comand
echo -e "${blue} commands setup${NC}"
    #goalkeeper fast conection 
    sudo rm /usr/local/bin/gol
    sudo touch /usr/local/bin/gol && sudo chmod 777 /usr/local/bin/gol
    echo -e "${bashshel}\n\nssh -X robo@192.168.7.2" >> /usr/local/bin/gol 

    #Comand for compile 
    sudo rm /usr/local/bin/compile
    sudo touch /usr/local/bin/compile && sudo chmod 777 /usr/local/bin/compile
    echo -e "${bashshel} \n\ncd ~/RoboFEI-HT_2023_SOFTWARE \ncolcon build --symlink-install \nsource install/setup.bash" >> /usr/local/bin/compile 

    #Comand for run IMU
    sudo rm /usr/local/bin/imu
    sudo touch /usr/local/bin/imu && sudo chmod 777 /usr/local/bin/imu
    echo -e "${bashshel} \n\ncd ~/RoboFEI-HT_2023_SOFTWARE \nsource install/setup.bash \nros2 run um7 um7_node" >> /usr/local/bin/imu 

    #Comand for run control
    sudo rm /usr/local/bin/control
    sudo touch /usr/local/bin/control && sudo chmod 777 /usr/local/bin/control
    echo -e "${bashshel} \n\ncd ~/RoboFEI-HT_2023_SOFTWARE \nsource install/setup.bash \nros2 launch robot_bringup control_bringup.launch.py" >> /usr/local/bin/control

    #Comand for run vision
    sudo rm /usr/local/bin/vision
    sudo touch /usr/local/bin/vision && sudo chmod 777 /usr/local/bin/vision
    echo -e "${bashshel} \n\ncd ~/RoboFEI-HT_2023_SOFTWARE \nsource install/setup.bash \nros2 launch robot_bringup vision_bringup.launch.py" >> /usr/local/bin/vision

    #Comand for run decision
    sudo rm /usr/local/bin/decision
    sudo touch /usr/local/bin/decision && sudo chmod 777 /usr/local/bin/decision
    echo -e "${bashshel} \n\ncd ~/RoboFEI-HT_2023_SOFTWARE \nsource install/setup.bash \nros2 run decision_pkg decision_node" >> /usr/local/bin/decision

    #Comand for run Gamecontroller
    sudo rm /usr/local/bin/gamecontroller
    sudo touch /usr/local/bin/gamecontroller && sudo chmod 777 /usr/local/bin/gamecontroller
    echo -e "${bashshel} \n\ncd ~/RoboFEI-HT_2023_SOFTWARE \nsource install/setup.bash \nros2 run game_controller connect" >> /usr/local/bin/gamecontroller

    #Comand for run motors
    sudo rm /usr/local/bin/motors
    sudo touch /usr/local/bin/motors && sudo chmod 777 /usr/local/bin/motors
    echo -e "${bashshel} \n\ncd ~/RoboFEI-HT_2023_SOFTWARE \nsource install/setup.bash \nros2 run motors_pkg motors_communication" >> /usr/local/bin/motors

    sudo chown root: /usr/local/bin/compile /usr/local/bin/imu /usr/local/bin/control /usr/local/bin/vision /usr/local/bin/decision /usr/local/bin/gamecontroller /usr/local/bin/motors

echo -e "${blue} Instaling Softwares${NC}"
    sudo apt update && sudo apt upgrade && sudo apt install snapd -y

    #VS Code
    sudo snap install --classic code

    #Dynamixel Wizard
    wget -P ~/Downloads https://www.dropbox.com/s/dl/csawv9qzl8m8e0d/DynamixelWizard2Setup-x86_64
    sudo chmod 755 ~/Downloads/DynamixelWizard2Setup-x86_64
    ~/Downloads/DynamixelWizard2Setup-x86_64

    #VLC
    sudo apt install vlc -y

    #Remmina
    sudo apt install remmina remmina-plugin-rdp remmina-plugin-secret remmina-plugin-spice -y

    #Terminator
    sudo apt install terminator -y

    #Filezila
    sudo apt install filezilla -y

    #Fortclient
    wget -O ~/Downloads/forticlient.deb https://filestore.fortinet.com/forticlient/forticlient_vpn_7.0.7.0246_amd64.deb
    sudo dpkg -i ~/Downloads/forticlient.deb
    sudo apt install -f -y

    #Dependence for GameController
    sudo apt  install ant -y
    sudo apt install openjdk-19-jdk -y
