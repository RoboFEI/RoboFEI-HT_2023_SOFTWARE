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
red='\e[0;31m'
NC='\e[0m' #No Color

folder_path=$(find ~/ -name "RoboFEI-HT_2023_SOFTWARE")

echo -e "${blue} Remove Sudo password${NC}"
    username=$(whoami)
    if ! sudo grep -q "^$username ALL=(ALL) NOPASSWD:ALL" /etc/sudoers; then
        echo "$username ALL=(ALL) NOPASSWD:ALL" | sudo EDITOR='tee -a' visudo
    fi

# =================================================== ROS2 Install ===================================================
echo -e "${blue} Instalação do ROS2...${NC}"
    # if ros is installed just continue without try to install again
    if [ -z "$(printenv ROS_DISTRO)" ]; then
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
        source /opt/ros/humble/setup.bash
        sudo apt install python3-colcon-common-extensions -y

        # Setup colcon and colcon_cd
        echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
        echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
        echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
    else 
        source /opt/ros/humble/setup.bash
    fi

    sudo apt update -y

    # Extra Packages
    sudo apt-get install ros-humble-vision-msgs -y
    sudo apt install ros-humble-usb-cam -y

    #Install rqt and plugins 
    sudo apt install ros-humble-rqt* -y

    #install ros2 dynamixel
    sudo apt-get install ros-humble-dynamixel-sdk -y

    # If already have the file, delete for a new version
    if [ -n "$(ls -a ~/ | grep .ros2_robot_plugin)" ]; then
        rm -f ~/.ros2_robot_plugin
    fi

    # Copy the ros2 plugins file for home location as hidden file
    cp "$folder_path"/robot_plugins/ros2_robot_plugin ~/.ros2_robot_plugin

    # If doesn't have the command in bashrc write in then
    if ! grep -q "source ~/.ros2_robot_plugin" ~/.bashrc; then
        echo "source ~/.ros2_robot_plugin" >> ~/.bashrc
    fi


echo -e "${blue} Instalação das bibliotecas necessárias...${NC}"
    sudo apt install python3-pip -y
    sudo pip3 install opencv-python-headless
    pip install construct

    #instalação da yolov8
    pip install ultralytics

    #Instalação da biblioteca para Json
    sudo apt install nlohmann-json3-dev -y

    echo -e "${blue} Robot Number: ${NC}"
    read rob_num
    echo "robot_number=$rob_num\nprint(robot_number)" | sudo tee /home/robot_num.py

echo -e "${blue} setup new rules for usb names${NC}"
    sudo rm /etc/udev/rules.d/robot-usb-ports.rules
    sudo cp "$folder_path"/robot_plugins/robot-usb-ports.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    # udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep ATTRS{serial}

#Bind Comand
# If the terminal is zsh you need to run this command after "rehash" 
echo -e "${blue} commands setup${NC}"
    if [ ! -d "$folder_path"/robot_plugins/robot_commands ]; then
        echo "${red}Robot commands folder not founded!${NC}"
    else 
        echo "${blue}Folder founded!${NC}"
        for filename in "$folder_path"/robot_plugins/robot_commands/*; do
            filename=$(basename $filename)
            sudo rm /usr/local/bin/"$filename"
            sudo cp "$folder_path"/robot_plugins/robot_commands/"$filename" /usr/local/bin
            sudo chmod 777 /usr/local/bin/"$filename"
        done
        hash -r
    fi

echo -e "${blue} Instaling Softwares${NC}"
    sudo apt update && sudo apt upgrade && sudo apt install snapd -y

    #VS Code
    sudo snap install --classic code

    #Dynamixel Wizard
    if [ -z "$(find ~/ -name DynamixelWizard2)" ]; then
        wget -P ~/Downloads https://www.dropbox.com/s/dl/csawv9qzl8m8e0d/DynamixelWizard2Setup-x86_64
        sudo chmod 755 ~/Downloads/DynamixelWizard2Setup-x86_64
        ~/Downloads/DynamixelWizard2Setup-x86_64
    fi

    #VLC
    sudo apt install vlc -y

    #Remmina
    sudo apt install remmina remmina-plugin-rdp remmina-plugin-secret remmina-plugin-spice -y

    #Terminator
    sudo apt install terminator -y

    #Filezila
    sudo apt install filezilla -y

    #Dependence for GameController
    sudo apt  install ant -y
    sudo apt install openjdk-11-jdk -y

    #install vim
    sudo apt install vim -y

    #install byobu
    sudo apt install byobu -y

    #install ssh-server
    sudo apt-get update -y
    sudo apt-get install openssh-server -y
    sudo ufw enable
    sudo ufw allow 22
    sudo ufw allow 5050
    sudo systemctl enable ufw