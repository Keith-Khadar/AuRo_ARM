# Change to the directory where this script is located
cd "$(dirname "$0")" || { echo "Failed to change directory. Exiting."; exit 1; }

# Install ROS JAZZY
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install ros-dev-tools -y

sudo apt update

sudo apt upgrade

sudo apt install ros-jazzy-desktop -y

# Install the USB camera driver
sudo apt install ros-jazzy-usb-cam -y
pip install pydantic==1.10.9 --break

# Add setup.bash to the bashrc
BASH_RC_LINES=". $(dirname $(realpath $0))/setup.bash"
echo "$BASH_RC_LINES" >> ~/.bashrc
WORKSPACE_DIR="$(cd $(dirname ${BASH_SOURCE[0]})/.. && pwd)"
echo "WORKSPACE_DIR=$WORKSPACE_DIR" >> ~/.bashrc

# Copies bashrc to interactive login shells (like tmux)
echo 'if [ -n "$BASH_VERSION" ] && [ -n "$PS1" ]; then
	# include .bashrc if it exists
	if [ -f "$HOME/.bashrc" ]; then
		. "$HOME/.bashrc"
	fi
fi' >>~/.profile


# Install python packages
pip install -r ./requirements.txt --break

# Install tmux
sudo apt install tmux -y

# Clear the terminal and source the bashrc
clear 
exec bash
