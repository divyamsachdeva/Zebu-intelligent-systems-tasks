# Zebu-intelligent-systems-tasks

You can download this git repository by running the following command in the command line:

git clone https://github.com/divyamsachdeva/Zebu-intelligent-systems-tasks.git

The pre-requisites required to run this directory are as follows:
1.Downloading the ardupilot directory

We have to clone this to the home directory.

git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot

git checkout Copter-3.6

git submodule update --init –recursive


I have installed the dependencies using the commands below:



sudo apt install python-matplotlib python-serial python-wxgtk3.0 python- wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect


sudo pip install future pymavlink MAVProxy


2.Downloading Gazebo

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable
`lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update

sudo apt install gazebo9 libgazebo9-dev

check the installation by launching Gazebo.

gazebo --verbose

3.Downloading the ardupilot plugin for Gazebo
I have used khancyr plugin available in the Github.

git clone https://github.com/khancyr/ardupilot_gazebo

cd ardupilot_gazebo

mkdir build

cd build

cmake ..

make -j4

sudo make install

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

Setting the models

echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >>
~/.bashrc

. ~/.bashrc







**Steps to run the files**

**Task 1**

After these are installed, we can go from the home folder to the “Scripts” on the command line using the command

cd Scripts

Now this terminal is ready to play the python files prepared. In another terminal we go the ardupilot directory using the command on command line:

cd ardupilot

And then start the MAVlink server using the command:

sim_vehicle.py -v ArduCopter -f gazebo-iris –console

Once the MAVlink server is setup, it will show “link 1 down” but after we run the following command in the third terminal, it will be connected to the server and gazebo will be launched with it:

gazebo –verbose /ardupilot_gazebo/worlds/iris_arducopter_runway.world

It will launch the runway world but we change it any world even an empty one.

Then in the first terminal, where scripts folder is open, we run the command:

python task01.py

And in gazebo we can see the copter switching to guided mode and then arming and then taking off and staying in air for 20 seconds and  then goes to the given waypoint and then lands at the waypoint only.



The same can be seen in the video using the given link: https://youtu.be/8_4ojpvLeKY

**Task 2**

After the simulation is set up and gazebo is up and running. We run the python script named "task02.py" by using the following command:

python task02.py

This command should be run after going into the scripts folder, using

cd Zebu/scripts


After the python script is executed, the drone moves in a spiral pattern. Continuously changing its, x and y and z, according to the below given formula:

X = sin(6*t)
Y = cos(6*t)
Z = t

Here t varies from 0.5 to 6*pi, changing with the interval of 0.3

This gives us an amazing spiral pattern, which can be seen in the following video link: https://youtu.be/EOUy9AWeCmQ

In this video, we can see that as x and y changes, z also increases with it and after it reaches integer value of 2*∏ which is 6.


**Task 3**

After task 2 is completed, in this task we will first change the mode of copter to "GUIDED", arm it and then take it off to a certain altitude and then, move it to another coordinate. As it reaches there, we can detect the ArUco marker and land over there after detection. The detection is done through the laptop's camera but not the drone's camera. To run the task 3 file in the scripts folder, just type this:

python task3.py

And this will let you execute task 3.




THANK YOU!!
