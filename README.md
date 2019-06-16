# Extended Kalman Filter Project - Readme
I created this repo as part 5 of the Udacity Self-Driving Car Engineer Nanodegree Program


In this project I utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

---

![alt text](https://github.com/FinnHendricks/UdacitySDC_Project5_Extended_Kalman_Filter/blob/master/data/RMSE_Visualization.JPG)


---




### If you want to run the kalman filter, you need several dependencies:

#### 1) Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).
#### 2) [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake .. && make
4. ./ExtendedKF
5. Start the Term 2 Simulator term2_sim.exe

---

## Other Important Dependencies that need to be installed before starting the build process

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


