<img src="https://raw.github.com/NIFTi-Fraunhofer/nifti_uav/master/doc/UAV.png" alt="NIFTi-UAV" width="55%"/>

* Authors: Marius Beul, Josef Boerding, Erik Zimmermann
* Licence: BSD

1. Introduction/Overview
------------------------

The NIFTi UAV is an eight motor Microcopter carrying a 2D laser scanner, two cameras, an ultrasonic-, and a GPS-sensor. It is remotely controlled via a 2,4GHz system.  
The data of the sensors can be processed on board or be transferred via WiFi over 2,4GHz or 5GHz to a remote access point.  
For the NIFTi project the copter is used to transmit overview images of a disaster scenario to support the information and decision management of a rescue team. 

This picture shows an overview of the internal modules and communication channels:  
<img src="https://raw.github.com/NIFTi-Fraunhofer/nifti_uav/master/doc/UAV_modules_and_connections.jpg" alt="NIFTi UAV modules and connections" width="90%"/>


2. Installation
---------------

This software is based on [ROS Fuerte](http://wiki.ros.org/fuerte).
You need also the following packages:
* [git] (http://git-scm.com)


2.1 Sources
-----------

You can get the sources by cloning the repository above:

*git clone https://github.com/NIFTi-Fraunhofer/nifti_uav*

The package *nifti_uav* and its dependencies is build with cmake/rosmake. You can find a tutorial for using cmake [here](http://www.youtube.com/watch?v=CLvZTyji_Uw).

To build the *nifti_uav* sources, make sure you have the *nifti_uav* package directory and its dependencies set correctly in your ROS_PACKAGE_PATH environment variable.  
Then type into a console:

*rosmake mosquito mosquito_launchers*  


3. Usage
--------

To launch the ROS nodes on the UAV you have to run a roscore on the UAV and then just type:  
*roslaunch mosquito_launchers UAV.launch*

Further documentation of how to handle the UAV you can find [here](https://github.com/NIFTi-Fraunhofer/nifti_uav/blob/master/doc/Flying%20the%20NIFTi-Mikrokopter.docx).


4. Report a bug
---------------
Please use the [issue tracker](https://github.com/NIFTi-Fraunhofer/nifti_uav/issues) of github to report a bug.


