# smartrail_hostctrl
A ROS package for the Smart Rail project.

## Installation
These nodes may be installed by cloning the repo into the package source directory of a catkin workspace, 
typically the `src` directory immediately under `$ROS_WORKSPACE`.

## SystemD Installation
The odroid user is included in the smartrail.service systemd configuration file
located in the scripts directory.  
`cp scripts/smartrail.service /etc/systemd/system/`
With this service file installed, the odroid user will execute `scripts/smartrail_service.sh`
on launch of the systemd service.

## Dependencies
