# Robotics-Navigation
Mobile robots are being used in different environments to reduce human intervention. Our project aims at developing one such mobile robot which can be deployed in hospital premises. The purpose of the
robot is to provide delivery services like medicines, food, etc. to patients. The robot is capable of navigation with use of path planning algorithm. Robot path planning is about finding a collision free motion from one
position to another.
In this project, the motion planning of mobile robot is based on Wavefront algorithm. The proposed approach has been tested through computer simulation, with a set of environmental parameters
with different levels of complexity depending on the density of the obstacles. The encoder motor wheels are used for motion which give information about the distance travelled. Ultrasonic sensors sense the
obstacles so that robot can perform required path change. The sensors have their own sensitivity which gives rise to errors. Further, the mechanical structure of the robot base adds on to the errors. In order to
minimize these errors, Kalman filter implementation is being done. Robot is controlled by an ARM
Cortex-A7 (Raspberry pi 2 board) processor running embedded Linux.
