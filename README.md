# Introduction
This ROS package implements a state dependent LQR controller for high accuracy trajectory tracking.

# Future Plans
Working on developing a learning algorithm for inflight thrust mapping. Strain gauges are attached to a quadrotor to collect thrust measurements in flight and how the mapping of normalized motor input [0,1] to thrust changes as a result of voltage decrement, ground effects, and propeller downwash. 

## Videos:

<a href="https://www.youtube.com/watch?v=phKAAQSkkjc" target="_blank"><img src="https://img.youtube.com/vi/phKAAQSkkjc/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

<a href="https://www.youtube.com/watch?v=Arlwatxqgfo" target="_blank"><img src="https://img.youtube.com/vi/Arlwatxqgfo/hqdefault.jpg" 
alt="VINS" width="320" height="240" border="10" /></a>

## Acknowledgements:
The euler-based state dependent LQR controller is an extension of [1] for euler angles instead of quaternions.

## References:
[1] Foehn, Philipp & Scaramuzza, Davide. (2018). Onboard State Dependent LQR for Agile Quadrotors. 10.1109/ICRA.2018.8460885. 
