# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

In this project a PID controller is used to control a car in a simulation environment. The purpose of this project is to understand the PID controller
and its application in self-driving car. Consequently, a such controller will be build in C++ to control a simulation car driving around the lake race
track.

With help of the simulator, a cross track error is calculated and given to the controller as input parameter for update of steering angle and the velocity.
Vehicle velocity is controlled indirectly over throttle value.

## PID controller

PID stands for proportional-integral-derivative, is a control loop feedback mechanism, what is depicted in the following image
![Screenshot](images/PID.png)
where r(t) is desired process value
      e(t) is error value
	  u(t) is control variable
	  y(t) is measured process value
	  
In Udacity PID project the desired and measured process values are monitored by the simulator. During the simulation the simulator measures the distance from car position to the center of
the lane, which is called cross track error (CTE). The CTE represents the component e(t) of a PID controller.

### P-Component

The P component is proportional to the current error value, in other words the control output is proportional to the error from the previous step. In this project the steering angle 
is set proportionally to the CTE, that means if the car is far from the trajectory, the steering angle needs to be big enough to get the car faster to its trajectory. But a big
value of the proportional component also makes the system oscillating or unstable.

P = -Kp * CTE

### 



	  



