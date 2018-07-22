# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

In this project a PID controller is used to control a car in a simulation environment. The purpose of this project is to understand the PID controller and its application in self-driving car. Consequently, a such controller will be built in C++ to control a simulation car driving around the lake race track.

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

```
P = -Kp * CTE
```

### D-Component

The D component considers the current rate of error change, trying to bring this rate to zero. It aims at flattening the error trajectory into a horizontal line, damping the force applied, and so reduces overshoot. In this project the D component serves to bring
the car to center smoothly without overshoot.

```
diff_cte = cte - prev_cte
D = - Kd * diff_cte
```

### I-Component

The I component integrates the error overtime, trying to remove residual error by adding a control effect due to the historic cumulative value of the error. In this project the I component helps to reduce the CTE around curves.

```
int_cte += cte
I = - Ki * int_cte
```

### Implementation of PID controller

By adding the three components we have the control value (steering value)

```
steer_value = -Kp * CTE - Kd * diff_cte - Ki * int_cte
```

### Hyperparameter tuning

Paramter tuning can be carried out with different methods like Ziegler-Nichols, Tyreus Luyben, Cohencoon, etc. In the lesson a twiddle algorithm for tuning is introduced. Since no system model of the simulation is provided, I decided to not use any tuning algorithm but manual tuning. First, I tuned the P value to keep the car driving on the track, then I tuned the D value to reduce the oscillation. After the car driving smoothly I tuned the I value to keep the car not leaving the trajectory, while it is driving around curves. My final hyperparameters after tuning:

```
Kp = 0.1
Ki = 0.0022
Kd = 2.4
```

### PID velocity controller

After making the car driving smoothly through the track with constant velocity, I continued to implement the second PID controller to control the velocity of the car. The velocity can only be manipulated indirectly over the throttle value. By increasing or decreasing the throttle value we can change the car velocity. The idea is the car should accelerate when it is driving on a straight line and the  CTE is small. When the car is driving around curves, the CTE increases, so it should slow down to avoid oscillation. The hyperparameters of the velocity controller are chosen as following:

```
Kp = 0.1
Ki = 0.0006
Kp = 7.0
```

Update of throttle value:

```
throttle_value = MAX_THROTLE_VALUE - pid_t.TotalError();
```

With this implemenation the car can reach over 60 mph on straight line and can still drive around 40 mph around curves.














	  



