# control-library-for-embedded-systems

CONTROL LIBRARY FOR EMMBEDDED SYSTEMS


 Solver design especially in the computer simulation is an important matter to emulate any physical dynamical system. In order to understand the underlying mechanism of this thing, there should be a model needs to be developed in terms of differential equation and matrices equation.
 
 In this study, control library with dynamic solver design is developed in order to simulate any kinds of systems with respect to the given time parameters and system models.
 
 Additionally, the most useful technique to control physical system is included to this repository. 
 
 There are seven main usage of this library:
 
 - simulation of the modeled system
 - obtaining time response characteristics of the system
 - creating solver design to be implemented on embedded systems
 - desiging PID controllers
 - dynamic simulation result
 - creating nonlinear parameters on system dynamic
  
Which kinds of projects you can utilize the basis of this codes?

 - Engine design (physic motors)
 - VR/AR application
 - Embedded software simulation
 - Estimation and Control of Physical Systems
 - Real time engine application

How can you use the library?

 - Suppose that you have a system like
  xdot = A*x + B*u
   with the parameter of A matrice (state matrice) and B matrice (input matrice). x is a vector representing states and u is an input vector.
 - You can set A and B matrices as nonlinear terms with the selection of state parameters which are dependent on time or state.
 - You can set the whole parameters related to discrete sampling period and you wil get time response of system in both console and text file.
 - You can use phiControllib.h to create discrete PID controller with different sampling period.
 - You can use root of system or specific PID coefficient in order to accomplish reference tracking controller.
 - The whole codes can easily be implemented on ARDUNIO or STM32 processors!

For more detailed information about us,

  http://www.phinitelab.com/
  https://www.udemy.com/user/phinite-academy/
  
  
  
  
