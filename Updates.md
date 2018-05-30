# Updates

* Version 1.0 (2017-11-08)
 The realease of Matavecontrol
 
* Version 1.5 (2017-11-18)
Removed the solver fsolve and replaced it with ode45 because fsolve is a part of MATLAB® Optimization Toolbox, but freeware under GNU Octave. The new ode45 is going to solve the Algebraic Riccati Equations.

* Version 1.6 (2017-12-13)
Fixed dbdrop.m so it takes G(0) as the beginning static gain.

* Version 2.0 (2018-01-18
All bode/nyquist/sigma diagrams have default frequency interval.
Fixed color at dbdrop.m
Fixed bug at pid.m so now you can choose only static gain transfer functions
initial.m can return states
lsim.m, impulse, ramp and step.m can return output

* Version 2.5 (2018-01-23)
Added nlsim.m - Nonlinear simulation with state staturation

* Version 2.9 (2018-02-15)
Added findmaxgain.m - To compute the maximum gain limit

* Version 3.0 (2018-02-18
Implemented updatematavecontrol.m so it can update the whole library by using the terminal

* Version 3.5 (2018-02-18)
Added pade approximation pade.m for time delays. Pade approximation is good enough when to construct controllers.

* Version 3.6 (2018-02-18)
Added an Otto Smith delay time compensator.

* Version 4.0 (2018-02-26)
Added Internal Model Control compensator imc.

* Version 5.0 (2018-03-31)
Added intss.m which is adding integration to a state space model.
Added a MPC - Model Predictive Controller - mpcreg.m. This controller is without constrains. But that will be included in the future.
Fixed feed forward factor for referece signal for better tracking - reg.m and lqgreg.m
Fixed lqi.m controller to get the integral control law. Also added precompensator factor for anti-winup.

* Version 6.0 (2018-05-30)
Removed intss.m.
Renamed nlsim.m to satlsim.m 
Removed mpcreg.m
Added lmpc.m


