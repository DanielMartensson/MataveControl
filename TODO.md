Here is a list of the following points that need to be fixed: 

* Fix better help comments at the top of all function

* Create a tutorial how to use the commands

* Rebuld all state feedback reg.m, mpcreg.m and lqgreg.m so they don't create feed forward compensator Kr
Also regmpc.m should only compute Kr and the control law. Not build the controller.
