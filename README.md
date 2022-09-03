# Matavecontrol V9.0
Matavecontrol is a basic toolbox for control engineering. The toolbox can be used for both GNU Octave and MATLAB®. Easy to use and easy to install. The main focus on matavecontrol is to offer a control toolbox which can be used in both GNU Octave and MATLAB®. Matavecontrol has the same function names as MATLAB®'s Control System ToolBox, but the time discrete functions are included in the time continuous functions. Also the library is a very basic library so other developers can fast dive into the code.

# Caution

Installing GNU Octave's Control-Toolbox or MATLAB's Control-Toolbox/System Identification Toolbox WILL cause problems with MataveID & MataveControl because they are using the same function names.

# Typical use

To use Matavecontrol, you should allways start with to create a transfer function or a state space model. Then you can use that mathematical model in almost all the function of Matavecontrol. 

Here is some examples when I use Matavecontrol. MATLAB pictures are from Umeå University.

Creating a transfer function in MATLAB®

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-00-55.png)

Creating a transfer function in GNU Octave

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-01-11.png)

Create a bode diagram plot in MATLAB®

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-04-59.png)

Create a bode diagram plot in GNU Octave

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-04-32.png)

Create a state space model in MATLAB®

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-06-29.png)

Create a state space model in GNU Octave 

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-06-41.png)

Do a step simulation in MATLAB®

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-08-23.png)

Do a step simulation in GNU Octave

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-07-36.png)

Convert a time continuous transfer function to a discrete transfer function in MATLAB®

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-09-19.png)

Convert a time continuous transfer function to a discrete transfer function in GNU Octave

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-10-03.png)

Do a nyquist diagram plot in MATLAB®

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-11-30.png)

Do a nyquist diagram plot in GNU Octave

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Sk%C3%A4rmbild%20fr%C3%A5n%202017-11-09%2000-12-02.png)

# Model Predictive Control - Linear programming

Here I use MPC with linear programming. I used to use MPC with a quadratic programming, but unfortunately quadprog is only available for MATLAB and you need to have a license for that. So I wrote my own linear programming MPC with regularization for smoother outputs and inputs. The linear programming algorithm is available in C code as well in my other projects for embedded MPC systems, and that's the main reason why I selected a simple optimization solver instead of a large and difficult one.

![alt text](https://github.com/DanielMartensson/matavecontrol/blob/master/examples/Markering_024.png)

# Install
To install Matavecontrol, download the folder "sourcecode" and place it where you want it. Then the following code need to be written in the terminal of your MATLAB® or GNU Octave program.

```matlab
path('path/to/the/sourcecode/folder/where/all/matave/files/are/matavecontrol', path)
savepath
```

Example:
```matlab
path('/home/hp/Dokument/Reglerteknik/matavecontrol', path)
savepath
```

Important! All the .m files need to be inside the folder matavecontrol if you want the update function to work.

# Update
Write this inside the terminal. Then Matavecontrol is going to download new .m files to matavecontrol from GitHub

```matlab
updatematavecontrol
```


