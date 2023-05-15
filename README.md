# Matavecontrol V11.0
Matavecontrol is a basic toolbox for control engineering. The toolbox can be used for both GNU Octave and MATLAB®. Easy to use and easy to install. The main focus on matavecontrol is to offer a control toolbox which can be used in both GNU Octave and MATLAB®. Matavecontrol has the same function names as MATLAB®'s Control System Toolbox, but the time discrete functions are included in the time continuous functions. Also the library is a very basic library so other developers can fast dive into the code.


# TODO

- `reg.m` This need to be fixed so it is the same as MATLAB
- `lqgreg.m` I don't know if `Matavecontrol` should keep it. It do the same as `reg.m`.
- More work on `gensig.m`
- Make `linprog.m` faster by using vectorization and not C-code style

# Typical use

To use Matavecontrol, you should allways start with to create a transfer function or a state space model. Then you can use that mathematical model in almost all the function of Matavecontrol. 

Here is some examples when I use Matavecontrol. MATLAB pictures are from Umeå University.

Creating a transfer function in MATLAB®

![Transfer Function Example - MATLAB](examples/transferFunctionExmapleMATLAB.png)

Creating a transfer function in GNU Octave

![Transfer Function Example - Octave](examples/transferFunctionExmapleOctave.png)

Create a bode diagram plot in MATLAB®

![Bode Diagram Example - MATLAB](examples/bodeExampleMATLAB.png)

Create a bode diagram plot in GNU Octave

![Bode Diagram Example - Octave](examples/bodeExampleOctave.png)

Create a state space model in MATLAB®

![State Space Example - MATLAB](examples/stateSpaceExampleMATLAB.png)

Create a state space model in GNU Octave 

![State Space Example - Octave](examples/stateSpaceExampleOctave.png)

Do a step simulation in MATLAB®

![Step Simulation - MATLAB](examples/stepExampleMATLAB.png)

Do a step simulation in GNU Octave

![Step Simulation - Octave](examples/stepExampleOctave.png)

Convert a time continuous transfer function to a discrete transfer function in MATLAB®

![c2d Example - MATLAB](examples/c2dExampleMATLAB.png)

Convert a time continuous transfer function to a discrete transfer function in GNU Octave

![c2d Example - Octave](examples/c2dExampleOctave.png)

Do a nyquist diagram plot in MATLAB®

![Nyquist Example - MATLAB](examples/nyquistExampleMATLAB.png)

Do a nyquist diagram plot in GNU Octave

![Nyquist Example - Octave](examples/nyquistExampleOctave.png)

# Model Predictive Control - Linear programming & Quadratic programming with integral action

Matavecontrol have both linear programming MPC and quadratic programming MPC. There is equivalent C code at [CControl](https://github.com/DanielMartensson/CControl) if you want to apply them to embedded hardware. Select the programming method, quadratic or linear, that works with your situation. Both works fine, but quadratic programming (Hildreth's Method) is faster than linear programming (Simplex Method) in MATLAB. In C-code, it depends on your C-compiler.

![MPC Example - MATLAB](examples/mpcExampleMATLAB.png)

# Install
To install Matavecontrol, download the `sourcecode` folder and rename the folder `sourcecode`it to a name of free choice.

```matlab
path('path/to/the/folder_of_sourcecode/+mc', path)
savepath
```

Example of a typical `path`. Notice that `+mc` must be there.

```matlab
path('C:\Users\dmn\Documents\Octave\libraries\+mc', path)
savepath
```

# Update
Write this inside the terminal. Then Matavecontrol is going to download new .m files to matavecontrol from GitHub

```matlab
updatematavecontrol
```

