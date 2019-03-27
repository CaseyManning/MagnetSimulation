# Physics Research Magnet Simulation

A simulation of a system of spherical magnets


### Setup and Workflow

- Make sure you have Blender 2.8 installed (2.79 may work, but may cause issues)

- Open blender. When running a python program inside blender, you will probably see an error reading "Python Script Failed, look in the console for now". You may wonder "but where is the console", and the answer is: it does't exist! In order to see error messages for your python script, navigate to `/Applications/Blender/blender.app/Contents/MacOS` (or wherever your blender is installed to, and run ./blender in your terminal. Then, the error messages will appear in that terminal window when you run a python script.

- You will probably want to be in the "Scripting" tab (on the top bar) for using blender


### Useful Keyboard Shortcuts
Space: Start simulation

Alt + P: Run python script

⌘  + Shift + S: Save python script

right click on the beginning of the timeline (may not be visible in scripting tab, go to layout) to reset the simulation


### State of the Simulation

Currently, as can be seen when you run the simulation, all the magnets are attracte to each other, and converge in the center. This is accomplished through the use of a force field object which is attached to each magnet, a simple sphere. In the future, this will happen programatically through the python API. We will have to find out whether or not it is feasable to leverage the force field objects through the python API to generate custom forces and rotations. If not, we may need to update positions manually through python. 

I will work on creating a system to use the python API to apply forces to the magnets during the simulation, so I will update the readme with information on how to apply force / rotation to magnets at that point.


### TODO

- create framework for implementing forces and rotation through python API (Casey will hopefully work on this next week)

- Create a good system to easily turn on and off gravity, and automatically place a certain number of magnets in the simulation

- create framework for representing magnets as a class in python
