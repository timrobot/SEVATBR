This is the Robot Class, where all of the hardware can finally communicate to the computer.

Features:
- serial communication
- inverse kinematics
- action states

Known Issues:
- serial sometimes takes awhile to connect, please look into
- takes a long time to connect to an arduino
- communication TO the robot class is very stringent and inflexible, please revise later if necessary
- communication FROM the robot class is virtually nonexistant, please think of a schema, suggestions:
  - json formatted string for fast conversion and easy access
  - perhaps a more easily structured piece of memory?
