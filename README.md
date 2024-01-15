# ros-simple-husky-mover
Specify destination for husky to move to. It should then go there.

Tested with ROS noetic (ROS1).

## Requirements
`pip3 install docopt`

## Preliminaries
In separate command line windows:
```
roscore                                   # Requires ROS1 (tested with noetic).
roslaunch husky_gazebo empty_world.launch # Requires husky gazebo demo.
```

## Run husky_mover.py
Run as follows:
```
# ./husky_mover.py -- <target_x> <target_y>
./husky_mover.py -- 10 1

# Invocations are repeatable.
./husky_mover.py -- 3 3
./husky_mover.py -- -5 0
./husky_mover.py -- 0 0
./husky_mover.py -- 6 7
./husky_mover.py -- -1 -1
# ...

# Alternatively:
while :; do
 x=$(($RANDOM % 20 - 10))
 y=$(($RANDOM % 20 - 10))
 cmd="./husky_mover.py -- ${x} ${y}"
 ${cmd}
done
```

## Strategy
1. Where am I?
2. Where is the target relative to me?
3. What's the distance to the target?
4. Am I within the specified vicinity?
   -> Yes: Shutdown. I'm done.
5. Calculate angle from current location to target in radians using arctan().
6. Transform orientation data to get current husky orientation angle in radians.
7. Current orientation in acceptable margin when compared to target angle?
   -> Yes: Just move forward.
   -> No: Turn (direction depends on relative angle).
8. Back to 1.

