# Lab 2: Automatic Emergency Braking

## YouTube video link
https://youtu.be/GvD-XuN7gVY (to see the logs, you may change to HD)

Since I notice that the performance is highly related on the speed of the car, I think one single fixed threshold might be not enough, so I used an adaptive ittc threshold as you can see in the code. Hence, my test is split into three scenarios: low speed, medium speed and high speed. And the test vedeo shows that under these three senarios, the car can normally behave even if it is going some curve trajectories or be quite near to the wall on the lateral side, and can stop as expected when there are some obstacles in the front.

- 0'00"~0'08": starting
- 0'08"~0'58": travelling straight in the hallway without false positives (low speed case)
- 1'00"~1'25": going towards the wall and test the node (low speed case)
- 1'26"~1'40": changing speed
- 1'40"~2'08": travelling straight in the hallway without false positives (medium speed case)
- 2'09"~2'39": going towards the wall and test the node (medium speed case)
- 2'39"~2'49": change speed
- 2'49"~3'02": travelling straight in the hallway without false positives (high speed case)
- 3'03"~3'28": going towards the wall and test the node (high speed case)

## Test Info
Since I implement this node by python but by compiled by ament\_camke\_python, to test the code, after adding executable permission, build and installation, run:
```
ros2 run safety_node safety_node.py
```
