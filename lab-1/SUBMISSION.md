# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

First of all ```source``` means run all the commands inside a script.

The first command ```source /opt/ros/foxy/setup.bash``` is used to:
1. setup ros environment, meanly by setting up the environment variables which includes the path of the dependencies, executables and other source codes;
2. make sure your terminal has access to ros libiary and dependencies (different from local_setup, local_setup only set up for current ternimal);
3. make sure you can use all the ros2 command lines;

The second command ```source install/local_setup.bash``` is used to:
1. setup and activate the environment of the workspace, which will update the environment variables which leads to the packages, nodes and executables, etc.. which will make the pkg and node build in the ws avaiable through command line;
2. make sure all the dependencies are recogonized and accessed;
3. through "local_setup" not "set_up", since it only set the ws up for the current terminal, it is convenient to work with multiple workspaces in multiple terminals, which each has its own environment and configuration;

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

We use ```queue_size``` meanly because the speed of publisher publishing messages and subscriber receiving and dealing with the messages may not be the same;
For publisher, ```queue_size``` determines the size of outgoing message queue, once a message is pubulished, it is added to this queue, once the queue is full, the oldest message will be aborted;
For subscriber, ```queue_size``` determines the size of incoming message queue, once a message is arrives, it is added to this queue, once the queue is full, the oldest message will be aborted;

With a larger ```queue_size```, the possibility of message dropping is reduced since more messages are buffered, but may lead to memory issue if the queue is getting too large especially the message itself is large, and besides the latency may increase;
With a smaller ```queue_size```, the data which is actually handled by the subscriber is much newer, which is or important to a real-time system and the latency may be lower, but the data loss might be much more possible.

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

When calling ```ros2 launch``` in the directory where the launch file is: since the current file is included, the system actually run the launch file trough the current path, hence we do not to rebuild the package;

When calling ```ros2 launch``` when the launch file is installed with the package, we have to rebuild the package since when building the package, the launch file will be installed in the "share path" and launch from this path, once you changed something in the launch file, only after you rebuild the package, your change can be installed into the package in "share path".
