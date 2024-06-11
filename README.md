# fsd_topic_synchronizer

This is a ROS package for a topic synchronizer to approximately synchronize multiple topics similar to the [message_filters](wiki.ros.org/message_filters)-package.
Unlike the [message_filters](wiki.ros.org/message_filters) package however 

# Requirements

- ROS 1 Noetic, C++ 14

# Example

In [examples/], there is a simple talker-listener example that synchronizes three different topics. 
It demonstrates the approximate time-stamp matching as well as the time-out behavior where after a topic "times out" (i.e. if a sensor dies), the node still receives the synchronized set of the remaining sensors.

After building the package, just start the listener by 

```sh
rosrun fsd_topic_synchronizer fsd_topic_synchronizer_example_listener
```

and then the talker: 

```sh
rosrun fsd_topic_synchronizer fsd_topic_synchronizer_example_talker
```

which should give the expected output: 

```sh
TODO
```

# Run tests 

To build and execute the unit-tests for the synchronization policy, just run: 

```sh
catkin test fsd_topic_synchronizer
```

# License 

Licensed under Apache 2




