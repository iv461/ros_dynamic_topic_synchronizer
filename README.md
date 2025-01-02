# fsd_topic_synchronizer

This is a Robot Operating System (ROS) package for synchronizing multiple topics by approximately matching the message timestamps, similarly to the [message_filters](wiki.ros.org/message_filters)-package.
Unlike the [message_filters](wiki.ros.org/message_filters) package however, the number of messages does not need to be set at compile-time, instead it can be done at runtime. 
This package thereore removes the limitation of having to know the number of topics at compile, instead only the message types have to be provided.

It also provides one additional feature that is very useful for implementing sensor fusion applications: A timeout. 

# Requirements

- ROS 1 Noetic, C++ 14

# Example

In [examples/], there is a simple talker-listener example that synchronizes three different topics. 
It demonstrates the approximate time-stamp matching as well as the time-out behavior. After a topic "times out" (for example due to sensor malfunction), we still receive the synchronized messages of the remaining sensors.

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

This code is licensed under Apache 2 license.




