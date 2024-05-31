# Tether
A ROS2 package enabling bi-directional communication across networks for remote robotics operations. It abstracts the network protocol through dynamically loaded plugins, providing out-of-the-box support for UDP and TCP interfaces. Ideal for robust, real-time data exchange between a robot and a base station over the internet.

## Minimal Example
### Robot
```
/udp_sender:
  ros__parameters:
    UdpInterface:
      local_address: "127.0.0.1"
      receive_port: 5001
      remote_address: "127.0.0.1"
      send_port: 5000
    
    topics:
      - "/gps/fix"
```
### Base Station
```
/udp_receiver:
  ros__parameters:
    UdpInterface:
      local_address: "127.0.0.1"
      receive_port: 5000
      remote_address: "127.0.0.1"
      send_port: 5001
```

In the above example, the `udp_receiver` node will publish `/gps/fix` messages that are received by `/udp_sender`.  This works with any message type automatically, as long as the message is sourced on both ends prior to launching the nodes.

