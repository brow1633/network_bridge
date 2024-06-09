# Network Bridge
[![CI](https://github.com/brow1633/network_bridge/actions/workflows/CI.yml/badge.svg)](https://github.com/brow1633/network_bridge/actions/workflows/CI.yml)

**Network Bridge** is a lightweight ROS2 node designed for robust communication between robotic systems over arbitrary network protocols. Supporting UDP and TCP protocols out of the box, this packages seamlessly bridges ROS2 topics across networks, facilitating effective remote communications between a base station and robotic systems, or between multiple robotic systems.

## Installation
Simply clone the repository into your ROS2 workspace and build with `colcon build`.

## Usage

### Demo
#### TCP
```
ros2 launch network_bridge tcp.launch.py

ros2 topic pub /tcp1/MyDefaultTopic std_msgs/msg/String "data: 'Hello World'"

ros2 topic echo /tcp2/MyDefaultTopic
```

#### UDP
```
ros2 launch network_bridge udp.launch.py

ros2 topic pub /udp1/MyDefaultTopic std_msgs/msg/String "data: 'Hello World'"

ros2 topic echo /udp2/MyDefaultTopic
```
### Configuration
Simply setup the network interface parameters and list your desired topics to get started.  If you are using UDP over cellular data, it is recommended to setup a VPN to facilitate connection.  Also, please note that **no encryption** occurs within this package.  Currently, if you would like encryption, you must use a VPN.

See `config/Udp1.yaml` for a description of all parameters, as well as the TCP example configuration files.
#### Minimal Example
The following configuration examples demonstrate a robot sending a message on `/gps/fix` over UDP to a basestation that will then re-publish the message.  This works seamlessly on all message types, so long as they are built and sourced on both ends of the transmission.
#### Robot
```
/udp_sender:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.2"
      receive_port: 5001
      remote_address: "192.168.1.3"
      send_port: 5000
    
    topics:
      - "/gps/fix"
```
#### Base Station
```
/udp_receiver:
  ros__parameters:
    UdpInterface:
      local_address: "192.168.1.3"
      receive_port: 5000
      remote_address: "192.168.1.2"
      send_port: 5001
```

### Choice of protocol
- **UDP**: Use UDP for low-latency, high-throughput communications, where occasional data loss is tolerable. Ideal for real-time telemetry data like sensor streams.
- **TCP**: Opt for TCP when data integrity and reliability are critical. This ensures that control commands and state transitions are reliably delivered, though with potentially higher latency.

Network protocols are implemented as pluginlib plugins, allowing the creation of arbitrary interfaces using the abstract class `include/network_interfaces/network_interface_base.hpp`.  Any interface that can send and receive bytes could theoretically be implemented, including protocols that go beyond point-to-point communication, such as ZMQ.  Please consider opening a pull request if you implement a new network interface.

### Tuning
This node can be launched with logger level DEBUG, which provides useful information for tuning the compression, rate and stale message parameters.  For each message that is sent, the receiving side will output the number of bytes received, the decompressed size in bytes and the transmission delay.

## Acknowledgements
This package was developed for use in the Indy Autonomous Challenge by the Purdue AI Racing team.  Inspiration was taken from mqtt_client (https://github.com/ika-rwth-aachen/mqtt_client/).
