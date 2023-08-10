# LoRa Glora (Super TaiRa) Communication

## Hardware Preparations


You should have at least two sets of the XBee modules as followings:

For LoRa:
| Parts       | Model                    | Where to Purchase  |
|-------------|--------------------------|--------------------|
| LoRa Module | DRAGINO LoRa Shield v1.4 | [DRAGINO website](https://www.dragino.com/products/lora/item/102-lora-shield.html) |
| Arduino     | Arduino UNO              | [Arduino website](https://www.arduino.cc/)                                         |
| Others      |                          |                    |
|             |                          |                    |

For GloRa (Super TaiRa)
| Parts        | Model                    | Where to Purchase  |
|--------------|--------------------------|--------------------|
| GloRa Module | XXX                      |                    |
| Arduino      | XXX                      |                    |
| Others       |                          |                    |
|              |                          |                    |

We also assume you have
* A: A laptop or workstation as base station, running Ubuntu system with Docker installed.
* B: A Respberry Pi or other embedded boards with Docker installed, or another computing unit like item A.
* C: One or more like item A or B for mesh.

## Installations

TODO: We should use the same Docker for XBee

We provide Docker images for
* GPU/CPU laptop or workstation 
```
$ docker pull XXX/YYY:tag
```

* RPi 3B: 

```
$ docker pull XXX/YYY:tag
```
TODO: should we add the following ROS packages to docker build?
* apriltags_ros
* apriltags_ros_test

## Data Packet Design

### General 

the only one or the last one data packet

| Header | type | bytes | bytes | bytes | bytes | realdata | ...... | realdata | checksum |
|--------|------|-------|-------|-------|-------|----------|--------|----------|----------|

else (not the last one)

| Header | type | bytes | bytes | bytes | bytes | realdata | ...... | realdata |
|--------|------|-------|-------|-------|-------|----------|--------|----------|

### Packet Size Considerations

## How to run

TODO: add scenario

###
to give glora a static usb port /dev/usb_glora
``` 
$ source set_usb_port.sh
``` 

#### For Host 
``` 
$ roslaunch lora_communication glora_host.launch port:=[SERIAL_PORT]
```
#### For Client
``` 
$ roslaunch lora_communication glora_client.launch port:=[SERIAL_PORT] veh:=[husky | duckiefloat1~5]
```

## Topic info
#### Host

Publish:
```
# Robot pose - topic: PoseStamped
/husky/pose_new
/duckiefloat1/pose_new
/duckiefloat2/pose_new
/duckiefloat3/pose_new
/duckiefloat4/pose_new

# Artifact pose - topic: PoseArray
/extinguisher/pose_list
/drill/pose_list
/survivor/pose_list
/backpack/pose_list
/smartphone/pose_list

# Anchorball info - topic: AnchorBallDetection
/anchorball/info_new
```

#### Client

Subscribe:
```
# subt_info topic: subt_info
```
<br /><br /><br /><br />

# lora_communication
ros package of LoRa communication for SubT

set the usb port of arduino

```
source set_arduino_port.sh
```

move the arduino-LoRa folder to your arduino libraries


run sender on husky
```
rosrun lora_communication sender_decoder.py
```

run receiver on laptop
```
rosrun lora_communication receiver_decoder.py
```
