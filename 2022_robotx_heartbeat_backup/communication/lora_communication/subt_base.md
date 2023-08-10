----------------------------------------------------
# base station
----------------------------------------------------

## terminal group 1

### roscore
```
$ source docker.sh
# source environment.sh
# roscore
```

### launch glora
```
$ docker exec -it subt16 bash
# source environment.sh
# roslaunch lora_communication glora_host.launch n:=2 (for 2 vehicle)
```

### launch joy stick for emergency stop
start button for husky        e_stop 
Y     button for duckiefloat  e_stop
```
$ docker exec -it subt16 bash
# source environment.sh
# rosrun joy joy_node
```

### launch artiface client
modify server IP address to DARPA provided ip address in 
```
SubT/catkin_ws/src/perception/arti_client/src/arti_client.py
```
```
$ docker exec -it subt16 bash
# source environment.sh
# roslaunch arti_client arti_client.launch
```
----------------------------------------------------
## terminal group 2

### launch mapping base
```
$ source docker.sh
# source environment.sh
# roscd lora_communication
# source set_second_master.sh
# roslaunch pcl_perception base.launch
```

### launch mapping relay
modify server IP address to DARPA provided ip address in 
```
sub_server/mapping_server/src/mapping_relay/launch/mapping_relay.launch
```
```
$ cd ~/SubT/catkin_ws/src/lora_communication
$ source set_second_master.sh
$ cd ~/subt_server
$ source catkin_ws/devel/setup.bash
$ roslaunch mapping_relay mapping_relay.launch
```


