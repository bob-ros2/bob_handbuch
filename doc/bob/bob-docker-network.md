# ROS Multi Docker Hosts Network

ROS Nodes need to be in the same subnet to find each other. This is a challange if you want to use Docker container shared between multiple Docker hosts and also want to be able to discover the own hosts in order to be able to access hardware like a GPU, audio or displays.

* This example setup creates a macvlan network of 3 Docker hosts which share all the same subnet with 128 IP addresses. 
* The other 128 addresses are owned by the router to be able to assign free addresses also to other devices.
* Each Docker container which use this network can discover each other.
* Each Docker container which use this network can also discover the own host.
* Each Docker host has its own address range
* With the help of Docker Compose, fixed IP addresses could also be assigned to specific container.

Be warned, this network cannot be called very secure because the doors are wide open, but that's not the point here.

## Contained components
- 1 Router with DHCP capabilities (e.g. a typical router from your service provider)
- 2 Normal Docker Hosts running on two different server
- 1 Synology NAS Docker Host

## Config
```bash
# Used subnet: 192.168.1.0
# The Aux addresses are the IP addresses of the respective Docker host.
# The sudo ip commands below need to be repeated after a reboot!

################################################################################
# router 50 % from address range

Address:   192.168.1.0           11000000.10101000.00000001.0 0000000
Netmask:   255.255.255.128 = 25  11111111.11111111.11111111.1 0000000
Wildcard:  0.0.0.127             00000000.00000000.00000000.0 1111111
=>
Network:   192.168.1.0/25        11000000.10101000.00000001.0 0000000 (Class C)
Broadcast: 192.168.1.127         11000000.10101000.00000001.0 1111111
HostMin:   192.168.1.1           11000000.10101000.00000001.0 0000001
HostMax:   192.168.1.126         11000000.10101000.00000001.0 1111110
Hosts/Net: 126                   (Private Internet)

################################################################################
# 25% synology

Address:   192.168.1.128         11000000.10101000.00000001.10 000000
Netmask:   255.255.255.192 = 26  11111111.11111111.11111111.11 000000
Wildcard:  0.0.0.63              00000000.00000000.00000000.00 111111
=>
Network:   192.168.1.128/26      11000000.10101000.00000001.10 000000 (Class C)
Broadcast: 192.168.1.191         11000000.10101000.00000001.10 111111
HostMin:   192.168.1.129         11000000.10101000.00000001.10 000001
HostMax:   192.168.1.190         11000000.10101000.00000001.10 111110
Hosts/Net: 62                    (Private Internet)

# synology docker macvlan settings
Subnetz 192.168.1.0/24
IP Bereich 192.168.1.128/26
Gateway 192.168.1.1

# create macvlan docker network with specific subnet and address range
# DHCP server of the host network should not assign configured ip-range
# on host bob
# check config with: sudo docker network ls
sudo docker network create -d macvlan -o parent=eth0 \
  --subnet 192.168.1.0/24 \
  --gateway 192.168.1.1 \
  --ip-range 192.168.1.128/26 \
  --aux-address 'host=192.168.1.11' \
  bobnet

# login as admin on synology NAS
# add macvlan host access between docker host and container
sudo ip link add bobnet-host link eth0 type macvlan mode bridge
sudo ip addr add 192.168.1.11/32 dev bobnet-host
sudo ip link set bobnet-host up
sudo ip route add 192.168.1.128/26 dev bobnet-host

################################################################################
# 12,5% bob

Address:   192.168.1.192         11000000.10101000.00000001.110 00000
Netmask:   255.255.255.224 = 27  11111111.11111111.11111111.111 00000
Wildcard:  0.0.0.31              00000000.00000000.00000000.000 11111
=>
Network:   192.168.1.192/27      11000000.10101000.00000001.110 00000 (Class C)
Broadcast: 192.168.1.223         11000000.10101000.00000001.110 11111
HostMin:   192.168.1.193         11000000.10101000.00000001.110 00001
HostMax:   192.168.1.222         11000000.10101000.00000001.110 11110
Hosts/Net: 30                    (Private Internet)

# create macvlan docker network with specific subnet and address range
# DHCP server of the host network should not assign configured ip-range
# on host bob
# check config with: sudo docker network ls
sudo docker network create -d macvlan -o parent=enp3s0 \
  --subnet 192.168.1.0/24 \
  --gateway 192.168.1.1 \
  --ip-range 192.168.1.192/27 \
  --aux-address 'host=192.168.1.36' \
  bobnet

# add macvlan host access between docker host and container
sudo ip link add bobnet-host link enp3s0 type macvlan mode bridge
sudo ip addr add 192.168.1.36/32 dev bobnet-host
sudo ip link set bobnet-host up
sudo ip route add 192.168.1.192/27 dev bobnet-host

################################################################################
# 12,5 % bob2

Address:   192.168.1.224         11000000.10101000.00000001.111 00000
Netmask:   255.255.255.224 = 27  11111111.11111111.11111111.111 00000
Wildcard:  0.0.0.31              00000000.00000000.00000000.000 11111
=>
Network:   192.168.1.224/27      11000000.10101000.00000001.111 00000 (Class C)
Broadcast: 192.168.1.255         11000000.10101000.00000001.111 11111
HostMin:   192.168.1.225         11000000.10101000.00000001.111 00001
HostMax:   192.168.1.254         11000000.10101000.00000001.111 11110
Hosts/Net: 30                    (Private Internet)

# create macvlan docker network with specific subnet and address range
# DHCP server of the host network should not assign configured ip-range
# on host bob2
# check config with: sudo docker network ls
sudo docker network create -d macvlan -o parent=enp3s0 \
  --subnet 192.168.1.0/24 \
  --gateway 192.168.1.1 \
  --ip-range 192.168.1.224/27 \
  --aux-address 'host=192.168.1.47' \
  bobnet

# add macvlan host access betwwen docker host and container
sudo ip link add bobnet-host link enp3s0 type macvlan mode bridge
sudo ip addr add 192.168.1.47/32 dev bobnet-host
sudo ip link set bobnet-host up
sudo ip route add 192.168.1.224/27 dev bobnet-host
```
