# dual_ursim
Dual UR arm simulation with moveit control

## Setup
### URSim
Clone docker image
```bash
docker pull universalrobots/ursim_e-series
cd dockerursim
docker build -t my/dockerursim .
docker network create --subnet=192.168.56.0/24 ursim_net
```

Firewall
```bash
sudo ufw status
sudo ufw allow 50002
sudo ufw allow 50012
```

### Controller

docker run --rm -it -p 5900:5900 -p 6080:6080 --net=ursim_net --ip 192.168.56.101 --add-host=host.docker.internal:host-gateway --name ursim_left_arm my/dockerursim
http://localhost:6080/vnc.html?host=localhost&port=6080


docker run --rm -it -p 5901:5900 -p 6081:6080 --net=ursim_net --ip 192.168.56.102 --add-host=host.docker.internal:host-gateway --name ursim_right_arm my/dockerursim
http://localhost:6081/vnc.html?host=localhost&port=6081