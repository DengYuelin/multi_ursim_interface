# Docker quick tutorial 

## installation 

### optional: uninstall old version
Follow instructions on https://docs.docker.com/engine/install/ubuntu/#uninstall-old-versions

### easy setup
Go to https://get.docker.com/
```bash
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ sh get-docker.sh
```

To verify installation:
```bash
# start docker (should automatically start)
$ sudo systemctl enable docker
$ sudo systemctl start docker
$ sudo docker run --rm hello-world
```

[To get rid of sudo](https://docs.docker.com/engine/install/linux-postinstall/) command:
```bash
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
```
Log out and log back in so that your group membership is re-evaluated.

## [Basic usage](https://docs.docker.com/reference/)

### General commands
```bash
$ docker version
$ docker info
$ docker cmd --help
```

### Image commands
```bash
$ docker images # check all the locally installed images
$ docker images --help # check all images commands

$ docker pull image[:tag] # get a image from docker hub

$ docker rmi imageID # rmi -> remove image
# force delete all images
$ docker rmi -f $(docker images -aq)
```

### Container commands

#### Create and run
Generate a container through an image.
```bash
$ docker run [args] image[:tag] 
# args
--name="Name"       # Name it!
-d                  # Run in background
-it                 # Interactive (via terminal)
-p                  # Publish a container's port(s) to the host
-rm                 # delete after closed
```
Example: run and interact with ROS container
```bash
# $ docker run -it osrf/ros:melodic-desktop-full /bin/bash
$ docker run -it ros:melodic /bin/bash
```

Exit a container (if ran in interactive mode)
```bash
root@d9812305cc24:/# exit               # exit and stop
root@d9812305cc24:/# Crtl + P + Q       # just exit
```


#### Interact with container
List all running/existing container
```bash
$ docker ps
$ docker ps -a
```

Delete a container 
```bash
$ docker rm containerID [containerName]
```

Restart/stop a container
```bash
$ docker start containerID              # start a stopped container
$ docker restart containerID            # restart a running container
$ docker stop containerID
$ docker kill containerID               # force stop
```

Enter an running container
```bash
$ docker exec -it containerID /bin/bash # this will create a new terminal
$ docker attach containerID             # we don't need this, this will only attach to the previous running process
```

Copy a file to and from container
```bash
$ docker cp containerID:/root/test.py /home/userName/docker_ws/
```

Connect docker with host (network)
```bash
# args
--net=host
```

Connect docker with host (file)
```bash
# args 
-v /host/dir:/container/dir
```

#### Create my image
```bash
$ docker commit -m="Info" -a="author" containerID targetImageName[:tag]
```