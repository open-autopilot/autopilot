# Agro Autopilot
This project is an autopilot designed to be run on all kinds of hardware for the agro conditions. 

## getting started
First step is to install WSL on windows. The scripts are not made to run nativily on windows or linux. 
So if you want to use them on those you need to change the code. 
The distro that is required to run this project is Ubuntu 22.04.

To be able to use this project you first need to install docker. 
You can install docker by following this tutorial [Ubuntu Install Docker](https://docs.docker.com/engine/install/ubuntu/).

After you have install docker make sure to run the following commands. 
``` 
$ sudo apt install qemu-user-static
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
$ sudo docker buildx create --use
```

Restart the WSL after running the commands from above and now you can use the project. 

## buildtools
To build this project you need to use the 'buildtools'. There you have multiple scripts. To see what they can do you can run --help or see below. 

The tools available are: 
- build.sh (options --service | -s, --platform | -p)
- deploy.sh (options --service | -s, --no-unpack) 
- transfer.sh (options --service | -s, --platform | -p)

With these tools you can build this autopilot for any platform. Although you may need to add docker to your user group. 

## documentation
You can find all other documentation about this autopilot at [Documentation](https://open-autopilot.github.io/autopilot/build/).
