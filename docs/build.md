# Build Procedures
To deploy this project there are a certain steps that need to be followed. 
You build the images on a powerfull windows computer with WSL or a Linux computer. 
Then you transfer them to your less powerfull Companion PC to run the code there.

## On WSL or Linux
### Install Docker
First you need to make sure you have installed docker on the machine. 
You can do this by following this tutorial: [Install Docker](https://docs.docker.com/engine/install/ubuntu/). 

### Install QEMU
To be able to build multiplatform images,  another tool is needed. This tool is called QEMU. 
You can install by running the following commands.
```bash
sudo apt install qemu-user-static
```

### Clone the repository
Clone the repository so we have acces to the buildtools. 
And so we also have access to the source code. You also might need to create a build dir. 
You can do this form autopilot/buildtools/build. 

### Build images
You can easily build the images with the build scripts provided. 
Make sure your in the autopilot directory when you run them. 
This build process is quite heavy. Make sure you have atleast 16GB RAM & 8 Threads. 
```bash
# ... = linux/arm64 or linux/amd64
./buildtools/build.sh -p ...
```
The resulted files are saved in the .tar format. They are not compressed. 

### Transfer images
To tranfer the images make sure you select the platform you want to transfer. 
Make sure your in the autopilot directory when you run them. 
```bash
# ... = linux/arm64 or linux/amd64
# ... = wsl or linux
./buildtools/transfer.sh -p ... -t ...
```
The -t says what type of tranfser to use either scp for windows or rsync for linux. 

## On Companion PC
### Install Docker
First you need to make sure you have installed docker on the machine. 
You can do this by following this tutorial: [Install Docker](https://docs.docker.com/engine/install/ubuntu/).

### Clone the repository
Clone the repository so we have acces to the buildtools. You also might need to create a build dir. 
You can do this form autopilot/buildtools/build. 

### Unpack images
To tranfer the images make sure you select the platform you want to transfer. 
Make sure your in the autopilot directory when you run them. 
```bash
./buildtools/unpack.sh
```
The -t says what type of tranfser to use either scp for windows or rsync for linux. 