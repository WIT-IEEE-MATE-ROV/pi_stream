# RPI zero streamer
 
RPI zeros UDP client video streamer for WUROV

## Setup and Build


**Install dependencies**

```bash
$ sudo apt install cmake lua5.4 v4l-utils gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```

**Build opencv**

Inorder to build this application you will **need** to [build opencv from source](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) 

**Install Premake**

Before we can build the `pi_stream` binary we need to [download premake5](https://premake.github.io/download). 
This can be done with your systems packagemanager, such as `apt` or `pacman`. 

```bash
$ sudo pacman -S premake
```
**Install Premake From Tarball**

If your package manager dose not maintain premake5 you can install from the tarball. 

[download premake5](https://premake.github.io/download) from the website unzip the tarball and change permissions. 
 
```bash
$ tar -xzvf premake-5.0.0-beta3-linux.tar.gz
$ sudo chmod +x premake5
```

You can add the premake5 binary to your path but this is not necessary. you could just move the latest premake5 binary into this project folder. 

Add the flowing line to your shells respective `.bashrc` file to add preamke to your path.
```bash
export PATH="$PATH:/path/to/premake5"
```
