# CPSwarm Communication Library
[![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/cpswarm/deployment-tool.svg)](https://github.com/cpswarm/swarmio/tags)
[![Build Status](https://travis-ci.com/cpswarm/deployment-tool.svg?branch=master)](https://travis-ci.com/cpswarm/swarmio)  
The CPSwarm Communication Library provides a unified interface that tools and swarm members can use to interact with each other.
## Getting started
For more information, see the [WIKI](https://github.com/cpswarm/swarmio/wiki).

## Deployment
Packages are built continuously with [Bamboo](https://pipelines.linksmart.eu/browse/CPSW-ALS/latest).

## Build
### Building on Linux
Build should work on any Linux distribution, but packaging
and architecture detection will only work on Debian and 
Ubuntu. By default, everything is compiled for the host 
architecture, but cross-compilation is available. Currently 
armhf and arm64 are supported if the required tools are 
available. The default compiler is clang, but GCC can also 
be used. Builds were tested with Ubuntu 16.04 and 18.04,
with all the latest updates installed.

There are three build modes you can choose from:

     - DEVELOPMENT (default)

 
 Development mode will build and install all components into
 one subdirectory under the build directory. Useful when an
 IDE needs a full include path and for debugging. By default,
 products are installed into the "devtree-<ARCH>" subdirectory.
 

     - PACKAGE

 
 Package mode will build .deb packages for each target -
 and set /opt/swarmio as the install prefix. Find the packages
 under the "packages" subdirectory.
 

     - INSTALL

 
 Install will install all outputs onto the local machine,
 by default under /opt/swarmio.
 
To build, create a new directory outside the source tree, 
and issue the following commands inside it:

     cmake <PATH TO SOURCE> [optional parameters]
     cmake --build .

 
Optional parameters:

     -DSWARMIO_BUILD_MODE=<MODE> (DEVELOPMENT, PACKAGE or INSTALL)

 Specifies build mode (see above). DEVELOPMENT build will
 generate debug executables, while PACKAGE and INSTALL builds
 will compile with release flags (and debug symbols).

     -DCMAKE_BUILD_TYPE=<TYPE> (Debug, Release, RelWithDebInfo)

 Override build type derrived from build mode. See CMake docs
 for more info.

     -DCMAKE_INSTALL_PREFIX=<PATH>

 Overrides the installation directory. For DEVELOPMENT and
 INSTALL build, this will override the output directory. For
 PACKAGE builds, it will override the installation prefix 
 of the generated packages.

     -DSWARMIO_TARGET_ARCHITECTURE=<ARCH> (armhf or arm64)

 Enables cross-compilation - required tools must already be
 installed. On Ubuntu systems, crossbuild-essential-<ARCH>
 packages should do the trick.

     -DSWARMIO_BUILD_ROS_NODE=ON

 If turned on, swarmros will be built.

     -DSWARMIO_MULTISTRAP_CONFIGURATION=<CONFIG>

 If configured, a full multistrap sysroot will be initialized
 before cross-compilation. Requires multistrap and a bunch of 
 other tools to be installed. Currently supported configurations:
  - xenial (Ubuntu 16.04)
  - xenial-ros (Ubuntu 16.04 with ROS Kinetic Kame)
  - bionic (Ubuntu 18.04)
  - bionic-ros (Ubuntu 18.04 with ROS Melodic Morenia)

```	
-DSWARMIO_SYSROOT=<SYSROOT PATH>
```

 If multistrap is not used, a sysroot needs to be manually set
 for cross-compilation to work. Ensure that the system image
 contains no absolute symbolic links before using it.

     -DSWARMIO_GCC_VERSION=<GCC VERSION>

 If multistrap is not used, this should be set to indicate the
 GCC version present in the manually specified sysroot in order
 to help compilers find the correct libraries to link against.

     -DSWARMIO_ROS_PREFIX=<PATH>

 Specifies the location of the ROS installation to use when 
 building the ROS node. If not specified, the script will try
 to automatically detect it - by looking for the default 
 installation directory of Kinetic Kame and Melodic Morenia.

     -DSWARMIO_PREFER_GCC=ON

 If specified, GCC will be used instead of clang. Please note
 that cross-compilation will most likely only work if the
 same operating system (with a different architecture) is
 used on the host machine. Requires GCC 6 or later.
 
### Building on Windows

On Windows, only DEVELOPMENT mode is supported. Building the
ROS node and multistrap environments are not supported. Basic
build command is the same as on Linux:

     cmake <PATH TO SOURCE> [optional parameters]
     cmake --build .

 
Optional parameters:

     -DCMAKE_BUILD_TYPE=<TYPE> (Debug, Release, RelWithDebInfo)

 Overrides the default build mode (Debug).

     -DCMAKE_INSTALL_PREFIX=<PATH>

 Overrides the output directory. 

     -DSWARMIO_TARGET_ARCHITECTURE=<ARCH> (x86 or x64)

 Specifies whether to generate 32 or 64 bit builds.


### Quick example


If you want to generate packages for an armhf target, with
ROS support turned on, targeting Ubuntu Xenial:

    cmake <SOURCE_DIR> -DSWARMIO_BUILD_MODE=PACKAGE
                       -DSWARMIO_TARGET_ARCHITECTURE=armhf
                       -DSWARMIO_MULTISTRAP_CONFIGURATION=xenial-ros
                       -DSWARMIO_BUILD_ROS_NODE=ON
    cmake --build .

All output packages will be placed into the packages subdirectory.

### Future plans
Add security features (Due in November 2019).

The security functionalities are to be provided by libsodium (based on NaCl) Libsodium is a popular solution for crypto library used by e.g.: WordPress, Discord, Secrets, Remembear. 
All cryptographic functions are based on:
- Edwards-Curve Digital Signature Algorithm (EdDSA)
- Encryption: XSalsa20 stream cipher
- Authentication: Poly1305 MAC

The following security dimensions will be addressed:
- Deployment tool is able to securely provision new node members (by generating their keys and signing their certificates)
- Access control is provided for provisioned nodes by certificate checking, using a pre-shared signing key
- Authentication is provided by signature checking
- Non-repudiation is provided by signature and timestamp checking for each packet
- Confidentiality is provided end-to-end by payload encryption 
- Integrity checking is provided by using a tag for packet integrity
- Availability is maintained using each nodes security table, which stores valid authentication credentials.

## Contributing
Contributions are welcome. 

Please fork, make your changes, and submit a pull request. For major changes, please open an issue first and discuss it with the other authors.

## Affiliation
![CPSwarm](https://github.com/cpswarm/template/raw/master/cpswarm.png)  
This work is supported by the European Commission through the [CPSwarm H2020 project](https://cpswarm.eu) under grant no. 731946.
