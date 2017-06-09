# README

## Dependencies:

* Eigen 3
* cmake 
* gcc 4.8 or higher

in Ubuntu these can be obtained by executing:
```bash
sudo apt-get install libeigen3-dev cmake
```

## Building
Building the project under Linux or MacOSX **should** be as simple as executing the following commands (from the project directory)
```bash
mkdir build
cd build
cmake ..
make
```

If the code compiles successfully there is a small demo application in the bin directory, 
```bash
../bin/demo_app
```
should run a short demo. The display function used in the demo may require linking to some other library than X11 for Windows users. You can then edit main.cpp or use the library in other projects. 

send criticism, comments, praise and requests to daniel.canelhas@oru.se
