#README

##Dependencies:

* eigen 3
* OpenCV 2.x
* cmake 

in Ubuntu these can be obtained by executing:
```bash
sudo apt-get install libeigen3-dev libopencv-dev cmake
```

##Building
Building the project **should** be as simple as executing the following commands (from the project directory)
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
should run a short demo. you can then edit main.cpp or use the library in other projects. 

send criticism, comments, praise and requests to daniel.canelhas@oru.se