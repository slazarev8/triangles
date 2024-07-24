This is an alghorithm to check whether 3D triangles intersect. An input format for test cases is: 
```
#it supports comments
0 0 0 10 0 0 0 10 0
1 1 0 1 1 0 1 1 0
```
A test file is provided in the tests folder.
Also it supports degenerate triangles (segment and dot) 

To build an application: 
```
mkdir build
cd build
cmake ..
cmake --build .
```

Usage:
./Triangles "path_to_test_file"

The program uses Separating axis theorem https://dyn4j.org/2010/01/sat/

Also some approaches from https://paulbourke.net/geometry/pointlineplane/#i2l are used as well
