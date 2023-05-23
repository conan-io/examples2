# Conan Protobuf Example

## Protobuf example using Conan for blog post

- Conan.io blog: https://blog.conan.io

#### How to build
To build this project using cmake:

    git clone https://github.com/conan-io/examples.git conan-examples
    cd conan-examples/libraries/protobuf/serialization
    mkdir build && cd build
    conan install ..
    cmake ..
    cmake --build .
    bin/sensor

#### Requirements
- CMake >=3.1.3
- C++ compiler with C++11 support (Protobuf requirement)
- Conan >=1.9.1
