# Protobuf Serialization Example with Conan v2

## Protobuf example using Conan for blog post

- Conan.io blog: https://blog.conan.io
- Blog post: https://blog.conan.io/2019/03/06/Serializing-your-data-with-Protobuf.html

#### How to build

To build this project using cmake on Linux or Mac:

```
    git clone https://github.com/conan-io/examples2.git conan-examples2
    cd conan-examples2/examples/libraries/protobuf/serialization
    pip install -r requirements.txt
    mkdir build && cd build/
    conan install .. -pr:h=default -pr:b=default --build=missing
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=Release/generators/conan_toolchain.cmake
    cmake --build .

    ./sensor

    PYTHONPATH=${PWD} python ../main.py
```

To build this project using cmake on Windows:

```
    git clone https://github.com/conan-io/examples2.git conan-examples2
    cd conan-examples2/examples/libraries/protobuf/serialization
    pip install -r requirements.txt
    mkdir build && cd build/
    conan install .. -pr:h=default -pr:b=default --build=missing
    cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake
    cmake --build . --config Release

    Release\sensor.exe

    SET PYTHONPATH="%CD%"
    python ../main.py
```

#### Requirements
- CMake >=3.15
- C++ compiler with C++14 support (Protobuf requirement)
- Conan >=2.0.0
