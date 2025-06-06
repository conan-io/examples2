# WASM simple game

This project is a copy of our [raylib](https://github.com/conan-io/examples2/tree/main/examples/libraries/raylib/introduction) example project.

It has minor modifications:

- C++ source code has been slightly modified in order to accept mouse clicks as inputs as it is more convenient for a webpage
- `CMakeLists.txt` has been modified in order to generate a `html` target which can bee opened by a web browser
- `conanfile.py` `generate()` method has been modified in order to link against raylib correctly using WASM
    - `-sUSE_GLFW=3 -sASYNCIFY`: see [raylib web manual](https://github.com/raysan5/raylib/wiki/Working-for-Web-(HTML5)#23-using-cmake) for a deeper explanation
    - `--shell-file`: generate the html code from the template

![image](https://github.com/user-attachments/assets/941b5922-edb6-400f-9d38-ba805fc8a3ab)


## Build and run

See instructions in [bindings example](https://github.com/conan-io/examples2/tree/main/examples/cross_build/wasm/bindings)
