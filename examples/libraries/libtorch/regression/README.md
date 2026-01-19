## Libtorch example with Conan

This example shows how to use Libtorch with Conan package manager to build a simple regression model.

The regression source code is not hosted in this repository,
we use [the official PyTorch examples repository](https://github.com/pytorch/examples/tree/main/cpp/regression),
so make sure to clone it first and then navigate to the `cpp/regression` folder:

```bash
$ git clone https://github.com/pytorch/examples/tree/main/cpp/regression
$ cd cpp/regression
```

Then, copy the `conanfile.txt` from this directory to the `cpp/regression` folder,
and finally run Conan and CMake as usual:

```bash
$ conan install --build=missing
$ cmake --preset conan-release  # conan-default for Windows
$ cmake --build --preset conan-release
$ ./build/Release/regression
```
