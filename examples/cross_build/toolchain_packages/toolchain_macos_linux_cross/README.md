# Creating a Toolchain Package for Cross-Building from macOS to Linux

## Introduction

This example presents a detailed guide for creating and utilizing a toolchain package to facilitate cross-compilation 
from *macOS* to *Linux*. The example is limited to cross-compiling from a *MacOs* with either `x86_64` or `aarch64` 
architecture to a *Linux GNU* system, also with `x86_64` or `aarch64`.

## Setup

To run the following example, you need:

- *Conan* installed
- *MacOS* machine with an `x86_64` or `aarch64` architecture
- *MacOS* build profile with an `x86_64` or `aarch64` architecture
- *Linux* host profile with an `x86_64` or `aarch64` architecture

Here you can see an example of the *MacOs* build profile:
```
[settings]
arch=armv8
build_type=Release
compiler=apple-clang
compiler.cppstd=gnu17
compiler.libcxx=libc++
compiler.version=15
os=Macos
```
Here you can see and example of the *Linux* host profile:
```
[settings]
os=Linux
arch=armv8
compiler=gcc
compiler.cppstd=gnu14
compiler.libcxx=libstdc++11
compiler.version=13
build_type=Release
```

## How to run it

The first step will be to navigate to the "*toolchain_macos_linux_cross*" folder. Once there, we will use the command:

``conan create . -pr:b=default -pr:h=../profiles/raspberry-64 --build-require``

We are assuming that this `default` profile is similar to the *MacOS* profile we defined earlier.

## More

You can find more information about a similar recipe where each method is explained in detail and how it is built 
in the documentation. [Docs](https://docs.conan.io/2/examples/cross_build/toolchain_packages.html)

