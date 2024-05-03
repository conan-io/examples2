# Demonstrates how to run gtest with Bazel >= 7.1.

## Prerequisites
Either bazel 7.1 or later version. Or you can use the bazelisk to run bazel 7.1 https://github.com/bazelbuild/bazelisk.

## How to
```bash
conan install . --build=missing
bazel test //...
```
