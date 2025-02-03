## Conan recipe tools examples

### [tools.cmake](cmake)

- CMakeToolchain: [Building your project using CMakePresets](cmake/cmake_toolchain/local_flow_cmake_presets/). [Docs](https://docs.conan.io/2/examples/tools/cmake/cmake_toolchain/build_project_cmake_presets.html)

- CMakeToolchain: [Extending your CMakePresets with Conan generated ones](cmake/cmake_toolchain/extend_own_cmake_presets/). [Docs](https://docs.conan.io/2/examples/tools/cmake/cmake_toolchain/extend_own_cmake_presets.html)

- CMakeToolchain: [Inject CMake variables via profile using user_toolchain](cmake/cmake_toolchain/user_toolchain_profile/). [Docs](https://docs.conan.io/2/examples/tools/cmake/cmake_toolchain/inject_cmake_variables.html)

### [tools.files](files)

- Learn how [patch sources](files/patches/). [Docs](https://docs.conan.io/2/examples/tools/files/patches/patch_sources.html)

### [tools.meson](meson)

- Build a [simple Meson project](meson/mesontoolchain/simple_meson_project/) using Conan. [Docs](https://docs.conan.io/2/examples/tools/meson/mesontoolchain/build_simple_meson_project.html)

### [tools.autotools](autotools)

- Build a [Autotools project](autotools/autotoolstoolchain/string_formatter/) using Conan and [fmt](https://fmt.dev/). [Docs](https://docs.conan.io/2/examples/tools/autotools/autotools_toolchain/build_project_autotools_toolchain.rst)

### [tools.google](google)

- Build a [Bazel 6.x compatible project](bazel/bazeltoolchain/6_x/string_formatter/) using Conan and [fmt](https://fmt.dev/). [Docs](https://docs.conan.io/2/examples/tools/google/bazeltoolchain/build_simple_bazel_project.rst)
- Build a [Bazel >= 7.1 compatible project](bazel/bazeltoolchain/7_x/string_formatter/) using Conan and [fmt](https://fmt.dev/). [Docs](https://docs.conan.io/2/examples/tools/google/bazeltoolchain/build_simple_bazel_7x_project.rst)

### [tools.ros](ros)

- Build [ROS packages inside their workspace](ros/rosenv/workspace) using the [fmt library from Conan Center](https://conan.io/center/recipes/fmt) and consuming them also as transitive dependencies.
- Build a [ROS navigation package](ros/rosenv/navigation_ws) that sends goals to a mobile robot from a YAML file using the [yaml-cpp library from Conan Center](https://conan.io/center/recipes/yaml-cpp).

### [tools.system](system)

- Wrap a [system package](system/package_manager/) using Conan and [ncurses](https://invisible-island.net/ncurses/). [Docs](https://docs.conan.io/2/examples/tools/system/package_manager.html)