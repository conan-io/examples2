# Conan 2.0 examples

## Tutorial

### Consuming packages

#### [Build a simple CMake project using Conan](tutorial/consuming_packages/simple_cmake_project/)

- Use Conan to manage dependencies for a simple application, a string compressor that uses Zlib. [Docs](https://docs.conan.io/en/2.0/tutorial/consuming_packages/build_simple_cmake_project.html)

#### [Using build tools as Conan packages](tutorial/consuming_packages/tool_requires/)

- Use Conan to install and use build tools like CMake. [Docs](https://docs.conan.io/en/2.0/tutorial/consuming_packages/use_tools_as_conan_packages.html)

#### [Building for multiple configurations: Release, Debug, Static and Shared](tutorial/consuming_packages/different_configurations/)

- Learn how to build for different configurations, like Release or Debug and build shared or static libraries. [Docs](https://docs.conan.io/en/2.0/tutorial/consuming_packages/different_configurations.html)

#### [Using conanfile.py vs conanfile.txt](tutorial/consuming_packages/conanfile_py/)

- Learn about the flexibility of using a conanfile.py instead a conanfile.txt. [Docs](https://docs.conan.io/en/2.0/tutorial/consuming_packages/the_flexibility_of_conanfile_py.html)

#### [How to cross-compile your applications using Conan](tutorial/consuming_packages/cross_building/)

- Learn how to cross-compile your applications with Conan. [Docs](https://docs.conan.io/en/2.0/tutorial/consuming_packages/cross_building_with_conan.html)

#### [Introduction to versioning](tutorial/consuming_packages/versioning/)

- Learn how to use different versions, version ranges and revisions. [Docs](https://docs.conan.io/en/2.0/tutorial/consuming_packages.html)

### Creating packages

#### [How to create your first Conan package](tutorial/creating_packages/first_package/)

- Learn how to create a simple "Hello World" Conan package using the ``conan new`` command. [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/create_your_first_package.html)

#### [How to handle external sources in recipes](tutorial/creating_packages/handle_sources/)

- Learn how to modify the previous example to bring source files form a zip file located
  in a remote server or from a git repository.
  [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/handle_sources_in_packages.html)

#### [How to add requirements to Conan packages](tutorial/creating_packages/add_requires/)

- Learn how to add requirements to a Conan package.
  [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/add_dependencies_to_packages.html)

#### [How to use the generate() method to prepare the Conan package build](tutorial/creating_packages/preparing_the_build/)

- Learn how to configure the toolchain in the generate() method.
  [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/preparing_the_build.html)

#### [Configure settings and options in recipes](tutorial/creating_packages/configure_options_settings/)

- Learn how to configure settings and options and how Conan models binary compatibility.
  [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/configure_options_settings.html)

#### [Build packages: the build() method](tutorial/creating_packages/build_method/)

- Learn how to build and run tests, conditionally patch the sources and select the build system conditionally.
  [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/creating_packages/build_packages.html)

#### [Package files: the package() method](tutorial/creating_packages/package_method/)

- Learn to copy package files into the Conan local cache.
  [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/package_method.html)

#### [Define the package information for consumers: the package_info() method](tutorial/creating_packages/package_information/)

- Define the information that consumers of a package need to build.
  [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/define_package_information.html)

### Creating packages: other types of packages

#### [How to create a Conan package for a header-only library](tutorial/creating_packages/other_packages/header_only/)

- Learn how to create a Conan package for a simple header-only library. [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/other_types_of_packages/header_only_packages.html)

#### [How to create a Conan package for a header-only library but testing the library](tutorial/creating_packages/other_packages/header_only_gtest/)

- Learn how to create a Conan package for a simple header-only library that needs the settings. [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/other_types_of_packages/header_only_packages.html#header-only-library-with-tests)

#### [How to create a Conan package for binaries built locally](tutorial/creating_packages/other_packages/prebuilt_local_project/)

- Learn how to create a Conan package when we are building our project with our IDE and we want to directly package 
  the binaries without calling "conan create". [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/other_types_of_packages/package_prebuilt_binaries.html#locally-building-binaries)

#### [How to create a Conan package for prebuilt binaries](tutorial/creating_packages/other_packages/prebuilt_binaries/)

- Learn how to create a Conan package when we have prebuilt libraries (like third-party vendors). [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/other_types_of_packages/package_prebuilt_binaries.html#packaging-already-pre-built-binaries)

#### [How to create a Conan package for prebuilt binaries in a remote](tutorial/creating_packages/other_packages/prebuilt_remote_binaries/)

- Learn how to create a Conan package when we have prebuilt libraries located in a remote repository. [Docs](https://docs.conan.io/en/2.0/tutorial/creating_packages/other_types_of_packages/package_prebuilt_binaries.html#downloading-and-packaging-pre-built-binaries)

## Examples

### [Use custom commands in your Conan CLI](examples/extensions/commands/)

- Learn how to create custom commands in Conan. [Docs](https://docs.conan.io/en/2.0/reference/commands/custom_commands.html)

    * ``conan clean`` [command](examples/extensions/commands/clean/cmd_clean.py): Deletes (from local cache or remotes) all recipe and package revisions but the latest package revision from the latest recipe revision. [Docs](https://docs.conan.io/en/2.0/examples/extensions/commands/clean/custom_command_clean_revisions.html)

### [Use Android NDK to cross-build](examples/cross_build/android/ndk_basic)

- Learn how to cross-build packages for Android. [Docs](https://docs.conan.io/en/2.0/examples/cross_build/android.html)