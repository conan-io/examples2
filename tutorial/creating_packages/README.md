
## Creating packages

### [How to create your first Conan package](first_package/)

- Learn how to create a simple "Hello World" Conan package using the ``conan new`` command. [Docs](https://docs.conan.io/2/tutorial/creating_packages/create_your_first_package.html)

### [How to handle external sources in recipes](handle_sources/)

- Learn how to modify the previous example to bring source files form a zip file located
  in a remote server or from a git repository.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/handle_sources_in_packages.html)

### [How to add requirements to Conan packages](add_requires/)

- Learn how to add requirements to a Conan package.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/add_dependencies_to_packages.html)

### [How to use the generate() method to prepare the Conan package build](preparing_the_build/)

- Learn how to configure the toolchain in the generate() method.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/preparing_the_build.html)

### [Configure settings and options in recipes](configure_options_settings/)

- Learn how to configure settings and options and how Conan models binary compatibility.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/configure_options_settings.html)

### [Build packages: the build() method](build_method/)

- Learn how to build and run tests, conditionally patch the sources and select the build system.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/creating_packages/build_packages.html)

### [Package files: the package() method](package_method/)

- Learn to copy package files into the Conan local cache.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/package_method.html)

### [Define the package information for consumers: the package_info() method](package_information/)

- Define the information that consumers of a package need to build.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/define_package_information.html)

### [Testing Conan packages](testing_packages/)

- Use a *test_package* to test that the Conan package can be consumed correctly.
  [Docs](https://docs.conan.io/2/tutorial/creating_packages/test_conan_packages.html)

## Creating packages: other types of packages

### [How to create a Conan package for a header-only library](other_packages/header_only/)

- Learn how to create a Conan package for a simple header-only library. [Docs](https://docs.conan.io/2/tutorial/creating_packages/other_types_of_packages/header_only_packages.html)

### [How to create a Conan package for a header-only library but testing the library](other_packages/header_only_gtest/)

- Learn how to create a Conan package for a simple header-only library that needs the settings. [Docs](https://docs.conan.io/2/tutorial/creating_packages/other_types_of_packages/header_only_packages.html#header-only-library-with-tests)

### [How to create a Conan package for binaries built locally](other_packages/prebuilt_local_project/)

- Learn how to create a Conan package when we are building our project with our IDE and we want to directly package 
  the binaries without calling "conan create". [Docs](https://docs.conan.io/2/tutorial/creating_packages/other_types_of_packages/package_prebuilt_binaries.html#locally-building-binaries)

### [How to create a Conan package for prebuilt binaries](other_packages/prebuilt_binaries/)

- Learn how to create a Conan package when we have prebuilt libraries (like third-party vendors). [Docs](https://docs.conan.io/2/tutorial/creating_packages/other_types_of_packages/package_prebuilt_binaries.html#packaging-already-pre-built-binaries)

### [How to create a Conan package for prebuilt binaries in a remote](other_packages/prebuilt_remote_binaries/)

- Learn how to create a Conan package when we have prebuilt libraries located in a remote repository. [Docs](https://docs.conan.io/2/tutorial/creating_packages/other_types_of_packages/package_prebuilt_binaries.html#downloading-and-packaging-pre-built-binaries)
