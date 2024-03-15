from test.examples_tools import run


print("- CMakeToolchain: Locating in-package xxx-config.cmake files instead of using CMakeDeps -")

run("conan create .")
run("conan install . -s build_type=Debug")
