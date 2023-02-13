import platform
from test.examples_tools import run, replace, chdir

run("conan remove hello* -c")
run("conan remove chat* -c")

with chdir("hello"):
    run("conan new cmake_lib -d name=hello -d version=1.0")
    run("conan create .")
    run("conan list hello/1.0#*")
    # new revision, change in code
    replace("src/hello.cpp", "Hello World", "Bye World")
    run("conan create .")
    run("conan list hello/1.0#*")
    # New revision, recipe change
    replace("conanfile.py", 'license = "<Put the package license here>"', 'license = "MIT"')
    run("conan create .")
    run("conan list hello/1.0#*")

with chdir("chat"):
    run("conan new cmake_lib -d name=chat -d version=1.0 -d requires=hello/1.0")
    run("conan create .")
    replace("conanfile.py", 'self.requires("hello/1.0")',
                            'self.requires("hello/1.0#e6f5d545a9629164fa57681be926f727")')
    run("conan create .")

with chdir("hello"):
    run("conan create .")
    run("conan create .")
    run("conan list hello/1.0:*#*")
