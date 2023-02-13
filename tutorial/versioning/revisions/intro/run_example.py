import platform
from test.examples_tools import run, replace, chdir

from conan import conan_version


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

    # FIXME: remove after beta10 is released or use the json output to parse the rrev
    hello_rrev = "2475ece651f666f42c155623228c75d2" if "beta9" in str(conan_version) else "6b908be14391834776ffc2f42ea07cb7"

    replace("conanfile.py", 'self.requires("hello/1.0")',
                            f'self.requires("hello/1.0#{hello_rrev}")')
    run("conan create .")

with chdir("hello"):
    run("conan create .")
    run("conan create .")
    run("conan list hello/1.0:*#*")
