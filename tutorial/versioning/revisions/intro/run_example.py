import json
import platform
from test.examples_tools import run, replace, chdir, load

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

    run("conan list hello/1.0#* --format=json > output.json")

    data = json.loads(load("output.json"))
    
    revisions = data["Local Cache"]["hello/1.0"]["revisions"]
    sorted_revisions = sorted(revisions, key=lambda x: revisions[x]["timestamp"])

    hello_rrev = sorted_revisions[-1] # oldest revision instead of the latest one

    replace("conanfile.py", 'self.requires("hello/1.0")',
                            f'self.requires("hello/1.0#{hello_rrev}")')
    run("conan create .")

with chdir("hello"):
    run("conan create .")
    run("conan create .")
    run("conan list hello/1.0:*#*")
