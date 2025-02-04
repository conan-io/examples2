from test.examples_tools import run
import sys


print("- Packaging system ncurses library using Conan -")

if sys.platform != "linux":
    print("INFO: Skipping test, only for Linux due to system requirements.")
    sys.exit(0)

confs = ["tools.system.package_manager:mode=install",
         "tools.system.package_manager:sudo=true",
         "tools.build:verbosity=verbose",
         "tools.compilation:verbosity=verbose"]

out = run("conan create . {}".format(" ".join(["-c " + conf for conf in confs])))

assert "ncurses/system: System requirements"
assert "package(): WARN: No files in this package" in out

print("- Consuming Conan package ncurses/system -")

out = run("conan build consumer/ --name=ncurses-version --version=0.1.0 {}".format(" ".join(["-c " + conf for conf in confs])))

assert "The example application has been successfully built" in out
