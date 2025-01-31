from test.examples_tools import run


print("- Packaging system ncurses library using Conan -")

confs = ["tools.system.package_manager:mode=install",
         "tools.system.package_manager:sudo=true",
         "tools.build:verbosity=verbose",
         "tools.compilation:verbosity=verbose"]

out = run("conan create . {}".format(" ".join(["-c " + conf for conf in confs])))

assert "Found Curses"
assert "Conan 2.x Examples - Installed NCurses version" in out
