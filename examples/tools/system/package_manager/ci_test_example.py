from test.examples_tools import run
import re
import os


print("- Packaging system ncurses library using Conan -")

confs = ["tools.system.package_manager:mode=install",
         "tools.system.package_manager:sudo=true",
         "tools.build:verbosity=verbose",
         "tools.compilation:verbosity=verbose"]

out = run("conan create . {}".format(" ".join(["-c " + conf for conf in confs])))

assert "ncurses/system: System requirements"
assert "package(): WARN: No files in this package" in out

print("- Consuming Conan package ncurses/system -")

out = run("conan create consumer/ --version=0.1.0 {}".format(" ".join(["-c " + conf for conf in confs])))

assert "Conan: Target declared 'ncurses::ncurses'" in out
assert "package(): Packaged 1 file: ncurses_version" in out

match = re.search(r"Full package reference:\s*(.+)", out)
assert match
full_package_ref = match.group(1)

package_folder = run(f"conan cache path {full_package_ref}").strip()
assert os.path.isdir(package_folder)
ncurses_version = os.path.join(package_folder, "bin", "ncurses_version")
assert os.path.isfile(ncurses_version)

out = run(ncurses_version)
assert "Conan 2.x Examples - Installed NCurses version" in out