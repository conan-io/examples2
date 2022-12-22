import os

from test.examples_tools import run, tmp_dir


non_deterministic_conanfile = """\
from datetime import datetime

from conan import ConanFile
from conan.tools.files import copy


class HelloConan(ConanFile):
    name = "{name}"
    version = "1.0"
    {comment}

    def build(self):
        with open("random.txt", "w") as f:
            f.write(str(datetime.now()))

    def package(self):
        # Packaging a random content, the package revision will be different every time
        copy(self, "random.txt", self.source_folder, self.package_folder)
"""


def install_clean_command():
    clean_command = os.path.join(os.path.dirname(os.path.realpath(__file__)), "clean")
    commands_folder = os.path.join("extensions", "commands")
    run(f"conan config install {clean_command} -tf {commands_folder}")


# At first, copy the commands into the ${CONAN_HOME}/extensions/commands folder
install_clean_command()

# 1. Check the custom command is appearing in conan help
output = run("conan -h")
assert "commands\nclean" in output.replace("\r\n", "\n")
# 2. Create several packages
with tmp_dir("clean_hello"):
    # Library (changing PREV each time)
    with open(os.path.join("conanfile.py"), "w") as f:
        f.write(non_deterministic_conanfile.format(name="clean_hello", comment=""))
    run("conan create .")
    run("conan create .")  # different PREV (this is the latest one)

with tmp_dir("clean_other"):
    with open(os.path.join("conanfile.py"), "w") as f:
        f.write(non_deterministic_conanfile.format(name="clean_other", comment=""))
    run("conan create .")
    # Changing RREV
    with open(os.path.join("conanfile.py"), "w") as f:
        f.write(non_deterministic_conanfile.format(name="clean_other", comment="# Changing RREV"))
    run("conan create .")  # different RREV (this is the latest one)

# 3. Run "conan clean" command: Cleaning all the non-latest RREVs (and its packages) and PREVs
output = run("conan clean --force")
assert "Removed package revision: clean_hello/1.0#" in output  # removing earlier PREV from clean_hello
assert "Removed recipe revision: clean_other/1.0#" in output  # removing earlier RREV from clean_other
# Now, it should have removed nothing
output = run("conan clean --force")
assert "Removed recipe revision: clean_other/1.0#" not in output
assert "Removed package revision: clean_hello/1.0#" not in output
