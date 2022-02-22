import os
import subprocess

def run(cmd, error=False):
    # Used by tools/scm check_repo only (see if repo ok with status)
    print("Running: {}".format(cmd))
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = process.communicate()
    out = out.decode("utf-8")
    err = err.decode("utf-8")
    ret = process.returncode

    output = err + out
    if ret != 0 and not error:
        raise Exception("Failed cmd: {}\n{}".format(cmd, output))
    if ret == 0 and error:
        raise Exception("Cmd succeded (failure expected): {}\n{}".format(cmd, output))
    return output


# This is a half diamond
#                        game1 -> engine -> math/1.0
#                          \--------------> math/2.0 (conflict)
# solved with force=True
run("conan create math --version=1.0")

run("conan create math --version=2.0")
run("conan create engine")
out = run("conan install game1", error=True)
assert "ERROR: Version conflict: engine/1.0->math/1.0, game1/1.0->math/2.0" in out

# Add the requires "force=True" fixes it
content = open("game1/conanfile.py").read()
new_content = content.replace('self.requires("math/2.0")',
                              'self.requires("math/2.0", force=True)')
open("game1/conanfile.py", "w").write(new_content)
# The jump in major version requires building  a new engine/1.0 binary
out = run("conan install game1", error=True)   # binary missing
assert "ERROR: Missing binary: engine/1.0" in out
run("conan install game1 --build=missing")

# restore the original contents:
with open("game1/conanfile.py", "w") as f:
    print(content)
    f.write(content)