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


# This is a full diamond
#                        game -> engine -> math/1.0
#                          \----> ai -----> math/2.0 (conflict)
# solved with override=True

# Demo the conflict
run("conan remove * -f")  # Make sure no packages from last run
run("conan create math --version=1.0")
run("conan create math --version=2.0")
run("conan create engine")
run("conan create ai")
out = run("conan install game", error=True)
# NOTE This output shows the downstream conflict not the immediate
assert "ERROR: Version conflict: ai/1.0->math/2.0, game/1.0->math/1.0" in out

# Add the requires "force=True" fixes it
content = open("game/conanfile.py").read()
new_content = content + '        self.requires("math/2.0", override=True)\n'
open("game/conanfile.py", "w").write(new_content)
# The jump in major version requires building  a new engine/1.0 binary
out = run("conan install game", error=True)   # binary missing
assert "ERROR: Missing binary: engine/1.0" in out

# With force=True and --build=missing, it works
run("conan install game --build=missing")

# restore the original contents:
open("game/conanfile.py", "w").write(content)
