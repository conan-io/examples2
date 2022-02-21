import os

def run(cmd, error=False):
    ret = os.system(cmd)
    if ret != 0 and not error:
        raise Exception("Failed cmd: {}".format(cmd))
    if ret == 0 and error:
        raise Exception("Cmd succeded (failure expected): {}".format(cmd))


# This is a half diamong game1 -> engine -> math/1.0
#                          \--------------> math/2.0 (conflict)
# solved with force=True
run("conan create math --version=1.0")
run("conan create math --version=2.0")
run("conan create engine")
run("conan install game1", error=True)

# Add the requires forces fixes it
content = open("game1/conanfile.py").read()
new_content = content.replace('self.requires("math/2.0")',
                              'self.requires("math/2.0", force=True)')
open("game1/conanfile.py", "w").write(new_content)
# The jump in major version requires building  a new engine/1.0 binary
run("conan install game1", error=True)   # binary missing
run("conan install game1 --build=missing")


# TODO: Full diamong example, introduce "override" instead of "force"