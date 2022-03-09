import os
import subprocess
import platform


def run(cmd, error=False):
    # Used by tools/scm check_repo only (see if repo ok with status)
    print("Running: {}".format(cmd))
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
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

def replace(filepath, old, new):
    content = open(filepath).read()
    new_content = content.replace(old, new)
    if new_content == content:
        raise Exception("No replacement of '{}' happened".format(old))
    open(filepath, "w").write(new_content)


############### Part 1 ###################################
print("- Part 1: Setup the project initial state -")
run('conan remove "*" -f')  # Make sure no packages from last run
run("conan create matrix --version=1.0")
run("conan create ai --version=1.0")
run("conan create physx --version=1.0")
run("conan create engine --version=1.0")
out = run("conan create game --version=1.0")
assert "game/1.0:fun game (Release)!" in out
out = run("conan create gameserver --version=1.0")
assert "gameserver/1.0:serving the game (Release)!" in out

############### Part 2 ###################################
print("- Part 2: Lets do a change in ai/1.0, and bump the version to 1.0.1 -")
replace("ai/src/ai.cpp", "Some Artificial", "SUPER BETTER Artificial")
out = run("conan create ai --version=1.0.1")
assert "ai/1.0.1: SUPER BETTER Artificial Intelligence for enemies (Release)!" in out

############### Part 3 ###################################
print("- Part 3: Lets see if this change integrates correctly downstream -")
run("conan install --requires=gameserver/1.0")
out = run("conan install --requires=game/1.0", error=True)
assert "ERROR: Missing prebuilt package for 'game/1.0'" in out
out = run("conan install --requires=game/1.0 --build=missing")
if platform.system() == "Windows":
    out = run("conanrun.bat && game")
else:
    out = run("source conanrun.sh && game")
assert "game/1.0:fun game (Release)!" in out
assert "ai/1.0.1: SUPER BETTER Artificial Intelligence for enemies (Release)!" in out

############### Part 4 ###################################
print("- Part 4: Start using lockfiles -")
run("conan lock create --requires=game/1.0 --lockfile-out=game.lock")
replace("ai/src/ai.cpp", "SUPER BETTER Artificial", "AUTONOMOUSLY EVOLVED Artificial")
out = run("conan create ai --version=1.0.2")
assert "ai/1.0.2: AUTONOMOUSLY EVOLVED Artificial Intelligence for enemies (Release)!" in out
out = run("conan install --requires=game/1.0 --build=missing --lockfile=game.lock")
assert "ai/1.0.1" in out
assert "ai/1.0.2" not in out
out = run("conan install --requires=game/1.0 --build=missing --lockfile=game.lock -s build_type=Debug")
assert "ai/1.0.1" in out
assert "ai/1.0.2" not in out

############### Part 5 ###################################
print("- Part 5: Change a public header, bump minor version -")
replace("ai/include/ai.h", "intelligence=0", "intelligence=50")
out = run("conan create ai --version=1.1.0")
print(out)


############### Part 6 ###################################
print("- Part 6: Lets see if the minor 1.1 integrate downstream -")
run("conan install --requires=gameserver/1.0")  # no changes, all good and ready
out = run("conan install --requires=game/1.0 --build=missing")
if platform.system() == "Windows":
    out = run("conanrun.bat && game")
else:
    out = run("source conanrun.sh && game")
print(out)

