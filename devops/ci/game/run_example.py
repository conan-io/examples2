import json
import os
import subprocess
import platform

DEVELOP_URL = "http://localhost:8081/artifactory/api/conan/develop"
PACKAGES_URL = "http://localhost:8081/artifactory/api/conan/packages"
PRODUCTS_URL = "http://localhost:8081/artifactory/api/conan/products"
PASSWORD = "Patata!12"
init_graph = True


def run(cmd, error=False, env_script=None, file_stdout=None):
    if env_script is not None:
        env = "{}.bat".format(env_script) if platform.system() == "Windows" else ". {}.sh".format(env_script)
        cmd = "{} && {}".format(env, cmd)
    # Used by tools/scm check_repo only (see if repo ok with status)
    print("Running: {}".format(cmd))
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    out, err = process.communicate()
    out = out.decode("utf-8")
    err = err.decode("utf-8")
    ret = process.returncode

    if file_stdout:
        open(file_stdout, "w").write(out)

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

def clean():
    run('conan remove "*" -c')  # Make sure no packages from last run
    run("conan remote remove *")

def add_repo(name, url):
    run(f"conan remote add {name} {url}")
    run(f"conan remote login {name} admin -p {PASSWORD}")

if init_graph:
    print("- Preparation of the dependency graph -")
    ############### Part 1 ###################################
    # Just create a project with a cool dependency graph, and
    # and 2 consuming applications, everything with version ranges
    print("- Part 1: Setup the project initial state -")
    clean()
    for repo in (DEVELOP_URL, PACKAGES_URL, PRODUCTS_URL):
        add_repo("repo", repo)
        run("conan remove * -c -r=repo")
        run("conan remote remove *")

    # create initial graph
    run("conan create mathlib -tf=")
    run("conan create ai -tf=")
    run("conan create graphics -tf=")
    run("conan create engine -tf=")
    out = run("conan create game")
    assert "game/1.0:fun game (Release)!" in out
    out = run("conan create mapviewer")
    assert "mapviewer/1.0:serving the game (Release)!" in out
    add_repo("develop", DEVELOP_URL)
    run("conan upload * -r=develop -c")


print("- Package pipeline -")
############### Package pipeline ###################################
# Simulates a change done by one developer to ai.cpp code (a patch/bug fix)
# We do a change in one of the packages in the middle of 
# the graph, bump its version to 1.0.1 and create it

############### Package pipeline: Single configuration ###################################
print("- Package pipeline, single configuration -")
clean()
add_repo("develop", DEVELOP_URL)
replace("ai/src/ai.cpp", "Some Artificial", "SUPER BETTER Artificial")
replace("ai/conanfile.py", 'version = "1.0"', 'version = "1.0.1"')
out = run("conan create ai --build=missing:ai/*")
assert "ai/1.0.1: SUPER BETTER Artificial Intelligence for aliens (Release)!" in out
# We don't want to disrupt developers or CI
add_repo("products", PRODUCTS_URL)
run("conan upload ai* -r=product -c")

############### Package pipeline: Multi configuration Release/Debug ###################################
print("- Package pipeline, multi configuration -")
clean()
add_repo("develop", DEVELOP_URL)

add_repo("packages", PACKAGES_URL)
# it could be distributed
run("conan create ai --build=missing:ai/* -s build_type=Release")
run("conan upload ai* -r=packages -c --format=json", file_stdout="upload_release.json")
run("conan create ai --build=missing:ai/* -s build_type=Debug")
run("conan upload ai* -r=packages -c --format=json", file_stdout="upload_debug.json")
# aggregate the package list
run("conan pkglist merge -l upload_release.json -l upload_debug.json --format=json", file_stdout="upload.json")

clean()
# We don't want to disrupt developers or CI
add_repo("packages", PACKAGES_URL)
add_repo("products", PRODUCTS_URL)
# Promotion with Artifactory CE (slow, can be improved with art:promote)
run("conan download --list=upload.json -r=packages")
run("conan upload --list=upload.json -r=product -c")

kk


############### Part 3 ###################################
# Try to see if our main products keep working fine with that change ai/1.0.1
# lets build the consumers game and mapviewer applications
# to integrate the ai/1.0.1 changes
print("- Part 3: Lets see if this change ai/1.0.1 integrates correctly downstream -")
run("conan install --requires=mapviewer/1.0")
out = run("conan install --requires=game/1.0", error=True)
assert "ERROR: Missing prebuilt package for 'game/1.0'" in out
run("conan install --requires=game/1.0 --build=missing")
out = run("game", env_script="conanrun")
assert "game/1.0:fun game (Release)!" in out
assert "ai/1.0.1: SUPER BETTER Artificial Intelligence for aliens (Release)!" in out


############### Part 4 ###################################
# If we are building different configurations, like Release
# and Debug, something could change in between in deps.
# Lets introduce a lockfile to avoid this
print("- Part 4: Start using lockfiles -")
run("conan lock create --requires=game/1.0 --lockfile-out=game.lock")
# This change and ai/1.0.2 will not be used, it is after the lock
replace("ai/src/ai.cpp", "SUPER BETTER Artificial", "AUTONOMOUSLY EVOLVED Artificial")
out = run("conan create ai --version=1.0.2")
assert "ai/1.0.2: AUTONOMOUSLY EVOLVED Artificial Intelligence for aliens (Release)!" in out
# applying the lock, still ai/1.0.1 used
out = run("conan install --requires=game/1.0 --build=missing --lockfile=game.lock")
assert "ai/1.0.1" in out
assert "ai/1.0.2" not in out
out = run("game", env_script="conanrun")
assert "ai/1.0.1: SUPER BETTER Artificial Intelligence for aliens (Release)!" in out


############### Part 5 ###################################
# What happens if the change is done in the public headers?
# the minor version should be bumped, and that implies building
# the intermediate dependencies too. Lets start doing the change
print("- Part 5: Change a public header, bump minor version -")
replace("ai/include/ai.h", "intelligence=0", "intelligence=50")
out = run("conan create ai --version=1.1.0")
assert "ai/1.1.0: AUTONOMOUSLY EVOLVED Artificial Intelligence for aliens (Release)!" in out
assert "ai/1.1.0: Intelligence level=50" in out

############### Part 6 ###################################
# Lets see how the change in ai/1.1.0 requires building engine/1.0 too
# as the change is in the public api and engine->ai
print("- Part 6: Lets see if the minor 1.1.0 integrate downstream -")
run("conan install --requires=mapviewer/1.0")  # no changes, all good and ready
out = run("conan install --requires=game/1.0", error=True)
assert "ERROR: Missing prebuilt package for 'game/1.0'" in out
out = run("conan install --requires=game/1.0 --build=game/1.0", error=True)
assert "ERROR: Missing prebuilt package for 'engine/1.0'" in out

############### Part 7 ###################################
# Now that there are some cases that all the intermediate dependencies
# need to be built, we might want to distribute the build in a CI farm
# and for that we need to know what to build, and very importantly in what order
print("- Part 7: Compute the build-order -")
run("conan lock create --requires=game/1.0 --lockfile-out=game.lock")
out = run("conan lock create --requires=game/1.0 -s build_type=Debug --lockfile=game.lock --lockfile-out=game.lock")
assert "ai/1.1.0" in out
run("conan lock create --requires=mapviewer/1.0 --lockfile=game.lock --lockfile-out=game.lock")
out = run("conan lock create --requires=mapviewer/1.0 -s build_type=Debug --lockfile=game.lock --lockfile-out=game.lock")
lock = open("game.lock").read()
print(lock)

out = run("conan graph build-order --requires=game/1.0 --lockfile=game.lock --build=missing --order-by=recipe --format=json", file_stdout="game_bo.json")
out = run("conan graph build-order --requires=game/1.0 --lockfile=game.lock --build=missing -s build_type=Debug --order-by=recipe --format=json", file_stdout="game_bo_debug.json")
out = run("conan graph build-order --requires=mapviewer/1.0 --lockfile=game.lock --build=missing --order-by=recipe --format=json", file_stdout="mapviewer_bo.json")
out = run("conan graph build-order --requires=mapviewer/1.0 --lockfile=game.lock --build=missing -s build_type=Debug --order-by=recipe --format=json", file_stdout="mapviewer_bo_debug.json")

############### Part 8 ###################################
# If we have the build order for several applications, and 
# serveral configurations, there might be overlap. The build-orders
# can be merged in a single one, to optimize the building
print("- Part 8: Aggregate build orders -")
out = run("conan graph build-order-merge --file=game_bo.json --file=game_bo_debug.json "
          "--format=json", file_stdout="bo.json")


############### Part 9 ###################################
# Now that we have the aggregated build-order, lets execute it
# simulating a distributed build
print("- Part 9: Iterate the build-order -")
json_file = open("bo.json").read()
print(json_file)
to_build = json.loads(json_file)
to_build = to_build["order"]

for level in to_build:
    for elem in level:  # This can be executed in parallel
        ref = elem["ref"]
        # For every ref, multiple binary packages are being built. This can be done in parallel too
        # And often, for different OSs, they will need to be distributed to different build agents
        for packages in elem["packages"]:
            for package in packages:
                binary = package["binary"]
                if binary != "Build":
                    continue
                # TODO: The options are not used, they should be passed too
                filenames = package["filenames"]
                # This is the mapping between the build-order filenames and the profiles
                build_type = "Debug" if "debug" in filenames[0] else "Release"
                cmd = "conan install --requires={ref} --build={ref} --lockfile=game.lock -s build_type={bt}".format(ref=ref, bt=build_type)
                run(cmd)

out = run("game", env_script="conanrunenv-release-x86_64")
print(out)
assert "ai/1.1.0: AUTONOMOUSLY EVOLVED Artificial Intelligence for aliens (Release)!" in out
assert "ai/1.1.0: Intelligence level=50" in out
out = run("game", env_script="conanrunenv-debug-x86_64")
print(out)
assert "ai/1.1.0: AUTONOMOUSLY EVOLVED Artificial Intelligence for aliens (Debug)!" in out
assert "ai/1.1.0: Intelligence level=50" in out
