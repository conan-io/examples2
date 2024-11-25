import json
import shutil

from project_setup import DEVELOP, PACKAGES, PRODUCTS
from project_setup import project_setup, run, title, chdir


def replace(filepath, old, new):
    content = open(filepath).read()
    new_content = content.replace(old, new)
    if new_content == content:
        raise Exception("No replacement of '{}' happened".format(old))
    open(filepath, "w").write(new_content)


setup_project = False
package_single = False
package_multi = False
package_multi_lock = False

product_simple = False
product_build_order = False
multi_product_build_order = False
multi_product_build_order_lock = True


print("Checking current home")
out = run("conan config home")
print("Current home is:", out)
run("conan profile detect -f")

if setup_project:
    project_setup()


############### Package pipeline ###################################
# Simulates a change done by one developer to ai.cpp code (a patch/bug fix)
# We do a change in one of the packages in the middle of 
# the graph, bump its version to 1.1.0 and create it

title("Package pipeline", c="*")
print("Doing changes to AI and bumping it to ai/1.1.0 version")
run("git checkout -- ai") # In case ai had previous changes 
replace("ai/src/ai.cpp", "Some Artificial", "SUPER BETTER Artificial")
replace("ai/include/ai.h", "intelligence=0", "intelligence=50")
replace("ai/conanfile.py", 'version = "1.0"', 'version = "1.1.0"')


############### Package pipeline: Single configuration ###################################
if package_single:
    title("Package pipeline, single configuration ")
    out = run('conan create ai --build="missing:ai/*"')
    assert "ai/1.1.0: SUPER BETTER Artificial Intelligence for aliens (Release)!" in out
    assert "ai/1.1.0: Intelligence level=50" in out
    # We don't want to disrupt developers or CI
    run(f'conan remote enable {PRODUCTS}')
    run(f'conan upload "ai*" -r={PRODUCTS} -c')
    run(f'conan remote disable {PRODUCTS}')


def promote(repo_src, repo_dst, pkg_list):
    # Promotion with Artifactory CE (slow, can be improved with art:promote --artifactory-ce)
    out = run(f"conan download --list={pkg_list} -r={repo_src} --format=json", file_stdout="promote.json")
    out = run(f"conan upload --list=promote.json -r={repo_dst} -c")
    print(out)


############### Package pipeline: Multi configuration Release/Debug ###################################
if package_multi:
    title("Package pipeline, multi configuration")
    run('conan remove "*" -c')
    # it could be distributed, both builds in parallel
    with chdir("ai"):
        run('conan create . --build="missing:ai/*" -s build_type=Release --format=json', file_stdout="graph.json")
        run("conan list --graph=graph.json --graph-binaries=build --format=json", file_stdout="built.json")
        run(f'conan remote enable {PACKAGES}')
        run(f"conan upload -l=built.json -r={PACKAGES} -c --format=json", file_stdout="uploaded_release.json")
        run(f'conan remote disable {PACKAGES}')

        run('conan create . --build="missing:ai/*" -s build_type=Debug --format=json', file_stdout="graph.json")
        run("conan list --graph=graph.json --graph-binaries=build --format=json", file_stdout="built.json")
        run(f'conan remote enable {PACKAGES}')
        run(f"conan upload -l=built.json -r={PACKAGES} -c --format=json", file_stdout="uploaded_debug.json")
        run(f'conan remote disable {PACKAGES}')

        print("- Running a promotion -")
        # aggregate the package list
        run("conan pkglist merge -l uploaded_release.json -l uploaded_debug.json --format=json", file_stdout="uploaded.json")
        run(f'conan remote enable {PACKAGES}')
        run(f'conan remote enable {PRODUCTS}')
        promote(PACKAGES, PRODUCTS, "uploaded.json")
        run(f'conan remote disable {PACKAGES}')
        run(f'conan remote disable {PRODUCTS}')


############### Package pipeline: Multi configuration Release/Debug ###################################
if package_multi_lock:
    title("Package pipeline, multi configuration with Lockfiles")
    run('conan remove "*" -c')
    with chdir("ai"):
        # it could be distributed
        run("conan lock create . --lockfile-out=conan.lock")
        run("conan lock create . -s build_type=Debug --lockfile=conan.lock --lockfile-out=conan.lock")  # To make sure we cover all

        run('conan create . --build="missing:ai/*" -s build_type=Release --lockfile=conan.lock --format=json', file_stdout="graph.json")
        run("conan list --graph=graph.json --graph-binaries=build --format=json", file_stdout="built.json")
        run(f'conan remote enable {PACKAGES}')
        run(f"conan upload -l=built.json -r={PACKAGES} -c --format=json", file_stdout="uploaded_release.json")
        run(f'conan remote disable {PACKAGES}')

        out = run('conan create . --build="missing:ai/*" -s build_type=Debug --lockfile=conan.lock --format=json', file_stdout="graph.json")
        run("conan list --graph=graph.json --graph-binaries=build --format=json", file_stdout="built.json")
        run(f'conan remote enable {PACKAGES}')
        run(f"conan upload -l=built.json -r={PACKAGES} -c --format=json", file_stdout="uploaded_debug.json")
        run(f'conan remote disable {PACKAGES}')

        print("- Running a promotion -")
        # aggregate the package list
        run("conan pkglist merge -l uploaded_release.json -l uploaded_debug.json --format=json", file_stdout="uploaded.json")
        run(f'conan remote enable {PACKAGES}')
        run(f'conan remote enable {PRODUCTS}')
        promote(PACKAGES, PRODUCTS, "uploaded.json")
        run(f'conan remote disable {PACKAGES}')
        run(f'conan remote disable {PRODUCTS}')


title("Product pipeline", c="*")

def test_product(product, build_type):
    print(f"---- Testing product {product} with build-type {build_type} ------")
    out = run(f"conan install --requires={product}/1.0 -s build_type={build_type}")
    out = run(product, env_script="conanrun")
    msg = "fun game" if product == "game" else "serving the game"
    assert f"{product}/1.0:{msg} ({build_type})!" in out
    if product == "game":
        assert f"ai/1.1.0: SUPER BETTER Artificial Intelligence for aliens ({build_type})!" in out
        assert "ai/1.1.0: Intelligence level=50" in out
    else:
        assert "ai/1" not in out


if product_simple:
    # Try to see if our main products keep working fine with that change ai/1.1.0
    # lets build the consumers game and mapviewer applications
    # to integrate the ai/1.1.0 changes
    title("Lets see if this change ai/1.1.0 integrates correctly downstream")
    run('conan remove "*" -c')
    run(f'conan remote enable {PRODUCTS}')
    with chdir("build"):
        out = run("conan install --requires=mapviewer/1.0")
        out = run("mapviewer", env_script="conanrun")
        assert "mapviewer/1.0:serving the game (Release)!" in out

        out = run("conan install --requires=game/1.0", error=True)
        assert "ERROR: Missing prebuilt package for 'game/1.0'" in out
        out = run("conan install --requires=game/1.0 --build=missing")
        test_product("game", "Release")


def execute_build_order(build_order_file, lockfile=None, upload=False):
    build_order = (open(build_order_file, "r").read())
    print(build_order)
    build_order = json.loads(build_order)
    to_build = build_order["order"]

    print(f"---- Executing build-order: {build_order_file} -------")

    pkg_lists = []
    for level in to_build:
        for recipe in level:  # This can be executed in parallel
            ref = recipe["ref"]
            # For every ref, multiple binary packages are being built. This can be done in parallel too
            # And often, for different OSs, they will need to be distributed to different build agents
            for packages_level in recipe["packages"]:  # This is also a nested list, packages might have dependencies to each other, but not common
                for package in packages_level:  # This could be executed in parallel too
                    build_args = package["build_args"]
                    filenames = package["filenames"]
                    build_type = "-s build_type=Debug" if any("debug" in f for f in filenames) else ""
                    lockfile_arg = f"--lockfile={lockfile}" if lockfile else ""
                    run(f"conan install {build_args} {build_type} {lockfile_arg} --format=json", file_stdout="graph.json")
                    if upload:
                        run("conan list --graph=graph.json --format=json", file_stdout="built.json")
                        filename = f"uploaded{len(pkg_lists)}.json"
                        run(f"conan upload -l=built.json -r={PRODUCTS} -c --format=json", file_stdout=filename)
                        pkg_lists.append(filename)

    if upload:
        # Merge all received pkg lists from all jobs
        pkg_list_arg = " ".join(f"-l uploaded{i}.json" for i in range(len(pkg_lists)))
        run(f"conan pkglist merge {pkg_list_arg} --format=json", file_stdout="uploaded.json")

if product_build_order:
    title("Introducing a simple build-order to check ai/1.1.0 integration")
    run('conan remove "*" -c')
    run(f'conan remote enable {PRODUCTS}')
    shutil.rmtree("build", ignore_errors=True)
    with chdir("build"):
        run("conan graph build-order --requires=game/1.0 --build=missing --order-by=recipe --reduce --format=json", file_stdout="game_build_order.json")
        execute_build_order("game_build_order.json")
        # We are not uploading yet
        test_product("game", "Release")


if multi_product_build_order:
    title("Using a multi-product multi-configuration build-order")
    run('conan remove "*" -c')
    run(f'conan remote enable {PRODUCTS}')
    shutil.rmtree("build", ignore_errors=True)
    with chdir("build"):
        # products = "game/1.0", "mapviewer/1.0"
        out = run("conan graph build-order --requires=game/1.0 --build=missing --order-by=recipe --format=json", file_stdout="game_release.json")
        out = run("conan graph build-order --requires=game/1.0 --build=missing -s build_type=Debug --order-by=recipe --format=json", file_stdout="game_debug.json")
        out = run("conan graph build-order --requires=mapviewer/1.0 --build=missing --order-by=recipe --format=json", file_stdout="mapviewer_release.json")
        out = run("conan graph build-order --requires=mapviewer/1.0 --build=missing -s build_type=Debug --order-by=recipe --format=json", file_stdout="mapviewer_debug.json")

        out = run("conan graph build-order-merge --file=game_release.json --file=game_debug.json --file=mapviewer_release.json --file=mapviewer_debug.json "
                  "--format=json --reduce", file_stdout="build_order.json")
        execute_build_order("build_order.json")
        # We are not uploading yet
        test_product("game", "Release")
        test_product("game", "Debug")
        test_product("mapviewer", "Release")
        test_product("mapviewer", "Debug")


if multi_product_build_order_lock:
    title("Using a multi-product multi-configuration build-order with lockfiles")
    run('conan remove "*" -c')
    run(f'conan remote enable {PRODUCTS}')
    shutil.rmtree("build", ignore_errors=True)
    with chdir("build"):
        # products = "game/1.0", "mapviewer/1.0"
        run("conan lock create --requires=game/1.0 --lockfile-out=conan.lock")
        run("conan lock create --requires=game/1.0 -s build_type=Debug --lockfile=conan.lock --lockfile-out=conan.lock")  # To make sure we cover all
        run("conan lock create --requires=mapviewer/1.0 --lockfile=conan.lock --lockfile-out=conan.lock")  # To make sure we cover all
        run("conan lock create --requires=mapviewer/1.0 -s build_type=Debug --lockfile=conan.lock --lockfile-out=conan.lock")  # To make sure we cover all

        out = run("conan graph build-order --requires=game/1.0 --lockfile=conan.lock --build=missing --order-by=recipe --format=json", file_stdout="game_release.json")
        out = run("conan graph build-order --requires=game/1.0 --lockfile=conan.lock --build=missing -s build_type=Debug --order-by=recipe --format=json", file_stdout="game_debug.json")
        out = run("conan graph build-order --requires=mapviewer/1.0 --lockfile=conan.lock --build=missing --order-by=recipe --format=json", file_stdout="mapviewer_release.json")
        out = run("conan graph build-order --requires=mapviewer/1.0 --lockfile=conan.lock --build=missing -s build_type=Debug --order-by=recipe --format=json", file_stdout="mapviewer_debug.json")

        out = run("conan graph build-order-merge --file=game_release.json --file=game_debug.json --file=mapviewer_release.json --file=mapviewer_debug.json "
                  "--format=json --reduce", file_stdout="build_order.json")
        execute_build_order("build_order.json", lockfile="conan.lock", upload=True)
        # We are not uploading yet
        test_product("game", "Release")
        test_product("game", "Debug")
        test_product("mapviewer", "Release")
        test_product("mapviewer", "Debug")

        promote(PRODUCTS, DEVELOP, "uploaded.json")
        run('conan remove "*" -c')
        run(f'conan remote disable {PRODUCTS}')
        test_product("game", "Release")
        test_product("game", "Debug")
