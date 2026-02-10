import platform
from test.examples_tools import chdir, run

run('conan config install https://github.com/perseoGI/conan-toolchains.git -sf conan_config --args "-b pgi/new/emsdk"')
run("conan build . --build=missing --profile:host emsdk/wasm32")
run("conan build . --build=missing --profile:host emsdk/wasm64")

if platform.system() == "Windows":
    with chdir("build"):
        run("generators\\conanbuild.bat && node --version && node release-wasm\\wasm-alloc")
        # Needs at least Node.js 24.0.0
        # run("generators\\conanbuild.bat && node --version && node release-wasm64\\wasm-alloc")
else: 
    with chdir("build/release-wasm"):
        run(". generators/conanbuild.sh && node --version && node wasm-alloc")

    # Needs at least Node.js 24.0.0
    # with chdir("build/release-wasm64"):
    #     run(". generators/conanbuild.sh && node --version && node wasm-alloc")

