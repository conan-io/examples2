import platform
from test.examples_tools import chdir, run

run("conan build . --build=missing --profile:host ../profiles/wasm32")
run("conan build . --build=missing --profile:host ../profiles/wasm64")

if platform.system() == "Windows":
    with chdir("build"):
        run("generators\\conanbuild.bat && node --version && node release-wasm\\wasm-alloc.js")
        # Needs at least Node.js 24.0.0
        # run("generators\\conanbuild.bat && node --version && node release-wasm64\\wasm-alloc.js")
else: 
    with chdir("build/release-wasm"):
        run(". generators/conanbuild.sh && node --version && node release-wasm/wasm-alloc.js")
        # Needs at least Node.js 24.0.0
        # run(". generators/conanbuild.sh && node --version && node release-wasm64/wasm-alloc.js")

