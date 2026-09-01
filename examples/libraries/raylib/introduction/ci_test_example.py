import platform
from test.examples_tools import run
import os

print("raylib example")

run("conan install . --build=missing")

if platform.system() == "Windows":
    run("cmake --preset conan-default")
    run("cmake --build --preset conan-release")
else:
    run("cmake --preset conan-release")
    run("cmake --build --preset conan-release")


# WASM tests
run("conan build . --build=missing --profile:host ../../../cross_build/emscripten/profiles/wasm32")

assert os.path.exists(os.path.join("build", "release-wasm", "runner_game.html"))
assert os.path.exists(os.path.join("build", "release-wasm", "runner_game.js"))
assert os.path.exists(os.path.join("build", "release-wasm", "runner_game.wasm"))
