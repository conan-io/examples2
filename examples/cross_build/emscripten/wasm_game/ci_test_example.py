import os
from test.examples_tools import run

run("conan build . --build=missing --profile:host ../profiles/wasm32")

assert os.path.exists(os.path.join("build", "release-wasm", "wasm_game.html"))
assert os.path.exists(os.path.join("build", "release-wasm", "wasm_game.js"))
assert os.path.exists(os.path.join("build", "release-wasm", "wasm_game.wasm"))
