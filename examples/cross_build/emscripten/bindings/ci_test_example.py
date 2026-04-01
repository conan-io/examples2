import os
from test.examples_tools import run

run('conan config install https://github.com/perseoGI/conan-toolchains.git -sf conan_config --args "-b pgi/new/emsdk"')
run("conan build . --build=missing --profile:host emsdk/wasm32")

assert os.path.exists(os.path.join("build", "release-wasm", "wasm_example.html"))
assert os.path.exists(os.path.join("build", "release-wasm", "wasm_example.js"))
assert os.path.exists(os.path.join("build", "release-wasm", "wasm_example.wasm"))
