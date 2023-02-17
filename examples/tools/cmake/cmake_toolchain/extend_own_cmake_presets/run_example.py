import os
import platform
import textwrap

from test.examples_tools import run, replace

from conan import conan_version

print("- CMakeToolchain: Extending your CMakePresets with Conan generated ones -")

run("conan new -d name=foo -d version=1.0 cmake_exe")

replace("conanfile.py", "tc.generate()", "tc.user_presets_path = 'ConanPresets.json'\n        tc.generate()")

cmake_presets = textwrap.dedent("""
    {
    "version": 4,
    "include": ["./ConanPresets.json"],
    "configurePresets": [
        {
            "name": "default",
            "displayName": "multi config",
            "inherits": "conan-default"
        },
        {
            "name": "release",
            "displayName": "release single config",
            "inherits": "conan-release"
        },
        {
            "name": "debug",
            "displayName": "debug single config",
            "inherits": "conan-debug"
        }
    ],
    "buildPresets": [
        {
            "name": "multi-release",
            "configurePreset": "default",
            "configuration": "Release",
            "inherits": "conan-release"
        },
        {
            "name": "multi-debug",
            "configurePreset": "default",
            "configuration": "Debug",
            "inherits": "conan-debug"
        },
        {
            "name": "release",
            "configurePreset": "release",
            "configuration": "Release",
            "inherits": "conan-release"
        },
        {
            "name": "debug",
            "configurePreset": "debug",
            "configuration": "Debug",
            "inherits": "conan-debug"
        }
    ]
    }""")

with open("CMakePresets.json", "w") as f:
    f.write(cmake_presets)


run("conan install .")
run("conan install . -s build_type=Debug")

if platform.system() == "Windows":
    run(f"cmake --preset default")
    run(f"cmake --build --preset release")
    run(f"cmake --build --preset debug")
else:
    run(f"cmake --preset release")
    run(f"cmake --build --preset release")
    run(f"cmake --preset debug")
    run(f"cmake --build --preset debug")

output = run(str(os.path.join("build", "Release", "foo")))
assert "Hello World Release!" in output

output = run(str(os.path.join("build", "Debug", "foo")))
assert "Hello World Debug!" in output
