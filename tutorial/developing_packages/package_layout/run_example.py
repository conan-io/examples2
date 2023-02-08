import platform
import os


from test.examples_tools import run, chdir, replace


print("- Understanding the package layout -")

# not in editable mode

configure_preset = "default" if platform.system() == "Windows" else "release"
executable_binary = "build\Release\hello.exe" if platform.system() == "Windows" else "./build/Release/hello"
lib_name = "say.lib" if platform.system() == "Windows" else "libsay.a"

with chdir("say"):
    run("conan create . -s build_type=Release")

with chdir("hello"):
    run("conan install . -s build_type=Release")
    cmd_out = run(f"cmake --preset {configure_preset} --log-level=VERBOSE")
    folder = "\\p\\lib\\" if platform.system() == "Windows" else "/p/lib/"
    assert f"{folder}{lib_name}" in cmd_out
    run("cmake --build --preset release")
    cmd_out = run(executable_binary)
    assert "say/1.0: Hello World Release!" in cmd_out


# use editable mode

run("conan editable add say say/1.0")

with chdir("say"):
    run("conan install . -s build_type=Release")
    run(f"cmake --preset {configure_preset}")
    run("cmake --build --preset release")

with chdir("hello"):
    run("conan install . -s build_type=Release")
    cmd_out = run(f"cmake --preset {configure_preset} --log-level=VERBOSE")
    folder = "say\\build\\Release\\" if platform.system() == "Windows" else "say/build/Release/"
    assert f"{folder}{lib_name}" in cmd_out
    run("cmake --build --preset release")
    cmd_out = run(executable_binary)
    assert "say/1.0: Hello World Release!" in cmd_out

run("conan editable remove say/1.0")
