import platform
import os


from test.examples_tools import run, chdir, replace


print("- Understanding the package layout -")

# not in editable mode

configure_preset = "default" if platform.system() == "Windows" else "release"

with chdir("say"):
    run("conan create . -s build_type=Release")

with chdir("hello"):
    run("conan install . -s build_type=Release")
    run(f"cmake --preset {configure_preset}")
    run("cmake --build --preset release")
    cmd_out = run("build\Release\hello.exe")
    assert "say/1.0: Hello World Release!" in cmd_out

with chdir("say"):
    replace(os.path.join("src", "say.cpp"), "Hello World", "Bye World")
    run("conan create . -s build_type=Release")

with chdir("hello"):
    run("cmake --build --preset release")
    cmd_out = run("build\Release\hello.exe")
    assert "say/1.0: Bye World Release!" in cmd_out


# use editable mode

run("conan editable add say say/1.0")

with chdir("say"):
    run("conan install . -s build_type=Release")
    run(f"cmake --preset {configure_preset}")
    run("cmake --build --preset release")

with chdir("hello"):
    run("conan install . -s build_type=Release")
    run("cmake --preset release")
    run("cmake --build --preset release")
    cmd_out = run("./build/Release/hello")
    assert "say/1.0: Hello World Release!" in cmd_out

with chdir("say"):
    replace(os.path.join("src", "say.cpp"), "Hello World", "Bye World")
    run("cmake --build --preset release")

with chdir("hello"):
    run("cmake --build --preset release")
    cmd_out = run("./build/Release/hello")
    assert "say/1.0: Bye World Release!" in cmd_out

run("conan editable remove say/1.0")
