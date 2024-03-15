import platform
import os
import shutil

from conan import conan_version

from test.examples_tools import run, chdir, replace

# TODO: for the moment the layout in the tutorial is only explained
# for the single config case
if platform.system() != "Windows":


    # FIXME: remove once 2.0-beta10 is out
    prefix_preset_name = "" if "beta9" in str(conan_version) else "conan-"
    editable_add_argument = "say/1.0" if "beta9" in str(conan_version) else "--name=say --version=1.0"
    editable_remove_argument = "say/1.0" if "beta9" in str(conan_version) else "--refs=say/1.0"

    print("- Understanding the package layout -")

    # not in editable mode

    configure_preset = "default" if platform.system() == "Windows" else "release"
    executable_binary = "build\Release\hello.exe" if platform.system() == "Windows" else "./build/Release/hello"
    lib_name = "say.lib" if platform.system() == "Windows" else "libsay.a"

    with chdir("say"):
        run("conan create . -s build_type=Release")

    with chdir("hello"):
        run("conan install . -s build_type=Release")
        cmd_out = run(f"cmake --preset {prefix_preset_name}{configure_preset} --log-level=VERBOSE")
        assert f"/p/lib/{lib_name}" in cmd_out
        run(f"cmake --build --preset {prefix_preset_name}release")
        cmd_out = run(executable_binary)
        assert "say/1.0: Hello World Release!" in cmd_out


    # use editable mode

    run(f"conan editable add say {editable_add_argument}")

    with chdir("say"):
        run("conan install . -s build_type=Release")
        run(f"cmake --preset {prefix_preset_name}{configure_preset}")
        run(f"cmake --build --preset {prefix_preset_name}release")

    with chdir("hello"):
        shutil.rmtree("./build")
        run("conan install . -s build_type=Release")
        cmd_out = run(f"cmake --preset {prefix_preset_name}{configure_preset} --log-level=VERBOSE")
        assert f"say/build/Release/{lib_name}" in cmd_out
        run(f"cmake --build --preset {prefix_preset_name}release")
        cmd_out = run(executable_binary)
        assert "say/1.0: Hello World Release!" in cmd_out

    run(f"conan editable remove {editable_remove_argument}")
