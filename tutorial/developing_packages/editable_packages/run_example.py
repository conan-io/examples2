import platform
import os

from conan import conan_version

from test.examples_tools import run, chdir, replace


print("- Editable packages -")

run("conan editable add say say/1.0")

# FIXME: remove once 2.0-beta10 is out
prefix_preset_name = "" if "beta9" in str(conan_version) else "conan-"

with chdir("say"):
    if platform.system() == "Windows":
        run("conan install . -s build_type=Release")
        run("conan install . -s build_type=Debug")
        run(f"cmake --preset {prefix_preset_name}default")
        run(f"cmake --build --preset {prefix_preset_name}release")
        run(f"cmake --build --preset {prefix_preset_name}debug")
    else:
        run("conan install . -s build_type=Release")
        run(f"cmake --preset {prefix_preset_name}release")
        run(f"cmake --build --preset {prefix_preset_name}release")

with chdir("hello"):
    if platform.system() == "Windows":
        run("conan install . -s build_type=Release")
        run("conan install . -s build_type=Debug")
        run(f"cmake --preset {prefix_preset_name}default")
        run(f"cmake --build --preset {prefix_preset_name}release")
        run(f"cmake --build --preset {prefix_preset_name}debug")
        cmd_out = run("build\Release\hello.exe")
        assert "say/1.0: Hello World Release!" in cmd_out
        cmd_out = run("build\Debug\hello.exe")
        assert "say/1.0: Hello World Debug!" in cmd_out
    else:
        run("conan install . -s build_type=Release")
        run(f"cmake --preset {prefix_preset_name}release")
        run(f"cmake --build --preset {prefix_preset_name}release")
        cmd_out = run("./build/Release/hello")
        assert "say/1.0: Hello World Release!" in cmd_out

with chdir("say"):
    replace(os.path.join("src", "say.cpp"), "Hello World", "Bye World")
    if platform.system() == "Windows":        
        run(f"cmake --build --preset {prefix_preset_name}release")
        run(f"cmake --build --preset {prefix_preset_name}debug")
    else:
        run("cmake --build --preset {prefix_preset_name}release")

with chdir("hello"):
    if platform.system() == "Windows":        
        run(f"cmake --build --preset {prefix_preset_name}release")
        run(f"cmake --build --preset {prefix_preset_name}debug")
        cmd_out = run("build\Release\hello.exe")
        assert "say/1.0: Bye World Release!" in cmd_out
        cmd_out = run("build\Debug\hello.exe")
        assert "say/1.0: Bye World Debug!" in cmd_out
    else:
        run(f"cmake --build --preset {prefix_preset_name}release")
        cmd_out = run("./build/Release/hello")
        assert "say/1.0: Bye World Release!" in cmd_out

run("conan editable remove say/1.0")
