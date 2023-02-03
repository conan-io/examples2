import platform
import os


from test.examples_tools import run, chdir, replace


print("- Editable packages -")

run("conan editable add say say/1.0")

with chdir("say"):
    if platform.system() == "Windows":
        run("conan install . -s build_type=Release")
        run("conan install . -s build_type=Debug")
        run("cmake --preset default")
        run("cmake --build --preset release")
        run("cmake --build --preset debug")
    else:
        run("conan install . -s build_type=Release")
        run("cmake --preset release")
        run("cmake --build --preset release")

with chdir("hello"):
    if platform.system() == "Windows":
        run("conan install . -s build_type=Release")
        run("conan install . -s build_type=Debug")
        run("cmake --preset default")
        run("cmake --build --preset release")
        run("cmake --build --preset debug")
        cmd_out = run("build\Release\hello.exe")
        assert "say/1.0: Hello World Release!" in cmd_out
        cmd_out = run("build\Debug\hello.exe")
        assert "say/1.0: Hello World Debug!" in cmd_out
    else:
        run("conan install . -s build_type=Release")
        run("cmake --preset release")
        run("cmake --build --preset release")
        cmd_out = run("./build/Release/hello")
        assert "say/1.0: Hello World Release!" in cmd_out

with chdir("say"):
    replace(os.path.join("src", "say.cpp"), "Hello World", "Bye World")
    if platform.system() == "Windows":        
        run("cmake --build --preset release")
        run("cmake --build --preset debug")
    else:
        run("cmake --build --preset release")

with chdir("hello"):
    if platform.system() == "Windows":        
        run("cmake --build --preset release")
        run("cmake --build --preset debug")
        cmd_out = run("build\Release\hello.exe")
        assert "say/1.0: Bye World Release!" in cmd_out
        cmd_out = run("build\Debug\hello.exe")
        assert "say/1.0: Bye World Debug!" in cmd_out
    else:
        run("cmake --build --preset release")
        cmd_out = run("./build/Release/hello")
        assert "say/1.0: Bye World Release!" in cmd_out

run("conan editable remove say/1.0")
