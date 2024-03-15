import os
import platform

from test.examples_tools import run, tmp_dir

# ############# Example ################
print("\n- Use the MesonToolchain and PkgConfigDeps generators -\n")

if platform.system() in ("Windows", "Darwin"):
    print(f"SKIPPED TEST BECAUSE MESON IS NOT INSTALLED IN WINDOWS AND MACOS YET.")
    exit(0)

output = run("meson -v")

with tmp_dir(os.path.join(os.path.dirname(__file__), "tmp")):
    if platform.system() == "Linux":
        print("\n- Conan installing all the files into the build folder -\n")
        run("conan install .. --output-folder=build --build=missing")
        print("\n- Setting up all the Meson variables declared by the conan_meson_native.ini file -\n")
        run("meson setup --native-file build/conan_meson_native.ini .. ./src")
        print("\n- Compiling the application -\n")
        run("meson compile -C ./src")
        print("\n- Running the application 'compressor' created -\n")
        run("./src/compressor")
    else:
        # TODO: Add meson to Windows machine
        pass
