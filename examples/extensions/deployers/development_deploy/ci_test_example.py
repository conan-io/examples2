import os
import shutil
import tempfile
import platform

from test.examples_tools import run, tmp_dir, chdir, load


#TODO: Temporary HOME folder
folder = tempfile.mkdtemp(suffix='conans')
folder = os.path.join(folder, "full_relocatable_deploy")
current = os.path.abspath(os.path.dirname(__file__))
print("Current folder", current)
print("Test folder", folder)
shutil.copytree(current, folder)


with chdir(folder):
    output = run("conan install . --deployer=full_deploy --build=missing")
    output = run("conan install . --deployer=full_deploy -s build_type=Debug --build=missing")

new_folder = tempfile.mkdtemp(suffix='conans')
new_folder = os.path.join(new_folder, "full_relocatable_deploy")

print("Relocating user folder", new_folder)
shutil.move(folder, new_folder)

with chdir(new_folder):
    if platform.system() == "Windows":
        with chdir("build"):
            run("generators\conanbuild.bat && cmake --version")
            run("generators\conanbuild.bat && cmake .. -G \"Visual Studio 17 2022\" -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake")
            run("generators\conanbuild.bat && cmake --build . --config Release")
            cmd_out = run("Release\\compressor.exe")
    else: 
        with chdir("build/Release"):
            # TODO: This is still necessary in Linux
            sed_args = "-i ''" if platform.system() == "Darwin" else "-i"
            # Since Conan 2.34.0 the generators folder also has UTF-16 .ps1 scripts, and macOS sed
            # fails on them with a UTF-8 locale ("illegal byte sequence"). LC_ALL=C avoids it. The
            # .ps1 files don't need the replacement, as they use paths relative to $PSScriptRoot.
            run(f"cd generators && LC_ALL=C sed {sed_args} 's,{folder},{new_folder},g' *")
            run(". generators/conanbuild.sh && cmake --version")
            run(". generators/conanbuild.sh && cmake ../.. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release")
            run(". generators/conanbuild.sh && cmake --build .")

            cmd_out = run("./compressor")
  