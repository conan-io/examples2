import platform
from test.examples_tools import run

# Only for Windows
if platform.system() == "Windows":
    print("libcurl, stb and fmt example for Visual Studio extension post")

    # not using conanfile in case someone downloads the whole folder
    # we do not want to interfere with the one created by the extension
    run("conan install --requires=libcurl/8.12.1 --requires=stb/cci.20240531 --requires=fmt/11.2.0 --build=missing -g MSBuildDeps -g MSBuildToolchain --output-folder=conan")
    run('.\\conan\\conanvcvars.bat & MSBuild asciiartgen.sln /p:Configuration=Release /p:Platform="x64"')
    run(".\\x64\\Release\\asciiartgen.exe")
