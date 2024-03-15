import os
import platform

from test.examples_tools import run, tmp_dir

# ############# Example ################
print("- Use the Android NDK to cross-build a package -")


profile = """

[settings]
os=Android
os.api_level=21
arch=armv8
compiler=clang
compiler.version=12
compiler.libcxx=c++_static
compiler.cppstd=14
build_type=Debug    

[conf]
tools.android:ndk_path={}
"""

ndk_path = {"Darwin": "/opt/homebrew/share/android-ndk", "Linux": "/opt/android-ndk-r23c"}.get(platform.system())

if ndk_path:
    profile = profile.format(ndk_path)
    with tmp_dir("tmp"):
        with open("android", "w") as _f:
            _f.write(profile)
        run("conan new -d name=foo -d version=1.0 cmake_lib")
        output = run("conan create . --profile ./android")
