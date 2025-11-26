import os

from test.examples_tools import run

# ############# Example ################
print("- Use the Android NDK to cross-build a package -")


profile = """

[settings]
os=Android
os.api_level=27
arch=armv8
compiler=clang
compiler.version=18
compiler.libcxx=c++_static
compiler.cppstd=17
build_type=Debug

[conf]
tools.android:ndk_path={}
"""

ndk_path = os.environ.get("ANDROID_NDK") or os.environ.get("ANDROID_NDK_HOME")
if ndk_path:
    profile = profile.format(ndk_path)
    os.makedirs(os.path.join("app", "build"), exist_ok=True)
    with open(os.path.join("app", "build", "android"), "w") as fd:
        fd.write(profile)

    run("gradle assembleDebug")
    run("gradle --stop")
    assert os.path.exists(os.path.join("app", "build", "outputs", "apk", "debug", "app-debug.apk"))
