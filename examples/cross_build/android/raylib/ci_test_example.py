import os

from test.examples_tools import run

# ############# Example ################
print("- Use the Android NDK to cross-build a package -")


ndk_path = os.environ.get("ANDROID_NDK") or os.environ.get("ANDROID_NDK_HOME")
if ndk_path:
    run("gradle --no-daemon assembleDebug")
    assert os.path.exists(os.path.join("app", "build", "outputs", "apk", "debug", "app-debug.apk"))
