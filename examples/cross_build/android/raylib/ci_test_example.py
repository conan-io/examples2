import os

from test.examples_tools import run

# ############# Example ################
print("- Use the Android NDK to cross-build a package -")


ndk_path = os.environ.get("ANDROID_NDK")
if ndk_path:
    print(f"Using Android NDK at: {ndk_path}")
    run("gradle --no-daemon assembleDebug")
    assert os.path.exists(os.path.join("app", "build", "outputs", "apk", "debug", "app-debug.apk"))
else:
    print("WARNING: Skipping Android example, ANDROID_NDK environment variable not set")
