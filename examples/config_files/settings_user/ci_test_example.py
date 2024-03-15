import os

from test.examples_tools import run, tmp_dir

cur_dir = os.path.dirname(os.path.realpath(__file__))
conanfile = """
from conan import ConanFile


class HelloConan(ConanFile):
    name = "pkg"
    version = "1.0"
    settings = "os", "compiler", "build_type", "arch"
"""
profile = """
[settings]
arch=x86_64
build_type=Release
compiler=apple-clang
compiler.cppstd=gnu98
compiler.libcxx=libc++
compiler.version=12.0
os=Macos
"""


def install_settings_user_yml():
    settings_user_yml = os.path.join(cur_dir, "sources", "settings_user.yml")
    run(f"conan config install {settings_user_yml}")


print("Installing settings_user.yml...")
install_settings_user_yml()


with tmp_dir("tmp"):
    with open("conanfile.py", "w") as f:
        f.write(conanfile)
    with open("profile", "w") as f:
        f.write(profile)

    # Creating package with some Conan default settings
    output = run(f"conan create . -pr profile")
    assert "os=Macos" in output
    # Creating package with new OS
    output = run(f"conan create . -pr profile -s os=webOS -s os.sdk_version=7.0.0")
    assert "os=webOS" in output
    assert "os.sdk_version=7.0.0" in output
    # Creating package with latest gcc version
    output = run(f"conan create . -pr profile -s compiler=gcc -s compiler.version=13.0-rc -s compiler.libcxx=libstdc++11")
    assert "compiler.version=13.0-rc" in output
    # Creating package with new OS and new arch
    output = run(f"conan create . -pr profile -s os=webOS -s arch=cortexa15t2hf")
    assert "os=webOS" in output
    assert "arch=cortexa15t2hf" in output
