import os
import shutil
import subprocess

from conan import conan_version
from test.examples_tools import run

if conan_version >= "2.26.0":
    current_dir = os.path.abspath(os.path.dirname(__file__))
    provider_folder = os.path.join(current_dir, "my-organization")
    os.makedirs(provider_folder, exist_ok=True)
    os.chdir(provider_folder)

    run("cosign generate-key-pair --output-key-prefix signing")

    os.chdir("..")

    run(f"conan config install {current_dir} -t dir --target-folder extensions/plugins/sign")

    run("conan new cmake_lib -d name=hello -d version=1.0")
    run("conan create")

    output = run("conan cache sign hello/1.0")
    assert "Package signed for reference hello/1.0" in output
    assert "[Package sign] Summary: OK=2, FAILED=0" in output
    output = run("conan cache verify hello/1.0")
    assert "Package verified for reference hello/1.0" in output
    assert "[Package sign] Summary: OK=2, FAILED=0" in output

    conan_home = run("conan config home").strip()
    shutil.rmtree(os.path.join(conan_home, "extensions", "plugins", "sign"))
