import os
import shutil

from test.examples_tools import run, tmp_dir

with tmp_dir("sources_tmp"):
    # Move to temp dir
    shutil.copy("../sources/conanfile.py", "./conanfile.py")
    shutil.copy("../sources/sources_deploy.py", "./sources_deploy.py")
    output = run("conan graph info conanfile.py -c tools.build:download_source=True --deployer=sources_deploy")
    # Our requirements should now be in here
    dependency_folders = set(os.listdir("dependencies_sources"))
    assert {"zlib", "mcap", "zstd", "lz4"}.issubset(dependency_folders)
