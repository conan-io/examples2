import os
import shutil

from test.examples_tools import run, tmp_dir

with tmp_dir("sources_tmp"):
    # Move to temp dir
    shutil.copytree("../sources", "./sources")
    output = run("conan graph info sources/conanfile.py -c tools.build:download_source=True --deploy=sources_deploy")
    # Our requirements should now be in here
    assert "zlib" in os.listdir(os.path.join("sources", "dependency_sources"))
