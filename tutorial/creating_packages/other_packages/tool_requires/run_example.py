import re

from test.examples_tools import run, chdir

with chdir("tool"):
    run('conan create .')

with chdir("consumer"):
    cmd_out = run('conan create .')
    assert re.search("Security Scanner: The path '.*' is secure!", cmd_out)
    assert "MY_VAR=23" in cmd_out


with chdir("tool"):
    run('conan create conanfile_package_id.py')
    run('conan create conanfile_package_id.py -s build_type=Debug --build missing')
    assert "secure_scanner/1.0: Already installed!" in cmd_out