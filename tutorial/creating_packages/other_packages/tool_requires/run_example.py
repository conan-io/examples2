import re

from test.examples_tools import run, chdir

with chdir("tool"):
    run('conan create .')

with chdir("consumer"):
    cmd_out = run('conan create .')
    assert re.search("Security Scanner: The path '.*' is secure!", cmd_out)
    assert "MY_VAR=23" in cmd_out


with chdir("tool"):
    cmd_out = run('conan create conanfile_package_id.py')
    assert "Package '82339cc4d6db7990c1830d274cd12e7c91ab18a1' created" in cmd_out
    cmd_out = run('conan create conanfile_package_id.py -s build_type=Debug')
    assert "Package '82339cc4d6db7990c1830d274cd12e7c91ab18a1' created" in cmd_out