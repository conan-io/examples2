import re

from test.examples_tools import run, chdir

with chdir("tool"):
    run('conan create .')

with chdir("consumer"):
    cmd_out = run('conan create .')
    assert re.search("Security Scanner: The path '.*' is secure!", cmd_out)
    assert "MY_VAR=23" in cmd_out
