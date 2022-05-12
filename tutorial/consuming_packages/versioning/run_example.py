import platform

from test.examples_tools import chdir, run


run("conan install . --build=missing")
assert "zlib/[~1.2]: zlib/1.2.12" in cmd_out
