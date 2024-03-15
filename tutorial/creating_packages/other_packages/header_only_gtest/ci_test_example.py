import platform

from test.examples_tools import run

cmd_out = run("conan create . -s compiler.cppstd=14 --build missing")
assert "[  PASSED  ] 1 test." in cmd_out
assert "sum/0.1: Package 'da39a3ee5e6b4b0d3255bfef95601890afd80709' built"

cmd_out = run("conan create . -s compiler.cppstd=17")
assert "[  PASSED  ] 1 test." in cmd_out
assert "sum/0.1: Package 'da39a3ee5e6b4b0d3255bfef95601890afd80709' built"

