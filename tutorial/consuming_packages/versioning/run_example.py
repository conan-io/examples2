from test.examples_tools import chdir, run


cmd_out = run('conan remove "zlib/*" -c')
cmd_out = run("conan install . --build=missing")
assert "zlib/[~1.2]: zlib/1.2." in cmd_out
