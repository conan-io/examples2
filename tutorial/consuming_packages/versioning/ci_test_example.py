from test.examples_tools import chdir, run


cmd_out = run('conan remove "zlib/*" -c')
cmd_out = run("conan install . --build=missing")
assert "zlib/[~1.3]: zlib/1.3." in cmd_out
