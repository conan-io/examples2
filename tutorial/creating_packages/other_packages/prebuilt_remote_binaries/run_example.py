import os

from test.examples_tools import run

run('conan create . -s os="Linux" -s arch="x86_64"')
run('conan create . -s os="Linux" -s arch="armv8"')
run('conan create . -s os="Macos" -s arch="x86_64"')
run('conan create . -s os="Macos" -s arch="armv8"')
run('conan create . -s os="Windows" -s arch="x86_64"')
run('conan create . -s os="Windows" -s arch="armv8"')

cmd_out = run('conan list packages foo/0.1#latest')
assert cmd_out.count("settings:") == 6

