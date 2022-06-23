import platform

from test.examples_tools import run

cmd_out = run('conan create .')
if platform.system() != "Windows":
    assert "Packaged 1 '.sh' file: say_hello.sh" in cmd_out
else:
    assert "Packaged 1 '.bat' file: say_hello.bat" in cmd_out

cmd_out = run('conan create consumer.py')

assert "Hello!!!" in cmd_out
assert "MYVAR=23" in cmd_out
