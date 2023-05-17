import os

from test.examples_tools import run, tmp_dir


run("conan create gcc -o myoption=1")
run("conan create gcc -o myoption=2")

output = run("conan create wine")
assert "MYGCC=1!!" in output
assert "MYGCC=2!!" in output
