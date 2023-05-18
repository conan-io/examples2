from conan import conan_version
from test.examples_tools import run


if conan_version >= "2.0.5":
    run("conan create gcc -o myoption=1")
    run("conan create gcc -o myoption=2")

    output = run("conan create wine")
    assert "MYGCC=1!!" in output
    assert "MYGCC=2!!" in output
