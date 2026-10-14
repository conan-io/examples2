from conan import conan_version
from test.examples_tools import run

run("conan create myfunctions")
run("conan build consumer")
