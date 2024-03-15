from test.examples_tools import run


print("- Injecting cmake variables via a profile -")

out = run("conan create . -pr=./myprofile")

assert "-- MYVAR1 MYVALUE1!!" in out
