from test.examples_tools import run, chdir


print("- Layout example: ConanFile in subfolder -")

with chdir("conan"):
    run("conan create . -s build_type=Release")
