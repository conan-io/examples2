import platform
import shutil
import os


from test.examples_tools import run, chdir, replace


print("- Using editables with components -")

run("conan create greetings")
cmd_out = run("conan build app")
assert cmd_out.count("hello: Release!") == 2
assert cmd_out.count("bye: Release!") == 1

run("conan remove 'greetings*' -c")
run("conan editable add greetings")
run("conan build greetings")
cmd_out = run("conan build app")
assert cmd_out.count("hello: Release!") == 2
assert cmd_out.count("bye: Release!") == 1

# Do a modification to one component, to verify it is used correctly
replace(os.path.join("greetings", "src", "bye.cpp"), "bye:", "adios:")
run("conan build greetings")
cmd_out = run("conan build app")
assert cmd_out.count("hello: Release!") == 2
assert cmd_out.count("adios: Release!") == 1
