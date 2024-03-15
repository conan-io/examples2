import platform
import shutil


from test.examples_tools import run, chdir



print("- Declaring the layout when we have multiple subprojects -")

run("conan install hello")
cmd_out = run("conan build hello")
assert "hello WORLD" in cmd_out

run("conan install bye")
cmd_out = run("conan build bye")
assert "bye WORLD" in cmd_out

cmd_out = run("conan create hello")
assert "hello WORLD" in cmd_out

cmd_out = run("conan create bye")
assert "bye WORLD" in cmd_out
