from test.examples_tools import run, tmp_dir

print("-  -")


print("- Declaring the layout when creating packages for third-party libraries -")

run("conan create .")

output = run("conan list hello")

# check the package recipe is in the cache
assert "hello/1.0" in output
