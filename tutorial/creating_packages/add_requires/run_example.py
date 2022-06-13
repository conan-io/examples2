from test.examples_tools import run, tmp_dir

print("- Handle external sources -")


print("- Download sources from zip file -")

run("conan remove 'hello*' -f ")

run("conan create .")
