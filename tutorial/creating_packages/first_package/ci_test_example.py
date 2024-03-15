from test.examples_tools import run, tmp_dir

print("- Create your first Conan package. -")

with tmp_dir("tmp"):
    run("conan new cmake_lib -d name=hello -d version=1.0")
    run("conan create .")
    run("conan create . -s build_type=Debug")
    run("conan create . -o hello/1.0:shared=True")

    output = run("conan list hello")

    # check the package recipe is in the cache
    assert "hello/1.0" in output

    output = run("conan list hello/1.0#:*")

    # assert that there are 3 binaries in the cache
    assert output.count("arch: ") == 3
