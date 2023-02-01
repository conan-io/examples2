import platform
from test.examples_tools import run, replace, chdir, load

run("conan remove matrix* -c")
run("conan remove engine* -c")
run("conan remove sound32* -c")

run("conan create matrix --version=1.0")
with chdir("engine"):
    run("conan install . --lockfile-out=conan.lock")
    print(load("conan.lock"))


run("conan create matrix --version=1.1")
with chdir("engine"):
    output = run("conan install .")  # By default it uses the lockfile!
    assert "1.1" not in output
    output = run("conan graph info . --lockfile=conan.lock --filter=requires")
    assert "1.1" not in output
    output = run("conan create . --version=1.0 --lockfile=conan.lock")
    assert "1.1" not in output


# Section multi-config
run("conan create sound32 --version=1.0")

with chdir("engine"):
    run("conan install . -s arch=x86", error=True)  # fails!
    run("conan install . -s arch=x86 --lockfile-partial")
    run("conan install . -s arch=x86 --lockfile-partial --lockfile-out=sound32.lock")
    print(load("sound32.lock"))

    # better, more compact alternative and more efficient (no binaries)
    run("conan lock create . -s arch=x86")
    print(load("conan.lock"))


# Evolving lockfiles
with chdir("engine"):
    run("conan lock add --requires=matrix/1.1")  # Must be in the valid range!
    print(load("conan.lock"))
    run("conan install . -s arch=x86 --lockfile-out=conan.lock")
    print(load("conan.lock"))
    run("conan install . -s arch=x86 --lockfile-out=conan.lock --lockfile-clean")
    print(load("conan.lock"))
    
    # Explain that merge exists, and cleaning with multi-config might require some more steps