import platform
from test.examples_tools import run, replace, chdir, load

try:
    run("conan remove matrix* -c")
    run("conan remove engine* -c")
    run("conan remove sound32* -c")
except:
    pass


run("conan create matrix --version=1.0")
with chdir("engine"):
    run("conan install .")
    run("conan lock create .")
    print(load("conan.lock"))


run("conan create matrix --version=1.1")
with chdir("engine"):
    output = run("conan install .")  # By default it uses the lockfile!
    assert "1.1" not in output
    output = run("conan graph info . --filter=requires")
    assert "1.1" not in output
    output = run("conan create . --version=1.0")
    assert "1.1" not in output


# Section multi-config
run("conan create sound32 --version=1.0")

with chdir("engine"):
    run("conan install . -s arch=x86", error=True)  # fails!
    run("conan install . -s arch=x86 --lockfile-partial")

    # better, more compact alternative and more efficient (no binaries)
    run("conan lock create . -s arch=x86")
    print(load("conan.lock"))


# Evolving lockfiles
with chdir("engine"):
    run("conan lock add --requires=matrix/1.1")  # Must be in the valid range!
    print(load("conan.lock"))
    run("conan install . -s arch=x86 --lockfile-out=conan.lock")
    print(load("conan.lock"))
    run("conan install . -s arch=x86 --lockfile-out=final.lock --lockfile-clean")
    print(load("final.lock"))

    # perfect cleaning
    run("conan lock create . --lockfile-out=64.lock --lockfile-clean")
    run("conan lock create . -s arch=x86 --lockfile-out=32.lock --lockfile-clean")
    print(load("64.lock"))
    print(load("32.lock"))
    run("conan lock merge --lockfile=32.lock --lockfile=64.lock --lockfile-out=conan.lock")
    print(load("conan.lock"))
    
    # Explain that merge exists, and cleaning with multi-config might require some more steps