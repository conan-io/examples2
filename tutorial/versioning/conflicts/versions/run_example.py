import platform
from test.examples_tools import run, replace, chdir, load

try:
    run("conan remove matrix* -c")
    run("conan remove engine* -c")
    run("conan remove game* -c")
    run("conan remove intro* -c")
except:
    pass


run("conan create matrix --version=1.0")
run("conan create matrix --version=1.1")
run("conan create engine --version=1.0")
run("conan create intro --version=1.0")
run("conan install game", error=True)


# Fix it:
replace("game/conanfile.py", 'self.requires("intro/1.0")',
                             'self.requires("intro/1.0")\n'
                             '        self.requires("matrix/1.1", override=True)')

run("conan install game")


# force
run("conan create matrix --version=1.2")
replace("game/conanfile.py", 'self.requires("matrix/1.1", override=True)',
                             'self.requires("matrix/1.2")')
run("conan install game", error=True)
replace("game/conanfile.py", 'self.requires("matrix/1.2")',
                             'self.requires("matrix/1.2", force=True)')
run("conan install game")
