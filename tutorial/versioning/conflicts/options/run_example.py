import platform
from test.examples_tools import run, replace, chdir, load

try:
    run("conan remove matrix* -c")
    run("conan remove engine* -c")
    run("conan remove game* -c")
    run("conan remove intro* -c")
except:
    pass

run("conan create matrix")
run("conan create matrix -o matrix*:shared=True")
run("conan create engine")
run("conan create intro")
run("conan install game", error=True)

# fix it:
replace("game/conanfile.py", 'version = "1.0"',
                             'version = "1.0"\n'
                             '    default_options = {"matrix*:shared": True}')
run("conan install game")
