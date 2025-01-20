import json
import os
import subprocess
import platform
from contextlib import contextmanager


# These Conan repos must exist in the server, with these names (local repos)
DEVELOP = "develop"
PACKAGES = "packages"
PRODUCTS = "products"
# TODO: This must be configured by users
SERVER_URL = os.environ.get("ARTIFACTORY_URL", "http://localhost:8081/artifactory/api/conan")
USER = os.environ.get("ARTIFACTORY_USER", "admin")
PASSWORD = os.environ.get("ARTIFACTORY_PASSWORD", "password")


def run(cmd, error=False, env_script=None, file_stdout=None):
    if env_script is not None:
        env = "{}.bat".format(env_script) if platform.system() == "Windows" else ". {}.sh".format(env_script)
        cmd = "{} && {}".format(env, cmd)
    # Used by tools/scm check_repo only (see if repo ok with status)
    print("Running: {}".format(cmd))
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, text=True)
    out, err = process.communicate()
    ret = process.returncode

    if file_stdout:
        open(file_stdout, "w").write(out)

    output = err + out
    if ret != 0 and not error:
        raise Exception("Failed cmd: {}\n{}".format(cmd, output))
    if ret == 0 and error:
        raise Exception("Cmd succeded (failure expected): {}\n{}".format(cmd, output))
    return output


def title(msg, c="-"):
    print("")
    print(c*80)
    print(f"{c} {msg} {c}")
    print(c*80)


@contextmanager
def chdir(newdir):
    old_path = os.getcwd()
    os.makedirs(newdir, exist_ok=True)
    os.chdir(newdir)
    try:
        yield
    finally:
        os.chdir(old_path)


def project_setup():
    title("init_project: Preparation of the dependency graph and cleaning repositories")
    ############### Part 1 ###################################
    # Just create a project with a cool dependency graph, and
    # and 2 consuming applications, everything with version ranges
    print("- Setup the project initial state -")
    run('conan remove "*" -c')  # Make sure no packages from last run
    run('conan remote remove "*"')
    run("conan profile detect -f")
    print("Cleaning server repos contents")
    for repo in (PRODUCTS, DEVELOP, PACKAGES):
        run(f"conan remote add {repo} {SERVER_URL}/{repo}")
        run(f'conan remote login {repo} {USER} -p "{PASSWORD}"')
        run(f'conan remove "*" -c -r={repo}')

    # create initial graph
    for build_type in ("Release", "Debug"):
        print(f"Creating {build_type} binaries")
        run(f"conan create mathlib -s build_type={build_type} -tf=")
        run(f"conan create ai -s build_type={build_type} -tf=")
        run(f"conan create graphics -s build_type={build_type} -tf=")
        run(f"conan create engine -s build_type={build_type} -tf=")
        out = run(f"conan create game -s build_type={build_type}")
        assert f"game/1.0:fun game ({build_type})!" in out
        out = run(f"conan create mapviewer -s build_type={build_type}")
        assert f"mapviewer/1.0:serving the game ({build_type})!" in out

    run(f'conan upload "*" -r={DEVELOP} -c')
    run('conan remove "*" -c')  # Make sure no packages from last run

    run('conan remote disable "*"')
    run(f'conan remote enable {DEVELOP}')
    print(run("conan remote list"))
    print(run('conan list "*"'))


if __name__ == "__main__":
    project_setup()
