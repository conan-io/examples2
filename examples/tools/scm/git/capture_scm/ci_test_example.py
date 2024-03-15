import os
import platform
import tempfile
import shutil

from test.examples_tools import run, chdir

# ############# Example ################
print("\n- Capturing the SCM information -\n")

folder = tempfile.mkdtemp(suffix='conans')
folder = os.path.join(folder, "capture_scm")
current = os.path.abspath(os.path.dirname(__file__))
print("Current folder", current)
shutil.copytree(current, folder)


with chdir(folder):
    run("git init .")
    run('git config user.name "Your Name"')
    run('git config user.email "you@example.com"')
    run("git add .")
    run("git commit . -m wip")
    run("conan create .")
    # will not fail with .gitignore
    run("conan create .")
