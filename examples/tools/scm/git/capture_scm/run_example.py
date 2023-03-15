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
    run("git add .")
    run("git commit . -m wip")
    run("conan create .")
