import os
import subprocess
import shutil
from contextlib import contextmanager


@contextmanager
def chdir(dir_path):
    current = os.getcwd()
    os.makedirs(dir_path, exist_ok=True)
    os.chdir(dir_path)
    try:
        yield
    finally:
        os.chdir(current)


def replace(file_path, text, replace):
    with open(file_path, "r") as f:
        content = f.read()
    content2 = content.replace(text, replace)
    assert content != content2
    with open(file_path, "w") as f:
        f.write(content2)


def load(file_path):
    with open(file_path, "r") as f:
        content = f.read()
    return content

@contextmanager
def tmp_dir(newdir):
    os.makedirs(newdir)
    try:
        with chdir(newdir):
            yield
    finally:
        shutil.rmtree(newdir)


def run(cmd, error=False):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True, universal_newlines=True)
    
    output = ""
    print("Running: {}".format(cmd))
    print("----- OUTPUT -------")
    for line in iter(process.stdout.readline, ""):
        print(line, end="")
        output += line
    print("----END OUTPUT------")
    
    ret = process.returncode
    process.stdout.close()
    process.wait()

    if ret != 0 and not error:
        raise Exception("Failed cmd: {}\n{}".format(cmd, output))
    if ret == 0 and error:
        raise Exception(
            "Cmd succeeded (failure expected): {}\n{}".format(cmd, output))
    return output


def replace(file_path, text, replace):
    with open(file_path, "r") as f:
        content = f.read()
    content2 = content.replace(text, replace)
    assert content != content2
    with open(file_path, "w") as f:
        f.write(content2)


def load(file_path):
    with open(file_path, "r") as f:
        content = f.read()
    return content
