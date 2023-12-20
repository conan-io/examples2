import os
import subprocess
import shutil
from contextlib import contextmanager
import time


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
    print("Running: {}".format(cmd))
    start_time = time.time()

    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True, text=True)

    output = ''
    
    for line in iter(process.stdout.readline, ''):
        print(line, end='', flush=True)
        output += line

    ret = process.wait()
    end_time = time.time()
    
    elapsed_time = end_time - start_time
    print(f"Elapsed time: {elapsed_time:.2f} seconds")

    if ret != 0 and not error:
        raise Exception(f"Failed cmd: {cmd}\n{output}")
    if ret == 0 and error:
        raise Exception(f"Cmd succeeded (failure expected): {cmd}\n{output}")

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
