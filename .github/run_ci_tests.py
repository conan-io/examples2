"""
Script to run CI test examples based on changed files in a PR.

This script identifies which example files to run based on the files
changed in a pull request. It supports filtering examples by affected
directories and excludes certain known-broken examples.

In case running in a non-PR context, all examples are executed.
"""

import os
import subprocess
import platform
from pathlib import Path


def run_command(cmd) -> str:
    """
    Run a shell command and return output.

    In case of error, it raises a CalledProcessError exception.

    :param cmd: str - command to run
    :returns: str - command output
    """
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, check=True)
    return result.stdout.strip()

def find_examples(runner_os) -> list:
    """
    Find all ci_test_example[.py|.bat|.sh] files based on OS.

    :param runner_os: str - the OS of the runner
    :returns: list of example file paths
    """
    examples = []
    patterns = ["*.py"]
    patterns += ["*.bat"] if runner_os == "Windows" else ["*.sh"]

    for pattern in patterns:
        for path in Path(".").rglob(f"ci_test_example{pattern[1:]}"):
            examples.append(str(path))

    return sorted(examples)

def find_affected_directories(base_ref) -> list:
    """
    Find directories affected by PR changes that contain example files.

    :param base_ref: str - the base branch to compare against
    :returns: list of affected directory paths
    """
    # Get changed files
    cmd = f"git diff --name-only --diff-filter=ACMRT origin/{base_ref}...HEAD"
    changed_files = run_command(cmd).split('\n')

    print("Changed files:")
    for file in changed_files:
        if file:
            print(file)

    affected_dirs = set()

    for file_path in changed_files:
        if not file_path:
            continue

        # Get directory of changed file
        dir_path = Path(file_path).parent

        # Walk up from deepest to root and find first dir with example files
        current = Path(".") / dir_path
        selected = None

        while str(current) != "." and not selected:
            # Check if this directory contains any ci_test_example files
            has_example = any(current.glob("**/ci_test_example.*"))
            if has_example:
                selected = str(current)
                break

            # Move one level up
            if current.parent == current:
                break
            current = current.parent

        if selected:
            affected_dirs.add(selected)

    affected_dirs = sorted(affected_dirs)
    print("\nAffected directories (deepest with examples):")
    for d in affected_dirs:
        print(d)

    return affected_dirs

def filter_examples_by_dirs(examples, affected_dirs) -> list:
    """
    Filter examples to only those in affected directories.

    :param examples: list of example file paths
    :param affected_dirs: list of affected directory paths
    :returns: filtered list of example file paths
    """
    filtered = []

    for example in examples:
        example_path = Path(example)
        for affected_dir in affected_dirs:
            affected_path = Path(affected_dir)
            try:
                # Check if example is under affected directory
                example_path.relative_to(affected_path)
                filtered.append(example)
                break
            except ValueError:
                continue

    return sorted(set(filtered))

def filter_exclusions(examples, is_pr) -> list:
    """
    FIXME: Filter out broken examples after GitHub Actions migration

    :param examples: list of example file paths
    :param is_pr: bool - whether this is a pull request
    :returns: filtered list of example file paths
    """
    filtered = []

    for example in examples:
        # FIXME: Filter out tensorflow examples in PRs
        if is_pr and "tensorflow" in example:
            continue

        filtered.append(example)

    return filtered

def run_example(example, workspace, runner_os) -> None:
    """
    Run a single example file.

    :param example: str - path to the example file
    :param workspace: Path - the GitHub workspace directory
    :param runner_os: str - the OS of the runner
    :returns: None
    """
    example_path = Path(example)
    example_dir = example_path.parent
    example_file = example_path.name

    # Use GitHub Actions grouping for better log readability
    print(f"##[group]Running example: {example_dir}")

    # Change to example directory
    os.chdir(workspace / example_dir)

    try:
        if example_file.endswith(".py"):
            subprocess.run(["python", example_file], check=True)
        elif example_file.endswith(".sh"):
            subprocess.run(["bash", example_file], check=True)
        elif example_file.endswith(".bat"):
            if runner_os == "Windows":
                subprocess.run(["cmd", "/c", example_file], check=True)
            else:
                print("Skipping .bat file on non-Windows platform")
    finally:
        os.chdir(workspace)
        print(f"##[endgroup]")

def main():
    """
    The main logic captures environment variables, finds affected directories,
    filters examples, and runs them accordingly.

    In case an example fails, it raises an exception and stops further execution.
    """
    # INFO: Using env vars so we can test locally too
    event_name = os.environ.get("GITHUB_EVENT_NAME", "")
    base_ref = os.environ.get("GITHUB_BASE_REF", "main")
    workspace = Path(os.environ.get("GITHUB_WORKSPACE", "."))
    runner_os = os.environ.get("RUNNER_OS", platform.system())

    is_pr = event_name == "pull_request"

    affected_dirs = []
    if is_pr:
        print("Pull request detected - finding affected directories\n")
        affected_dirs = find_affected_directories(base_ref)

    examples = find_examples(runner_os)

    # INFO: Filter examples only when is a PR.
    if is_pr and affected_dirs:
        examples = filter_examples_by_dirs(examples, affected_dirs)
        print("\nFiltered to affected directories only:")
        for example in examples:
            print(example)

    # FIXME: Filter out some non-working examples in GitHub Actions
    examples = filter_exclusions(examples, is_pr)

    print("\nExamples to run:")
    for example in examples:
        print(example)

    for example in examples:
        if example:
            run_example(example, workspace, runner_os)

if __name__ == "__main__":
    main()
