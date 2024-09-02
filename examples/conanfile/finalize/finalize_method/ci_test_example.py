from test.examples_tools import run
import re

print("- Finalize test -")

output = run("conan create .")
assert re.search("whoisconan/1.0: Finalized folder (.*)(/f)", output)
