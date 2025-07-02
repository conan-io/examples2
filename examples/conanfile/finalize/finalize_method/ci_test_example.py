from test.examples_tools import run
import re

print("- Finalize test -")

output = run("conan create .")
assert re.search("whoisconan/1.0: Finalized folder (.*)(f)", output)

cache_integrity = run("conan cache check-integrity whoisconan/1.0")
assert "Integrity checked: ok" in cache_integrity or "Integrity check: ok" in cache_integrity
