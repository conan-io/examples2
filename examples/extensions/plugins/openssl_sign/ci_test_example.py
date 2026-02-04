import os

from test.examples_tools import run


current_dir = os.path.abspath(os.path.dirname(__file__))
provider_folder = os.path.join(current_dir, "your-organization")
os.makedirs(provider_folder)
run(f"openssl genpkey -algorithm RSA -out {provider_folder}/private_key.pem -pkeyopt rsa_keygen_bits:2048")
run(f"openssl pkey -in {provider_folder}/private_key.pem -pubout -out {provider_folder}/public_key.pem")

run(f"conan config install {current_dir} -t dir --target-folder extensions/plugins/sign")

run("conan new cmake_lib -d name=hello -d version=1.0")
run("conan create")

output = run("conan cache sign hello/1.0")
assert "Package signed for reference hello/1.0" in output
assert "[Package sign] Summary: OK=2, FAILED=0" in output
output = run("conan cache verify hello/1.0")
assert "Package verified for reference hello/1.0" in output
assert "[Package sign] Summary: OK=2, FAILED=0" in output
