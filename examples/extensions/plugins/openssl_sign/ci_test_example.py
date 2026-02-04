import os

from test.examples_tools import run, tmp_dir

run("git clone https://github.com/conan-io/examples2.git")
provider_folder = os.path.join("examples2", "examples", "extensions", "plugins", "openssl_sign", "your-organization")
os.makedirs(provider_folder)
run(f"openssl genpkey -algorithm RSA -out {provider_folder}/private_key.pem -pkeyopt rsa_keygen_bits:2048")
run(f"openssl pkey -in {provider_folder}/private_key.pem -pubout -out {provider_folder}/public_key.pem")

run("conan config install examples2/examples -t dir --source-folder extensions/plugins/openssl_sign --target-folder extensions/plugins/sign")

run("conan new cmake_lib -d name=hello -d version=1.0")
run("conan create")

output = run("conan cache sign hello/1.0")
assert "kk" in output
output = run("conan cache verify hello/1.0")
assert "kk" in output
