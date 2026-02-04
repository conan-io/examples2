import os
import shutil

from test.examples_tools import run, tmp_dir

run("git clone https://github.com/conan-io/conan-extensions.git")
run("openssl genpkey -algorithm RSA -out conan-extensions/plugins/openssl_sign/your-organization/private_key.pem -pkeyopt rsa_keygen_bits:2048")
run("openssl pkey -in conan-extensions/plugins/openssl_sign/your-organization/private_key.pem -pubout -out conan-extensions/plugins/openssl_sign/your-organization/public_key.pem")

run("conan config install conan-extensions -t dir --source-folder plugins/openssl_sign --target-folder plugins/sign")

run("conan new cmake_lib -d name=hello -d version=1.0")
run("conan create")

output = run("conan cache sign hello/1.0")
assert "kk" in output
output = run("conan cache verify hello/1.0")
assert "kk" in output
