#!/bin/bash

set -e
set -x

echo "Setup settings user"
cp -f settings_user.yml $(conan config home)

echo "Conan Examples 2: Compiler Sanitizers - Index Out of Bounds"

cd index_out_of_bounds/
conan build . -pr ../profiles/asan -c tools.compilation:verbosity=verbose
build/Debug/index_out_of_bounds || true

echo "Conan Examples 2: Compiler Sanitizers - Signed Integer Overflow"

cd signed_integer_overflow/
conan build . -pr ../profiles/asan_ubsan -c tools.compilation:verbosity=verbose
build/Debug/signed_integer_overflow || true
