#!/bin/bash

set -e
set -x

echo "Setup settings user"
cp -f settings_user.yml $(conan config home)

echo "Conan Examples 2: Compiler Sanitizers - Index Out of Bounds"

pushd index_out_of_bounds/
conan build . -pr ../profiles/clang_asan -c tools.compilation:verbosity=verbose
build/Debug/index_out_of_bounds || true
popd

echo "Conan Examples 2: Compiler Sanitizers - Signed Integer Overflow"

pushd signed_integer_overflow/
conan build . -pr ../profiles/clang_asan_ubsan -c tools.compilation:verbosity=verbose
build/Debug/signed_integer_overflow || true
popd
