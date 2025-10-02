@echo off
setlocal enabledelayedexpansion

echo Setup settings user
for /f "usebackq delims=" %%H in (conan config home) do set "CONAN_HOME=%%H"
copy /Y settings_user.yml "%CONAN_HOME%"

echo Conan Examples 2: Compiler Sanitizers - Index Out of Bounds

conan export index_out_of_bounds/
conan build index_out_of_bounds/ --version=0.1.0 -pr profiles/asan -of index_out_of_bounds/install --build=missing -c tools.compilation:verbosity=verbose
index_out_of_bounds/build/Debug/index_out_of_bounds 2>nul || echo Process completed with errors (expected for sanitizer demo)

echo Conan Examples 2: Compiler Sanitizers - Signed Integer Overflow

conan export signed_integer_overflow/
conan build signed_integer_overflow/ --version=0.1.0 -pr profiles/asan_ubsan -of signed_integer_overflow/install --build=missing -c tools.compilation:verbosity=verbose
signed_integer_overflow/build/Debug/signed_integer_overflow 2>nul || echo Process completed with errors (expected for sanitizer demo)

exit /b 0