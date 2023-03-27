@echo ON

echo "- AutotoolsToolchain: The toolchain generator for Autotools -"

conan install . --build=missing
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

conanbuild.bat
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

aclocal
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

automake --add-missing
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

autoconf
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

./configure
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

make
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

output=$(./string_formatter)

if [[ "$output" != 'Conan - The C++ Package Manager!' ]]; then
    echo "ERROR: The String Formatter output does not match with the expected value: 'Conan - The C++ Package Manager!'"
    exit 1
fi

echo "AutotoolsToolchain example has been executed with SUCCESS!"
exit /b 0

:ERROR_EXIT
    echo "ERROR: Could not execute the example with success."
    exit /b %ERRORLEVEL%