@echo ON

set PROFILE_ARGS=-pr profiles/msys
set MSYSTEM=MINGW64
set MSYS2_PATH_TYPE=inherit
set CHERE_INVOKING=1

bash run_example.sh
if %ERRORLEVEL% neq 0 goto ERROR_EXIT

exit /b 0


:ERROR_EXIT
    echo "ERROR: Could not execute the example with success."
    exit /b %ERRORLEVEL%