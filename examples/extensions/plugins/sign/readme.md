
## Package signing plugin example with Openssl

To run the package signing example, make sure you are using Conan with the changes in this branch:

    - https://github.com/danimtb/conan/tree/feature/improve_pkg-sign

Steps to test the example:

- Copy the ``sign.py`` file to your Conan home at ```CONAN_HOME/extensions/plugins/sign/sign.py```.
- Generate your signing keys (see comment at the top of sign.py) and place them next to the ``sign.py`` file.
- Create a new package to test the sign and verify commands: ``conan new cmake_lib -d name=hello -d version=1.0``.
- Sign the package: ``conan cache sign hello/1.0``.
- Verify the package signature: ```conan cache verify hello/1.0```.
- You can also use the ``conan upload`` command, and the packages should be signed automatically.
- You can also use the ``conan install`` command, and the packages should be verified automatically.
