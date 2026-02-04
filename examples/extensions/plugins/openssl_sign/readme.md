
## Package signing plugin example with Openssl

Steps to test the example:

- Copy the ``sign.py`` file to your Conan home at ```CONAN_HOME/extensions/plugins/sign/sign.py```.
- Generate your signing keys (see comment at the top of the ``sign.py`` file) and place them  next to the ``sign.py`` file,
  inside a folder with the name of your provider (``your-organization`` in the example).
- Generate a new project to test the sign and verify commands: ``conan new cmake_lib -d name=hello -d version=1.0``.
- Create the package: ``conan create``.
- Sign the package: ``conan cache sign hello/1.0``.
- Verify the package signature: ```conan cache verify hello/1.0```.
- You can also use the ``conan install`` command, and the packages should be verified automatically.
