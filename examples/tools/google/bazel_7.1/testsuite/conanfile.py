from conan import ConanFile
from conan.tools.google import Bazel, BazelToolchain, BazelDeps


class BazelDemo(ConanFile):
    name = "bazel_demo"
    version = "0.0.0"
    settings = "os", "arch", "build_type"

    def requirements(self):
        self.requires("gtest/1.14.0")

    def generate(self):
        bazel_deps = BazelDeps(self)
        bazel_deps.generate()

        tc = BazelToolchain(self)
        tc.generate()

    def build(self):
        bazel = Bazel(self)
        bazel.build()
