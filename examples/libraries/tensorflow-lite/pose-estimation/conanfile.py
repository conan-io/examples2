import os

from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout
from conan.tools.files import copy


class PoseEstimationRecipe(ConanFile):
    name = "pose-estimation"
    version = "1.0"
    package_type = "application"

    # Optional metadata
    license = "MIT"
    description = "Pose estimation example using Tensorflow Lite and OpenCV"

    # Binary configuration
    settings = "os", "compiler", "build_type", "arch"

    # Sources are located in the same place as this recipe, copy them to the recipe
    exports_sources = "CMakeLists.txt", "src/*", "licenses/*", "res/lite-model_movenet_singlepose_lightning_tflite_float16_4.tflite"

    generators = "CMakeDeps", "CMakeToolchain"

    def requirements(self):
        self.requires("tensorflow-lite/2.10.0")
        self.requires("opencv/4.5.5")
        self.requires("libwebp/1.3.0", override=True)
        self.requires("eigen/3.4.0", override=True)

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def package(self):
        copy(self, "licenses/lite-model_movenet_singlepose_lightning_tflite_float16_4_tflite_LICENSE.txt", 
             src=self.source_folder, dst=os.path.join(self.package_folder, "licenses"), keep_path=False)
        copy(self, "res/lite-model_movenet_singlepose_lightning_tflite_float16_4.tflite", 
             src=self.source_folder, dst=os.path.join(self.package_folder, "res"), keep_path=False)
        cmake = CMake(self)
        cmake.install()
