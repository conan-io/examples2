import os

from conan import ConanFile
from conan.tools.build import can_run
from conan.tools.layout import basic_layout
from conan.tools.files import copy


class PoseEstimationTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "VirtualRunEnv"

    def requirements(self):
        self.requires(self.tested_reference_str)

    def layout(self):
        basic_layout(self)

    def test(self):
        if can_run(self):
            model_path = os.path.join(self.dependencies[self.tested_reference_str].package_folder, 
                                      "res", "lite-model_movenet_singlepose_lightning_tflite_float16_4.tflite")
            # Video by Olia Danilevich from https://www.pexels.com/
            input_video = os.path.join(self.source_folder, "dancing.mov")
            self.run(f"pose-estimation {model_path} {input_video} 0", env="conanrun")
