from conan.tools.files import copy
import os


def deploy(graph, output_folder):
    for name, dep in graph.root.conanfile.dependencies.items():
        copy(dep, "*", dep.folders.source_folder, os.path.join(output_folder, "dependency_sources", str(dep)))
