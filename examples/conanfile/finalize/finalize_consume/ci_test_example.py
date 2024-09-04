from test.examples_tools import run
import re

print("- Finalize test dependency/consumer -")

dep_output = run("conan create dependency")

assert re.search("Running finalize method in (.*)f", dep_output)
assert re.search("Running package_info method in (.*)f", dep_output)

consumer_output = run("conan create consumer")

assert "Running generate method" in consumer_output
assert re.search("Dependency package_folder: (.*)f", consumer_output)

package_folder_content = """
consumer/1.0: Content in dependency package_folder:
['file1.txt']
"""  
assert package_folder_content in consumer_output

assert re.search("Dependency immutable_package_folder: (.*)p", consumer_output)

