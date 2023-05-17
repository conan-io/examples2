from test.examples_tools import run

output = run("conan create libfoo --build missing")
assert "Protobuf HOST/BUILD versions: 3.18.1/3.18.1" in output
output = run("conan create consumer --build missing")
assert "Protobuf HOST/BUILD versions: 3.21.9/3.21.9" in output
assert "Consumer(): created a person with id 1337"
# Removing libfoo and consumer
run("conan remove libfoo -c")
run("conan remove consumer -c")
