# Writing a Godot GDExtension in C++ with godot-cpp

Example that uses [godot-cpp](https://github.com/godotengine/godot-cpp) and
[flecs](https://github.com/SanderMertens/flecs), both installed with Conan
from ConanCenter, to write a
[GDExtension](https://docs.godotengine.org/en/stable/tutorials/scripting/cpp/about_godot_cpp.html)
for Godot 4.

The extension registers a `Swarm` node that simulates 100,000 particles with
flecs, an Entity Component System, and draws them with a single `MultiMesh`.
The particles flee from the mouse cursor and bounce off the window edges.

## Build

```bash
$ conan build . --build=missing
```

This builds `demo/bin/libgdexample.template_debug.<ext>`, the library Godot
loads from the editor and when running the game with debug enabled. For the
library used by release exports, build again with:

```bash
$ conan build . -o "godot-cpp/*:target=template_release" --build=missing
```

godot-cpp requires C++17, so add `-s compiler.cppstd=17` if your default
profile uses an older standard.

## Run

Open `demo/project.godot` with Godot 4.7 and press Play (F5), or run it from
the terminal:

```bash
$ cd demo
$ godot --import
$ godot
```
