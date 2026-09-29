#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "swarm.h"

using namespace godot;

void initialize_gdexample_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    // A runtime class only runs its callbacks in the game, not in the editor
    GDREGISTER_RUNTIME_CLASS(Swarm);
}

void uninitialize_gdexample_module(ModuleInitializationLevel p_level) {
}

extern "C" {
// Entry point that Godot calls when loading the library. Its name must match
// the entry_symbol in demo/bin/gdexample.gdextension
GDExtensionBool GDE_EXPORT gdexample_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
        GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
    GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_gdexample_module);
    init_obj.register_terminator(uninitialize_gdexample_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}
}
