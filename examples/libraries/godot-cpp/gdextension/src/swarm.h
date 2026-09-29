#pragma once

#include <flecs.h>
#include <godot_cpp/classes/multi_mesh.hpp>
#include <godot_cpp/classes/node2d.hpp>

namespace godot {

// flecs components are plain structs
struct Position {
    float x, y;
};

struct Velocity {
    float x, y;
};

// A Node2D that simulates a swarm of particles with flecs, an Entity
// Component System, and draws all of them with a single MultiMesh. The
// particles flee from the mouse cursor and bounce off the window edges.
class Swarm : public Node2D {
    GDCLASS(Swarm, Node2D)

    static constexpr float CRUISE_SPEED = 40.0f;

    int count = 100000;
    double flee_radius = 150.0;

    flecs::world world;
    flecs::query<const Position, const Velocity> render_query;
    Ref<MultiMesh> multimesh;
    PackedFloat32Array buffer;
    Vector2 mouse;
    Vector2 bounds;

protected:
    static void _bind_methods();

public:
    void _ready() override;
    void _process(double p_delta) override;

    void set_count(int p_count);
    int get_count() const;
    void set_flee_radius(double p_radius);
    double get_flee_radius() const;
};

} // namespace godot
