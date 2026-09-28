#include "swarm.h"

#include <godot_cpp/classes/multi_mesh_instance2d.hpp>
#include <godot_cpp/classes/quad_mesh.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void Swarm::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_count", "count"), &Swarm::set_count);
    ClassDB::bind_method(D_METHOD("get_count"), &Swarm::get_count);
    ADD_PROPERTY(PropertyInfo(Variant::INT, "count"), "set_count", "get_count");

    ClassDB::bind_method(D_METHOD("set_flee_radius", "radius"), &Swarm::set_flee_radius);
    ClassDB::bind_method(D_METHOD("get_flee_radius"), &Swarm::get_flee_radius);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "flee_radius"), "set_flee_radius", "get_flee_radius");
}

void Swarm::_ready() {
    bounds = get_viewport_rect().size;

    // Entities are just ids with components attached
    for (int i = 0; i < count; i++) {
        float angle = UtilityFunctions::randf() * Math::TAU;
        world.entity()
                .set<Position>({ static_cast<float>(UtilityFunctions::randf()) * bounds.x,
                        static_cast<float>(UtilityFunctions::randf()) * bounds.y })
                .set<Velocity>({ CRUISE_SPEED * Math::cos(angle), CRUISE_SPEED * Math::sin(angle) });
    }

    // A system runs on every progress() for all entities with these components
    world.system<Position, Velocity>("Move").each([this](flecs::iter &it, size_t, Position &p, Velocity &v) {
        float dt = it.delta_time();

        // Flee from the mouse, harder the closer it is
        float dx = p.x - mouse.x;
        float dy = p.y - mouse.y;
        float dist = Math::sqrt(dx * dx + dy * dy);
        float radius = static_cast<float>(flee_radius);
        if (dist < radius && dist > 1.0f) {
            float push = 3000.0f * (1.0f - dist / radius) * dt;
            v.x += dx / dist * push;
            v.y += dy / dist * push;
        }

        // Slow down back to cruising speed
        float speed = Math::sqrt(v.x * v.x + v.y * v.y);
        if (speed > CRUISE_SPEED) {
            float drag = Math::max(1.0f - 2.0f * dt, CRUISE_SPEED / speed);
            v.x *= drag;
            v.y *= drag;
        }

        p.x += v.x * dt;
        p.y += v.y * dt;

        // Bounce off the window edges
        if (p.x < 0.0f || p.x > bounds.x) {
            v.x = -v.x;
            p.x = Math::clamp(p.x, 0.0f, bounds.x);
        }
        if (p.y < 0.0f || p.y > bounds.y) {
            v.y = -v.y;
            p.y = Math::clamp(p.y, 0.0f, bounds.y);
        }
    });

    render_query = world.query<const Position, const Velocity>();

    // A single MultiMesh draws every particle in one draw call
    Ref<QuadMesh> quad;
    quad.instantiate();
    quad->set_size(Vector2(2.0, 2.0));

    multimesh.instantiate();
    multimesh->set_transform_format(MultiMesh::TRANSFORM_2D);
    multimesh->set_use_colors(true);
    multimesh->set_mesh(quad);
    multimesh->set_instance_count(count);

    MultiMeshInstance2D *instance = memnew(MultiMeshInstance2D);
    instance->set_multimesh(multimesh);
    add_child(instance);

    buffer.resize(count * 12); // per instance: 8 floats of 2D transform and 4 of color
    UtilityFunctions::print("Swarm ready: ", count, " entities simulated with flecs");
}

void Swarm::_process(double p_delta) {
    mouse = get_local_mouse_position();
    world.progress(static_cast<float>(p_delta));

    // Copy every position into the MultiMesh buffer, colored by speed
    float *data = buffer.ptrw();
    render_query.each([&data](const Position &p, const Velocity &v) {
        float speed = Math::min(Math::sqrt(v.x * v.x + v.y * v.y) / 400.0f, 1.0f);
        const float instance[12] = {
            1, 0, 0, p.x, // transform: basis x and origin x
            0, 1, 0, p.y, // transform: basis y and origin y
            0.3f + 0.7f * speed, 0.5f, 1.0f - 0.6f * speed, 1 // color
        };
        for (float value : instance) {
            *data++ = value;
        }
    });
    multimesh->set_buffer(buffer);
}

void Swarm::set_count(int p_count) {
    count = p_count;
}

int Swarm::get_count() const {
    return count;
}

void Swarm::set_flee_radius(double p_radius) {
    flee_radius = p_radius;
}

double Swarm::get_flee_radius() const {
    return flee_radius;
}
