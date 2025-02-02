#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"


class RigidCircle {
public:
    Vector2 prevPos;
    Vector2 pos;

    Vector2 cumulativeImpulse;

    float radius;
    float mass;

public:
    RigidCircle() {}

    RigidCircle(struct Vector2 _pos, float _radius, float _mass);

    const Vector2& GetPos() const { return pos; }

    float GetRadius() { return radius; }

    float GetRadius() const { return radius; }

    float GetMass() { return mass; }

    const Vector2& GetImpulse() { return cumulativeImpulse; }

    void ApplyImpulse(Vector2 impulse);

    void SetPos(Vector2 newPos) {
        pos = newPos;
    }

    void Draw();
};