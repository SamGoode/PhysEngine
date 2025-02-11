#pragma once

#include "raylib.h"

#include "RigidBody.h"

struct Collision {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Vector2 worldNormal;
    Vector2 pointA;
    Vector2 pointB;
    float depth;

    float lambdaSum = 0.f;
};