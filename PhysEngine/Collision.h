#pragma once

#include "raylib.h"

#include "RigidBody.h"

struct Collision {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Vector2 worldNormal = { 0.f, 0.f };
    Vector2 pointA = { 0.f, 0.f };
    Vector2 pointB = { 0.f, 0.f };
    float depth = 0.f;

    float lambdaSum = 0.f;
    float tangentLambdaSum = 0.f;
};

struct Joint {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Vector2 localA = { 0.f, 0.f };
    Vector2 localB = { 0.f, 0.f };
};

struct MouseJoint {
    RigidBody* bodyA = nullptr;

    Vector2 localA = { 0.f, 0.f };
    Vector2 mousePos = { 0.f, 0.f };
};