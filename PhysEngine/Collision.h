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

    Vector2 impulseVelA = { 0.f, 0.f };
    float impulseAngVelA = 0.f;
    Vector2 impulseVelB = { 0.f, 0.f };
    float impulseAngVelB = 0.f;
};