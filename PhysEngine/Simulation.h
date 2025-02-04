#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "RigidBody.h"
#include "StaticBody.h"

class Simulation {
private:
    Vector4 boundary;
    Vector2 gravity;

    RigidBody* bodies[64];
    StaticBody* statics[64];

    int rigidBodyCount;
    int staticBodyCount;

public:
    Simulation();
    ~Simulation();

    bool CheckBoundaryCollision(RigidBody* body, struct CollisionInfo& result);

    void InitialStep(float DeltaTime);

    void Step(float DeltaTime);

    void Draw();
};