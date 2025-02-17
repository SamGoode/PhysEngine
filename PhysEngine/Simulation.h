#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "RigidBody.h"
#include "Collision.h"


#define MAX_COLLISIONS 64

class Simulation {
private:
    Vector4 boundary;
    Vector2 gravity;

    float biasSlop;
    float biasFactor;

    float elasticity = 0.5f;
    float friction = 0.5f;

    RigidBody* bodies[64];

    int rigidBodyCount;

    const int maxCollisions = MAX_COLLISIONS;
    Collision collisions[MAX_COLLISIONS];
    int collisionCount = 0;

public:
    Simulation();
    ~Simulation();

    void AddCollision(Collision collision) { collisions[collisionCount] = collision; collisionCount++; }
    void ClearCollisions() { collisionCount = 0; }

    void CheckBoundaryCollision(RigidBody* body);
    void CheckCollision(RigidBody* A, RigidBody* B);
    void CheckCollisionRR(RigidBody* A, RigidBody* B);
    void CheckCollisionRC(RigidBody* A, RigidBody* B);
    void CheckCollisionCC(RigidBody* A, RigidBody* B);

    void ResolveCollision(Collision& collision, float DeltaTime);
    void ResolveCollisionPair(Collision& collisionA, Collision& collisionB, float DeltaTime);

    void InitialStep(float DeltaTime);

    void Step(float DeltaTime);

    void Draw();
};