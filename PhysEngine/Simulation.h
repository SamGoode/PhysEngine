#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "RigidBody.h"
#include "Collision.h"


#define MAX_COLLISIONS 64

class Simulation {
private:
    Vector4 boundary = { 0.f, 0.f, 1800.f, 900.f };
    Vector2 gravity = { 0.f, 200.f };

    float biasSlop = 0.2f;
    float biasFactor = 0.1f;

    float elasticity = 0.f;
    float friction = 0.2f;

    RigidBody* bodies[64];

    int rigidBodyCount = 0;

    const int maxCollisions = MAX_COLLISIONS;
    Collision collisions[MAX_COLLISIONS];
    int collisionCount = 0;

public:
    Simulation();
    ~Simulation();

    void AddRigidBody(RigidBody* newBody);

    void AddCollision(Collision collision) { collisions[collisionCount] = collision; collisionCount++; }
    void ClearCollisions() { collisionCount = 0; }

    void CheckBoundaryCollision(RigidBody* body);
    void CheckCollision(RigidBody* A, RigidBody* B);
    void CheckCollisionRR(RigidBody* A, RigidBody* B);
    void CheckCollisionRC(RigidBody* A, RigidBody* B);
    void CheckCollisionCC(RigidBody* A, RigidBody* B);

    void SolveFriction(Collision& collision);
    void SolveFrictionPair(Collision& collision1, Collision& collision2);

    void ResolveCollision(Collision& collision, float DeltaTime);
    void ResolveCollisionPair(Collision& collision1, Collision& collision2, float DeltaTime);

    void InitialStep(float DeltaTime);
    void Step(float DeltaTime);

    void Draw();
};