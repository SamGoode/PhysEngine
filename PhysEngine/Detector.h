#pragma once

#include "rlshort.h"
#include "Collision.h"

#define MAX_COLLISIONS 64

class Detector {
public:
    int collisionCount = 0;
    const int maxCollisions = MAX_COLLISIONS;
    Collision collisions[MAX_COLLISIONS];

public:
    void CheckBoundaryCollision(class RigidBody* body, Vector4 boundary);

    void CheckCollision(RigidBody* A, RigidBody* B);
    void CheckCollisionRR(RigidBody* A, RigidBody* B);
    void CheckCollisionRC(RigidBody* A, RigidBody* B);
    void CheckCollisionCC(RigidBody* A, RigidBody* B);

    void AddCollision(Collision collision) { collisions[collisionCount++] = collision; }
    void ClearCollisions() { collisionCount = 0; }

    bool IsWithinBody(RigidBody* body, Vector2 pos);
};