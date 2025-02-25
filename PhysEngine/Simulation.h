#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "RigidBody.h"
#include "Collision.h"


#define MAX_COLLISIONS 64
#define MAX_JOINTS 16

class Simulation {
private:
    Vector4 boundary = { 0.f, 0.f, 1800.f, 900.f };
    Vector2 gravity = { 0.f, 0.f };

    int solverIterations = 4;

    float biasSlop = 0.5f;
    float biasFactor = 0.2f;

    float elasticity = 0.f;
    float friction = 0.8f;

    RigidBody* bodies[64];

    int rigidBodyCount = 0;

    int collisionCount = 0;
    const int maxCollisions = MAX_COLLISIONS;
    Collision collisions[MAX_COLLISIONS];

    int jointCount = 0;
    const int maxJoints = MAX_JOINTS;
    Joint joints[MAX_JOINTS];

public:
    Simulation();
    ~Simulation() { for (int i = 0; i < rigidBodyCount; i++) { delete bodies[i]; } }

    void AddRigidBody(RigidBody* newBody) { bodies[rigidBodyCount++] = newBody; }

    void AddCollision(Collision collision) { collisions[collisionCount] = collision; collisionCount++; }
    void ClearCollisions() { collisionCount = 0; }

    void CheckBoundaryCollision(RigidBody* body);
    void CheckCollision(RigidBody* A, RigidBody* B);
    void CheckCollisionRR(RigidBody* A, RigidBody* B);
    void CheckCollisionRC(RigidBody* A, RigidBody* B);
    void CheckCollisionCC(RigidBody* A, RigidBody* B);

    void SolveJointPosition(Joint& joint);
    void SolveJoint(Joint& joint);

    void ApplyTorque(Joint& joint, float torque);

    void SolvePosition(Collision& collision);

    void SolveFriction(Collision& collision);
    void SolveFrictionPair(Collision& collision1, Collision& collision2);

    void SolveImpulse(Collision& collision);
    void SolveImpulsePair(Collision& collision1, Collision& collision2);

    void ApplyRestitution(Collision& collision);

    void InitialStep(float DeltaTime);
    void Step(float DeltaTime);

    void Draw();
};