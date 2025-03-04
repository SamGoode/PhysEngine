#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include "RigidBody.h"
#include "Detector.h"
#include "Solver.h"


#define MAX_BODIES 64
#define MAX_JOINTS 16

class Simulation {
private:
    Vector2 gravity = { 0.f, 10.f };

    float scale = 100.f; // pixels/m
    Vector4 boundary = { 0.f, 0.f, 1800.f / scale, 900.f / scale };

    Detector d;
    Solver s;

    int rigidBodyCount = 0;
    const int maxBodies = MAX_BODIES;
    RigidBody* bodies[MAX_BODIES];

    int jointCount = 0;
    const int maxJoints = MAX_JOINTS;
    Joint joints[MAX_JOINTS];

    MouseJoint mouseJoint;

    float motorImpulse = 1.5f;

public:
    Simulation();
    ~Simulation() { for (int i = 0; i < rigidBodyCount; i++) { delete bodies[i]; } }

    void AddRigidBody(RigidBody* newBody) { bodies[rigidBodyCount++] = newBody; }
    void AddJoint(Joint joint) { joints[jointCount++] = joint; }

    void ApplyAngularImpulse(Joint& joint, float angularImpulse);

    void InitialStep(float DeltaTime);
    void Step(float DeltaTime);

    void Draw();
};