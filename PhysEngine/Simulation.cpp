#include "Simulation.h"

#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>

#include "rlshort.h"

#include "Collision.h"
#include "Solver.h"


Simulation::Simulation() {
    // Dynamic Objects
    
    // Floating joint thing
    //AddRigidBody(new RigidRect({ 900.f, 400.f }, 100.f, 100.f, 100.f, 0.f));
    //AddRigidBody(new RigidRect({ 850.f, 450.f }, 100.f, 100.f, 100.f, 0.f));
    //AddJoint({ bodies[0], bodies[1], {-25.f, 25.f}, {25.f, -25.f} });

    // Car thing
    AddRigidBody(new RigidRect({ 6.f, 2.f }, 200.f, 1.5f, 1.f, 0.f));
    //AddRigidBody(new RigidRect({ 750.f, 480.f }, 100.f, 100.f, 100.f, 0.f));
    //AddRigidBody(new RigidRect({ 1050.f, 480.f }, 100.f, 100.f, 100.f, 0.f));
    AddRigidBody(new RigidCircle({ 5.93f, 2.04f }, 10.f, 0.3f));
    AddRigidBody(new RigidCircle({ 6.07f, 2.04f }, 10.f, 0.3f));
    AddJoint({ bodies[0], bodies[1], {-0.7f, 0.4f}, {0.f, 0.f} });
    AddJoint({ bodies[0], bodies[2], {0.7f, 0.4f}, {0.f, 0.f} });

    // Random Objects
    AddRigidBody(new RigidRect({ 7.f, 2.f }, 40.f, 1.f, 1.f, PI / 3));
    AddRigidBody(new RigidRect({ 11.f, 2.f }, 50.f, 1.f, 2.5f, -PI / 3));
    AddRigidBody(new RigidCircle({ 7.f, 1.f }, 20.f, 0.5f));

    // Static Objects
    AddRigidBody(new RigidRect({ 1.f, 5.2f }, 0.f, 2.f, 0.5f, 0.f));
    AddRigidBody(new RigidRect({ 4.f, 4.5f }, 0.f, 6.f, 0.5f, -PI / 12));
    AddRigidBody(new RigidRect({ 16.f, 8.f }, 0.f, 5.f, 0.5f, -PI / 6));
    AddRigidBody(new RigidCircle({ 8.f, 6.5f }, 0.f, 2.f));

    bodies[1]->SetColor(PURPLE);
    bodies[2]->SetColor(PURPLE);
}

void Simulation::ApplyAngularImpulse(Joint& joint, float angularImpulse) {
    RigidBody* A = joint.bodyA;
    RigidBody* B = joint.bodyB;

    A->ApplyAngularImpulse(-angularImpulse);
    B->ApplyAngularImpulse(angularImpulse);
}


void Simulation::InitialStep(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* body = bodies[i];

        if (body->isStatic) { continue; }

        body->acc += gravity;

        body->pos += body->vel * DeltaTime + body->acc * (DeltaTime * DeltaTime * 0.5f);
        body->vel += body->acc * DeltaTime; //This assumes acceleration is the same next tick.

        body->rot += body->angVel * DeltaTime + body->angAcc * (DeltaTime * DeltaTime * 0.5f);
        body->angVel += body->angAcc * DeltaTime;
    }
}


void Simulation::Step(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        bodies[i]->Update(DeltaTime);

        // Gravity application
        bodies[i]->acc += gravity;
    }

    // Motor Controls
    if (IsKeyDown(KEY_RIGHT)) {
        //ApplyAngularImpulse(joints[0], motorImpulse);
        for (int i = 0; i < jointCount; i++) {
            ApplyAngularImpulse(joints[i], motorImpulse);
        }
    }
    if (IsKeyDown(KEY_LEFT)) {
        //ApplyAngularImpulse(joints[1], -motorImpulse);
        for (int i = 0; i < jointCount; i++) {
            ApplyAngularImpulse(joints[i], -motorImpulse);
        }
    }

    if (IsKeyPressed(KEY_DOWN)) {
        //RigidBody* B = joints[1].bodyB;
        //ApplyAngularImpulse(joints[1], -B->angVel / B->invMOI);

        for (int i = 0; i < jointCount; i++) {
            RigidBody* B = joints[i].bodyB;
            ApplyAngularImpulse(joints[i], -B->angVel/B->invMOI);
        }
    }

    // Mouse Joint
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        for (int i = 0; i < rigidBodyCount; i++) {
            if (bodies[i]->isStatic) { continue; }

            Vector2 mousePos = GetMousePosition() / scale;

            if (d.IsWithinBody(bodies[i], mousePos)) {
                mouseJoint.bodyA = bodies[i];
                
                Vector2 toMouse = mousePos - bodies[i]->pos;
                mouseJoint.localA = Vector2Rotate(toMouse, -bodies[i]->rot);
                break;
            }
        }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        mouseJoint.bodyA = nullptr;
        mouseJoint.localA = { 0.f, 0.f };
        mouseJoint.mousePos = { 0.f, 0.f };
    }

    if (mouseJoint.bodyA) {
        mouseJoint.mousePos = GetMousePosition() / scale;
        for (int i = 0; i < s.iterations; i++) {
            s.SolveMouseJoint(mouseJoint, DeltaTime);
        }
    }


    // Solve joint positions
    for (int i = 0; i < s.iterations; i++) {
        for (int n = 0; n < jointCount; n++) {
            s.SolveJointPosition(joints[n]);
        }
    }

    // Solve joint velocity
    for (int i = 0; i < s.iterations; i++) {
        for (int n = 0; n < jointCount; n++) {
            s.SolveJointVelocity(joints[n]);
        }
    }

    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* A = bodies[i];

        // Boundary collision check
        d.CheckBoundaryCollision(A, boundary);

        // RigidBody collision check
        for (int n = i + 1; n < rigidBodyCount; n++) {
            RigidBody* B = bodies[n];

            bool hasJoint = false;
            for (int jointIndex = 0; jointIndex < jointCount; jointIndex++) {
                if (A != joints[jointIndex].bodyA && A != joints[jointIndex].bodyB) {
                    continue;
                }
                if (B != joints[jointIndex].bodyA && B != joints[jointIndex].bodyB) {
                    continue;
                }

                hasJoint = true;
                break;
            }

            if (hasJoint) {
                continue;
            }

            d.CheckCollision(A, B);
        }
    }
    
    // Sequential Position Solver
    for (int i = 0; i < s.iterations; i++) {
        for (int n = 0; n < d.collisionCount; n++) {
            s.SolvePosition(d.collisions[n]);
        }
    }

    // Sequential Impulse Solver
    for (int i = 0; i < s.iterations; i++) {
        // Apply Friction Impulses
        for (int n = 0; n < d.collisionCount; n++) {
            if (n + 1 < d.collisionCount) {
                if (d.collisions[n].bodyA == d.collisions[n + 1].bodyA && d.collisions[n].bodyB == d.collisions[n + 1].bodyB && d.collisions[n].worldNormal == d.collisions[n + 1].worldNormal) {
                    s.SolveFrictionPair(d.collisions[n], d.collisions[n + 1]);
                    n++;
                    continue;
                }
            }

            s.SolveFriction(d.collisions[n]);
        }

        // Apply Normal Impulses
        for (int n = 0; n < d.collisionCount; n++) {
            if (n + 1 < d.collisionCount) {
                if (d.collisions[n].bodyA == d.collisions[n + 1].bodyA && d.collisions[n].bodyB == d.collisions[n + 1].bodyB && d.collisions[n].worldNormal == d.collisions[n + 1].worldNormal) {
                    s.SolveImpulsePair(d.collisions[n], d.collisions[n + 1]);
                    n++;
                    continue;
                }
            }

            s.SolveImpulse(d.collisions[n]);
        }
    }

    // Apply Elastic Impulses
    for (int i = 0; i < d.collisionCount; i++) {
        s.ApplyRestitution(d.collisions[i]);
    }

    d.ClearCollisions();

    for (int i = 0; i < rigidBodyCount; i++) {
        bodies[i]->PrepUpdate(DeltaTime);
    }
}

void Simulation::Draw() {
    for (int i = 0; i < rigidBodyCount; i++) {
        bodies[i]->Draw(scale);
    }

    for (int i = 0; i < jointCount; i++) {
        RigidBody* B = joints[i].bodyB;
        Vector2 pos = B->pos + Vector2Rotate(joints[i].localB, B->rot);

        DrawCircle(pos.x * scale, pos.y * scale, 2.f, BLACK);
    }

    if (mouseJoint.bodyA) {
        Vector2 pos = mouseJoint.bodyA->pos + Vector2Rotate(mouseJoint.localA, mouseJoint.bodyA->rot);
        DrawCircle(pos.x * scale, pos.y * scale, 2.f, BLACK);
    }

    
    DrawVectorText(bodies[0]->pos, 20, 50, 20, BLUE);
    DrawVectorText(bodies[0]->vel, 20, 100, 20, BLUE);
    DrawText(std::to_string(bodies[0]->rot).c_str(), 20, 150, 20, BLUE);
    DrawText(std::to_string(bodies[0]->angVel).c_str(), 20, 200, 20, BLUE);

    DrawVectorText(bodies[1]->pos, 420, 50, 20, BLUE);
    DrawVectorText(bodies[1]->vel, 420, 100, 20, BLUE);
    DrawText(std::to_string(bodies[1]->rot).c_str(), 420, 150, 20, BLUE);
    DrawText(std::to_string(bodies[1]->angVel).c_str(), 420, 200, 20, BLUE);
}