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
    AddRigidBody(new RigidRect({ 800.f, 400.f }, 2000.f, 150.f, 75.f, 0.f));
    //AddRigidBody(new RigidRect({ 750.f, 480.f }, 100.f, 100.f, 100.f, 0.f));
    //AddRigidBody(new RigidRect({ 1050.f, 480.f }, 100.f, 100.f, 100.f, 0.f));
    AddRigidBody(new RigidCircle({ 760.f, 430.f }, 200.f, 35.f));
    AddRigidBody(new RigidCircle({ 840.f, 430.f }, 200.f, 35.f));
    AddJoint({ bodies[0], bodies[1], {-40.f, 30.f}, {0.f, 0.f} });
    AddJoint({ bodies[0], bodies[2], {40.f, 30.f}, {0.f, 0.f} });

    // Random Objects
    //AddRigidBody(new RigidRect({ 700.f, 200.f }, 400.f, 100.f, 100.f, PI / 3));
    //AddRigidBody(new RigidRect({ 1100.f, 200.f }, 500.f, 200.f, 25.f, -PI / 3));
    //AddRigidBody(new RigidCircle({ 700.f, 100.f }, 200.f, 50.f));

    // Static Objects
    AddRigidBody(new RigidRect({ 700.f, 500.f }, 0.f, 500.f, 50.f, -PI / 12));
    //AddRigidBody(new RigidRect({ 1100.f, 600.f }, 0.f, 500.f, 50.f, -PI / 4));
    //AddRigidBody(new RigidCircle({ 900.f, 300.f }, 0.f, 50.f));

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

        body->pos += body->vel * DeltaTime + body->acc * (DeltaTime * DeltaTime * 0.5);
        body->vel += body->acc * DeltaTime; //This assumes acceleration is the same next tick.

        body->rot += body->angVel * DeltaTime + body->angAcc * (DeltaTime * DeltaTime * 0.5);
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
        for (int i = 0; i < jointCount; i++) {
            ApplyAngularImpulse(joints[i], motorImpulse);
        }
    }
    if (IsKeyDown(KEY_LEFT)) {
        for (int i = 0; i < jointCount; i++) {
            ApplyAngularImpulse(joints[i], -motorImpulse);
        }
    }

    if (IsKeyPressed(KEY_DOWN)) {
        RigidBody* B = joints[1].bodyB;
        ApplyAngularImpulse(joints[1], -B->angVel / B->invMOI);

        //for (int i = 0; i < jointCount; i++) {
        //    RigidBody* B = joints[i].bodyB;
        //    ApplyAngularImpulse(joints[i], -B->angVel/B->invMOI);
        //}
    }

    // Mouse Joint
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        for (int i = 0; i < rigidBodyCount; i++) {
            if (d.IsWithinBody(bodies[i], GetMousePosition())) {
                mouseJoint.bodyA = bodies[i];
                
                Vector2 toMouse = GetMousePosition() - bodies[i]->pos;
                mouseJoint.localA = Vector2Rotate(toMouse, -bodies[i]->rot);
                break;
            }
        }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        mouseJoint.bodyA = nullptr;
        mouseJoint.localA = { 0.f, 0.f };
    }

    if (mouseJoint.bodyA) {
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
        bodies[i]->Draw();
    }

    for (int i = 0; i < jointCount; i++) {
        RigidBody* A = joints[i].bodyA;
        Vector2 pos = A->pos + Vector2Rotate(joints[i].localA, A->rot);

        DrawCircle(pos.x, pos.y, 5.f, BLACK);
    }

    if (mouseJoint.bodyA) {
        Vector2 pos = mouseJoint.bodyA->pos + Vector2Rotate(mouseJoint.localA, mouseJoint.bodyA->rot);
        DrawCircle(pos.x, pos.y, 5.f, BLACK);
    }

    
    DrawVectorText(bodies[0]->pos, 20, 50, 20, BLUE);
    DrawVectorText(bodies[0]->vel, 20, 100, 20, BLUE);

    DrawText(std::to_string(bodies[0]->rot).c_str(), 20, 150, 20, BLUE);
    DrawText(std::to_string(bodies[0]->angVel).c_str(), 20, 200, 20, BLUE);

    //DrawVectorText(bodies[1]->pos, 420, 50, 20, BLUE);
    //DrawVectorText(bodies[1]->vel, 420, 100, 20, BLUE);

    //DrawText(std::to_string(bodies[1]->rot).c_str(), 420, 150, 20, BLUE);
    //DrawText(std::to_string(bodies[1]->angVel).c_str(), 420, 200, 20, BLUE);
}