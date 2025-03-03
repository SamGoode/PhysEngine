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
    AddRigidBody(new RigidRect({ 900.f, 400.f }, 1000.f, 400.f, 200.f, 0.f));
    //AddRigidBody(new RigidRect({ 750.f, 480.f }, 100.f, 100.f, 100.f, 0.f));
    //AddRigidBody(new RigidRect({ 1050.f, 480.f }, 100.f, 100.f, 100.f, 0.f));
    AddRigidBody(new RigidCircle({ 750.f, 480.f }, 200.f, 50.f));
    AddRigidBody(new RigidCircle({ 1050.f, 480.f }, 200.f, 50.f));
    AddJoint({ bodies[0], bodies[1], {-150.f, 80.f}, {0.f, 0.f} });
    AddJoint({ bodies[0], bodies[2], {150.f, 80.f}, {0.f, 0.f} });


    //AddRigidBody(new RigidRect({ 700.f, 200.f }, 400.f, 100.f, 100.f, PI / 3));
    //AddRigidBody(new RigidRect({ 1100.f, 200.f }, 500.f, 200.f, 25.f, -PI / 3));
    //AddRigidBody(new RigidCircle({ 700.f, 100.f }, 200.f, 50.f));

    // Static Objects
    //AddRigidBody(new RigidRect({ 700.f, 400.f }, 0.f, 300.f, 50.f, PI / 6));
    //AddRigidBody(new RigidRect({ 1000.f, 600.f }, 0.f, 400.f, 50.f, -PI / 4));
    //AddRigidBody(new RigidCircle({ 900.f, 300.f }, 0.f, 50.f));

    bodies[1]->SetColor(PURPLE);
    bodies[2]->SetColor(PURPLE);
}


void Simulation::CheckBoundaryCollision(RigidBody* body) {
    if (body->isStatic) { return; }

    RigidCircle* circle = dynamic_cast<RigidCircle*>(body);
    if (circle) {
        Collision collision;
        collision.bodyA = circle;
        collision.bodyB = nullptr;

        if (circle->pos.y > boundary.w - circle->radius) {
            collision.worldNormal = { 0, 1.f };
            collision.depth = circle->pos.y + circle->radius - boundary.w;
            collision.pointA = circle->pos + collision.worldNormal * circle->radius;
            
            AddCollision(collision);
        }
        if (circle->pos.y < boundary.y + circle->radius) {
            collision.worldNormal = { 0, -1.f };
            collision.depth = -circle->pos.y + circle->radius + boundary.y;
            collision.pointA = circle->pos + collision.worldNormal * circle->radius;

            AddCollision(collision);
        }
        if (circle->pos.x < boundary.x + circle->radius) {
            collision.worldNormal = { -1.f, 0.f };
            collision.depth = -circle->pos.x + circle->radius + boundary.x;
            collision.pointA = circle->pos + collision.worldNormal * circle->radius;

            AddCollision(collision);
        }
        if (circle->pos.x > boundary.z - circle->radius) {
            collision.worldNormal = { 1.f, 0.f };
            collision.depth = circle->pos.x + circle->radius - boundary.z;
            collision.pointA = circle->pos + collision.worldNormal * circle->radius;

            AddCollision(collision);
        }
    }

    RigidRect* rect = dynamic_cast<RigidRect*>(body);
    if (rect) {
        Vector2 corners[4];
        corners[0] = { -rect->width / 2, -rect->height / 2 };
        corners[1] = { rect->width / 2, -rect->height / 2 };
        corners[2] = { rect->width / 2, rect->height / 2 };
        corners[3] = { -rect->width / 2, rect->height / 2 };

        Collision collisions[16];

        for (int i = 0; i < 4; i++) {
            Vector2 cornerPos = rect->pos + Vector2Rotate(corners[i], rect->rot);

            if (cornerPos.y > boundary.w) {
                Collision collision = {
                    .bodyA = rect,
                    .bodyB = nullptr,
                    .worldNormal = { 0.f, 1.f },
                    .pointA = cornerPos,
                    .depth = cornerPos.y - boundary.w,
                };

                collisions[i] = collision;
            }
            else if (cornerPos.y < boundary.y) {
                Collision collision = {
                    .bodyA = rect,
                    .bodyB = nullptr,
                    .worldNormal = { 0.f, -1.f },
                    .pointA = cornerPos,
                    .depth = -cornerPos.y + boundary.y
                };

                collisions[i + 4] = collision;
            }

            if (cornerPos.x < boundary.x) {
                Collision collision = {
                    .bodyA = rect,
                    .bodyB = nullptr,
                    .worldNormal = { -1.f, 0.f },
                    .pointA = cornerPos,
                    .depth = -cornerPos.x + boundary.x
                };

                collisions[i + 8] = collision;
            }
            else if (cornerPos.x > boundary.z) {
                Collision collision = {
                    .bodyA = rect,
                    .bodyB = nullptr,
                    .worldNormal = { 1.f, 0.f },
                    .pointA = cornerPos,
                    .depth = cornerPos.x - boundary.z
                };

                collisions[i + 12] = collision;
            }
        }

        for (int i = 0; i < 16; i++) {
            if (!collisions[i].bodyA) { continue; }

            AddCollision(collisions[i]);
        }
    }
}


std::pair<float, float> MinMaxProjection(Vector2 corners[4], Vector2 axis) {
    float projection = Vector2DotProduct(corners[0], axis);
    float max = projection;
    float min = projection;

    for (int i = 1; i < 4; i++) {
        projection = Vector2DotProduct(corners[i], axis);

        if (projection > max) {
            max = projection;
        }
        if (projection < min) {
            min = projection;
        }
    }

    return std::pair(min, max);
}

void Simulation::CheckCollisionRR(RigidBody* A, RigidBody* B) {
    RigidRect* rectA = dynamic_cast<RigidRect*>(A);
    RigidRect* rectB = dynamic_cast<RigidRect*>(B);

    if (rectA && rectB) {
        Vector2 cornersA[4] = {
            -rectA->width / 2, -rectA->height / 2,
            rectA->width / 2, -rectA->height / 2,
            rectA->width / 2, rectA->height / 2,
            -rectA->width / 2, rectA->height / 2,
        };

        Vector2 cornersB[4] = {
            -rectB->width / 2, -rectB->height / 2,
            rectB->width / 2, -rectB->height / 2,
            rectB->width / 2, rectB->height / 2,
            -rectB->width / 2, rectB->height / 2,
        };

        Vector2 normals[8];
        for (int i = 0; i < 4; i++) {
            cornersA[i] = rectA->pos + Vector2Rotate(cornersA[i], rectA->rot);
            normals[i] = Vector2Rotate({ 0.f, -1.f }, rectA->rot + (PI * 0.5 * i));

            cornersB[i] = rectB->pos + Vector2Rotate(cornersB[i], rectB->rot);
            normals[i + 4] = Vector2Rotate({ 0.f, -1.f }, rectB->rot + (PI * 0.5 * i));
        }

        int minOverlapIndexA = -1;
        float minOverlapA = FLT_MAX;

        int minOverlapIndexB = -1;
        float minOverlapB = FLT_MAX;

        for (int i = 0; i < 8; i++) {
            Vector2 normal = normals[i];

            std::pair r1 = MinMaxProjection(cornersA, normal);
            std::pair r2 = MinMaxProjection(cornersB, normal);

            if (r1.second < r2.first || r2.second < r1.first) {
                return;
            }

            float overlap;

            if (i < 4) {
                overlap = abs(r1.second - r2.first);

                if (overlap < minOverlapA) {
                    minOverlapIndexA = i;
                    minOverlapA = overlap;
                }
            }
            else {
                overlap = abs(r2.second - r1.first);

                if (overlap < minOverlapB) {
                    minOverlapIndexB = i;
                    minOverlapB = overlap;
                }
            }
        }

        Collision collision;
        collision.bodyA = rectA;
        collision.bodyB = rectB;
        
        if (minOverlapA <= minOverlapB) {
            Vector2 normal = normals[minOverlapIndexA];
            collision.worldNormal = normal;
            Vector2 cornerA = cornersA[minOverlapIndexA];

            Vector2 nextCorner = cornersA[(minOverlapIndexA + 1) % 4];
            float edgeLength = Vector2Distance(cornerA, nextCorner);
            Vector2 midpoint = Vector2Lerp(cornerA, nextCorner, 0.5f);

            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(cornersB[i] - cornerA, normal) < 0.f) {
                    if (Vector2Distance(cornersB[i], midpoint) > edgeLength / 2) { continue; }

                    collision.depth = abs(Vector2DotProduct(cornerA, normal) - Vector2DotProduct(cornersB[i], normal));
                    collision.pointA = cornersB[i] + normal * collision.depth;
                    collision.pointB = cornersB[i];

                    AddCollision(collision);
                }
            }
        }
        if(minOverlapB <= minOverlapA) {
            Vector2 normal = normals[minOverlapIndexB];
            collision.worldNormal = normal * -1;
            Vector2 cornerB = cornersB[minOverlapIndexB - 4];

            Vector2 nextCorner = cornersB[(minOverlapIndexB - 4 + 1) % 4];
            float edgeLength = Vector2Distance(cornerB, nextCorner);
            Vector2 midpoint = Vector2Lerp(cornerB, nextCorner, 0.5f);

            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(cornersA[i] - cornerB, normal) < 0.f) {
                    if (Vector2Distance(cornersA[i], midpoint) > edgeLength / 2) { continue; }

                    collision.depth = abs(Vector2DotProduct(cornerB, normal) - Vector2DotProduct(cornersA[i], normal));
                    collision.pointA = cornersA[i];
                    collision.pointB = cornersA[i] + normal * collision.depth;

                    AddCollision(collision);
                }
            }
        }
    }
}


void Simulation::CheckCollisionRC(RigidBody* A, RigidBody* B) {
    RigidRect* rect = dynamic_cast<RigidRect*>(A);
    RigidCircle* circle = dynamic_cast<RigidCircle*>(B);

    if (rect && circle) {
        Collision collision;
        collision.bodyA = rect;
        collision.bodyB = circle;
        
        Vector2 toCircle = Vector2Rotate(circle->pos - rect->pos, -rect->rot);
        Vector2 absToCircle = { abs(toCircle.x), abs(toCircle.y) };

        if (abs(toCircle.x) > circle->radius + rect->width / 2 || abs(toCircle.y) > circle->radius + rect->height / 2) {
            return;
        }

        if (abs(toCircle.x) > rect->width / 2 && abs(toCircle.y) > rect->height / 2) {
            Vector2 absCorner = { rect->width / 2, rect->height / 2 };

            if (Vector2DistanceSqr(absCorner, absToCircle) > circle->radius * circle->radius) {
                return;
            }

            Vector2 toCorner = { absCorner.x * ((toCircle.x > 0) * 2.f - 1.f), absCorner.y * ((toCircle.y > 0) * 2.f - 1.f) };
            Vector2 cornerPos = rect->pos + Vector2Rotate(toCorner, rect->rot);

            collision.worldNormal = Vector2Normalize(circle->pos - cornerPos);
            collision.pointA = cornerPos;
            collision.pointB = circle->pos - collision.worldNormal * circle->radius;
            collision.depth = Vector2DotProduct(collision.pointB - collision.pointA, collision.worldNormal * -1);

            AddCollision(collision);

            return;
        }

        if (abs(toCircle.x) > rect->width / 2) {
            Vector2 normal = { (toCircle.x > 0) * 2.f - 1.f, 0.f };
            collision.worldNormal = Vector2Rotate(normal, rect->rot);
            collision.pointA = rect->pos + Vector2Rotate({ normal.x * rect->width / 2, toCircle.y }, rect->rot);
            collision.pointB = circle->pos - collision.worldNormal * circle->radius;
            collision.depth = Vector2DotProduct(collision.pointB - collision.pointA, collision.worldNormal * -1);

            AddCollision(collision);
        }
        else {
            Vector2 normal = { 0.f, (toCircle.y > 0) * 2.f - 1.f };
            collision.worldNormal = Vector2Rotate(normal, rect->rot);
            collision.pointA = rect->pos + Vector2Rotate({ toCircle.x, normal.y * rect->height / 2 }, rect->rot);
            collision.pointB = circle->pos - collision.worldNormal * circle->radius;
            collision.depth = Vector2DotProduct(collision.pointB - collision.pointA, collision.worldNormal * -1);

            AddCollision(collision);
        }
    }
}

void Simulation::CheckCollisionCC(RigidBody* A, RigidBody* B) {
    RigidCircle* circleA = dynamic_cast<RigidCircle*>(A);
    RigidCircle* circleB = dynamic_cast<RigidCircle*>(B);

    if (circleA && circleB) {
        float radii = (circleA->radius + circleB->radius);
        Vector2 AtoB = circleB->pos - circleA->pos;

        if (Vector2LengthSqr(AtoB) > radii * radii) {
            return;
        }

        Vector2 normal = Vector2Normalize(AtoB);

        Collision collision = {
            .bodyA = circleA,
            .bodyB = circleB,
            .worldNormal = normal,
            .pointA = circleA->pos + normal * circleA->radius,
            .pointB = circleB->pos - normal * circleB->radius,
            .depth = radii - Vector2Length(AtoB)
        };

        AddCollision(collision);
    }
}


void Simulation::CheckCollision(RigidBody* A, RigidBody* B) {
    if (A->isStatic && B->isStatic) { return; }
    
    if (A->GetID() > B->GetID()) {
        RigidBody* temp = A;
        A = B;
        B = temp;
    }
    
    int total = A->GetID() + B->GetID();
    switch (total) {
    case 0:
        CheckCollisionRR(A, B);
        break;
    case 1:    
        CheckCollisionRC(A, B);
        break;
    case 2:
        CheckCollisionCC(A, B);
        break;
    }
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
        RigidBody* body = bodies[i];

        if (body->isStatic) { continue; }

        // First half of kinematics update
        body->vel += body->acc * DeltaTime * 0.5;
        body->pos += body->vel * DeltaTime;
        body->acc = { 0.f, 0.f };

        body->angVel += body->angAcc * DeltaTime * 0.5;
        body->rot += body->angVel * DeltaTime;
        body->angAcc = 0.f;

        // Gravity application
        body->acc += gravity;
    }

    if (IsKeyDown(KEY_RIGHT)) {
        for (int i = 0; i < jointCount; i++) {
            s.ApplyAngularImpulse(joints[i], motorImpulse);
        }
    }
    if (IsKeyDown(KEY_LEFT)) {
        for (int i = 0; i < jointCount; i++) {
            s.ApplyAngularImpulse(joints[i], -motorImpulse);
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
        CheckBoundaryCollision(A);

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

            CheckCollision(A, B);
        }
    }
    
    // Sequential Position Solver
    for (int i = 0; i < s.iterations; i++) {
        for (int n = 0; n < collisionCount; n++) {
            s.SolvePosition(collisions[n]);
        }
    }

    // Sequential Impulse Solver
    for (int i = 0; i < s.iterations; i++) {
        // Apply Friction Impulses
        for (int n = 0; n < collisionCount; n++) {
            if (n + 1 < collisionCount) {
                if (collisions[n].bodyA == collisions[n + 1].bodyA && collisions[n].bodyB == collisions[n + 1].bodyB && collisions[n].worldNormal == collisions[n + 1].worldNormal) {
                    s.SolveFrictionPair(collisions[n], collisions[n + 1]);
                    n++;
                    continue;
                }
            }

            s.SolveFriction(collisions[n]);
        }

        // Apply Normal Impulses
        for (int n = 0; n < collisionCount; n++) {
            if (n + 1 < collisionCount) {
                if (collisions[n].bodyA == collisions[n + 1].bodyA && collisions[n].bodyB == collisions[n + 1].bodyB && collisions[n].worldNormal == collisions[n + 1].worldNormal) {
                    s.SolveImpulsePair(collisions[n], collisions[n + 1]);
                    n++;
                    continue;
                }
            }

            s.SolveImpulse(collisions[n]);
        }
    }

    // Apply Elastic Impulses
    for (int i = 0; i < collisionCount; i++) {
        s.ApplyRestitution(collisions[i]);
    }

    ClearCollisions();

    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* body = bodies[i];

        if (body->isStatic) { continue; }

        // Second half of kinematics update
        body->vel += body->acc * DeltaTime * 0.5;
        body->angVel += body->angAcc * DeltaTime * 0.5;
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

    DrawVectorText(bodies[0]->pos, 20, 50, 20, BLUE);
    DrawVectorText(bodies[0]->vel, 20, 100, 20, BLUE);

    DrawText(std::to_string(bodies[0]->rot).c_str(), 20, 150, 20, BLUE);
    DrawText(std::to_string(bodies[0]->angVel).c_str(), 20, 200, 20, BLUE);

    //DrawVectorText(bodies[1]->pos, 420, 50, 20, BLUE);
    //DrawVectorText(bodies[1]->vel, 420, 100, 20, BLUE);

    //DrawText(std::to_string(bodies[1]->rot).c_str(), 420, 150, 20, BLUE);
    //DrawText(std::to_string(bodies[1]->angVel).c_str(), 420, 200, 20, BLUE);
}