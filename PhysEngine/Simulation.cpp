#include "Simulation.h"

#include <cmath>
#include <iostream>
#include <string>

#include "rlshort.h"

#include "Collision.h"

#include "StaticBody.h"

Simulation::Simulation() {
    boundary = { 0.f, 0.f, 1800.f, 900.f };
    gravity = { 0.f, 100.f };

    rigidBodyCount = 0;
    staticBodyCount = 0;

    //bodies[0] = new RigidCircle({ 900.f, 100.f }, 10.f, 25.f);
    //rigidBodyCount++;

    bodies[0] = new RigidRect({ 1000.f, 200.f }, 50.f, 200.f, 50.f, -PI/8);
    rigidBodyCount++;

    bodies[1] = new RigidRect({ 700.f, 400.f }, 1.f, 300.f, 50.f, PI / 4);
    bodies[1]->SetStatic();
    rigidBodyCount++;

    bodies[2] = new RigidRect({ 1100.f, 600.f }, 1.f, 300.f, 50.f, -PI / 4);
    bodies[2]->SetStatic();
    rigidBodyCount++;
}

Simulation::~Simulation() {
    for (int i = 0; i < rigidBodyCount; i++) {
        delete bodies[i];
    }

    for (int i = 0; i < staticBodyCount; i++) {
        delete statics[i];
    }
}

void Simulation::CheckBoundaryCollision(RigidBody* body) {
    RigidCircle* circle = dynamic_cast<RigidCircle*>(body);
    if (circle) {
        if (circle->pos.y > boundary.w - circle->radius) {
            Collision collision;
            collision.bodyA = circle;
            collision.bodyB = nullptr;
            collision.worldNormal = { 0, 1.f };
            collision.depth = circle->pos.y + circle->radius - boundary.w;
            collision.pointA = circle->pos + collision.worldNormal * circle->radius;
            
            AddCollision(collision);
        }
        if (circle->pos.y < boundary.y + circle->radius) {
            Collision collision;
            collision.bodyA = circle;
            collision.bodyB = nullptr;
            collision.worldNormal = { 0, -1.f };
            collision.depth = -circle->pos.y + circle->radius + boundary.y;
            collision.pointA = circle->pos + collision.worldNormal * circle->radius;

            AddCollision(collision);
        }
        if (circle->pos.x < boundary.x + circle->radius) {
            Collision collision;
            collision.bodyA = circle;
            collision.bodyB = nullptr;
            collision.worldNormal = { -1.f, 0.f };
            collision.depth = -circle->pos.x + circle->radius + boundary.x;
            collision.pointA = circle->pos + collision.worldNormal * circle->radius;

            AddCollision(collision);
        }
        if (circle->pos.x > boundary.z - circle->radius) {
            Collision collision;
            collision.bodyA = circle;
            collision.bodyB = nullptr;
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

                AddCollision(collision);
            }
            if (cornerPos.y < boundary.y) {
                Collision collision = {
                    .bodyA = rect,
                    .bodyB = nullptr,
                    .worldNormal = { 0.f, -1.f },
                    .pointA = cornerPos,
                    .depth = -cornerPos.y + boundary.y
                };

                AddCollision(collision);
            }
            if (cornerPos.x < boundary.x) {
                Collision collision = {
                    .bodyA = rect,
                    .bodyB = nullptr,
                    .worldNormal = { -1.f, 0.f },
                    .pointA = cornerPos,
                    .depth = -cornerPos.x + boundary.x
                };

                AddCollision(collision);
            }
            if (cornerPos.x > boundary.z) {
                Collision collision = {
                    .bodyA = rect,
                    .bodyB = nullptr,
                    .worldNormal = { 1.f, 0.f },
                    .pointA = cornerPos,
                    .depth = cornerPos.x - boundary.z
                };

                AddCollision(collision);
            }
        }
    }
}

//void resolveBoundaryCollision(RigidBody* A, const Collision& result, float& lambdaSum, float DeltaTime) {
//    Vector2 velA = A->vel;
//    float angVelA = A->angVel;
//
//    float invMassA = A->invMass;
//    float invMOIA = A->invMOI;
//
//    Vector2 norm = result.worldNormal;
//
//    Vector2 radA = result.pointA - A->pos;
//    Vector2 radPerpA = { -radA.y, radA.x };
//
//    float bias = -result.depth * (0.1f / DeltaTime);
//
//    float JV = Vector2DotProduct(norm * -1, velA) + (Vector2DotProduct(norm, radPerpA) * -angVelA);
//    float effMass = invMassA + (Vector2DotProduct(norm, radPerpA) * Vector2DotProduct(norm, radPerpA) * invMOIA);
//
//    float lambda = -(JV + bias) / effMass;
//
//    float oldSum = lambdaSum;
//    lambdaSum = std::max(lambdaSum + lambda, 0.f);
//
//    lambda = lambdaSum - oldSum;
//
//    A->vel += (norm * -invMassA) * lambda;
//    A->angVel += Vector2DotProduct(norm, radPerpA) * -invMOIA * lambda;
//}

void Simulation::CheckCollision(RigidBody* A, RigidBody* B) {
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

        int minOverlapIndex = -1;
        float minOverlap = FLT_MAX;

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
            }
            else {
                overlap = abs(r1.first - r2.second);
            }

            if (overlap < minOverlap) {
                minOverlapIndex = i;
                minOverlap = overlap;
            }
        }

        Collision collision;
        collision.bodyA = rectA;
        collision.bodyB = rectB;
        collision.worldNormal = normals[minOverlapIndex];
        collision.depth = minOverlap;

        if (minOverlapIndex < 4) {
            Vector2 cornerA = cornersA[minOverlapIndex];

            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(cornersB[i] - cornerA, collision.worldNormal) < 0.f) {
                    collision.pointA = cornersB[i] + collision.worldNormal * collision.depth;
                    collision.pointB = cornersB[i];
                    break; //Maybe remove this later?
                }
            }
        }
        else {
            Vector2 cornerB = cornersB[minOverlapIndex - 4];
            
            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(cornersA[i] - cornerB, collision.worldNormal) < 0.f) {
                    collision.pointA = cornersA[i];
                    collision.pointB = cornersA[i] - collision.worldNormal * collision.depth;
                    break; //Maybe remove this later?
                }
            }
        }

        AddCollision(collision);
    }
}

void Simulation::ResolveCollision(Collision& collision, float DeltaTime) {
    RigidBody* A = collision.bodyA;
    RigidBody* B = collision.bodyB;
    
    Vector2 norm = collision.worldNormal;

    Vector2 velA = A->vel;
    float angVelA = A->angVel;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;

    Vector2 radA = collision.pointA - A->pos;
    Vector2 radPerpA = { -radA.y, radA.x };

    Vector2 velB = { 0.f, 0.f };
    float angVelB = 0.f;

    float invMassB = 0.f;
    float invMOIB = 0.f;

    Vector2 radB = { 0.f, 0.f };
    Vector2 radPerpB = { 0.f, 0.f };

    if (B) {
        velB = B->vel;
        angVelB = B->angVel;

        invMassB = B->invMass;
        invMOIB = B->invMOI;

        radB = collision.pointB - B->pos;
        radPerpB = { -radB.y, radB.x };
    }

    float bias = -collision.depth * (0.1f/DeltaTime);

    float JV = Vector2DotProduct(norm * -1, velA) + (Vector2DotProduct(norm, radPerpA) * -angVelA) + Vector2DotProduct(norm, velB) + (Vector2DotProduct(norm, radPerpB) * angVelB);
    float effMass = invMassA + (Vector2DotProduct(norm, radPerpA) * Vector2DotProduct(norm, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);

    float lambda = -(JV + bias) / effMass;

    float oldSum = collision.lambdaSum;
    collision.lambdaSum = std::max(collision.lambdaSum + lambda, 0.f);
    
    lambda = collision.lambdaSum - oldSum;

    A->vel += (norm * -invMassA) * lambda;
    A->angVel += Vector2DotProduct(norm, radPerpA) * -invMOIA * lambda;

    if (B) {
        B->vel += (norm * invMassB) * lambda;
        B->angVel += Vector2DotProduct(norm, radPerpA) * invMOIB * lambda;
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

    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* A = bodies[i];

        // Boundary collision check
        CheckBoundaryCollision(A);

        // RigidBody collision check
        for (int n = i + 1; n < rigidBodyCount; n++) {
            RigidBody* B = bodies[n];

            CheckCollision(A, B);
        }
    }


    // Sequential Impulse Solver
    int iterations = 5;
    for (int i = 0; i < iterations; i++) {
        for (int n = 0; n < collisionCount; n++) {
            ResolveCollision(collisions[n], DeltaTime);
            std::cout << n << std::endl;
        }
    }
    ClearCollisions();

    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* body = bodies[i];

        if (body->isStatic) { continue; }

        // Second half of kinematics update
        body->vel += body->acc * DeltaTime * 0.5;
        body->angVel += body->angAcc * DeltaTime * 0.5;
    }

    //for (int i = 0; i < rigidBodyCount; i++) {
    //    RigidBody* body = bodies[i];

    //    body->vel += body->acc * DeltaTime * 0.5;
    //    body->pos += body->vel * DeltaTime;
    //    body->acc = { 0.f, 0.f };

    //    body->angVel += body->angAcc * DeltaTime * 0.5;
    //    body->rot += body->angVel * DeltaTime;
    //    body->angAcc = 0.f;

    //    body->acc += gravity;

    //    CollisionInfo boundaryCollision;
    //    if (CheckBoundaryCollision(body, boundaryCollision)) {
    //        Vector2 radial = boundaryCollision.impact - body->pos;
    //        Vector2 radialPerp = { -radial.y, radial.x };

    //        Vector2 impactVel = body->vel + radialPerp * body->angVel;

    //        float relNormVel = Vector2DotProduct(impactVel, boundaryCollision.normal);
    //        float biasVel = boundaryCollision.depth * (0.5f / DeltaTime) * (relNormVel / abs(relNormVel));
    //        float radialPerpNorm = Vector2DotProduct(radialPerp, boundaryCollision.normal);

    //        float impulseMag = (-1.f * (relNormVel + biasVel)) / (body->invMass + (radialPerpNorm * radialPerpNorm * body->invMOI));
    //        Vector2 impulse = boundaryCollision.normal * impulseMag;
    //        float angularImpulse = Vector2DotProduct(radialPerp, impulse);

    //        body->acc += impulse * body->invMass * (1 / DeltaTime);
    //        body->angAcc += angularImpulse * body->invMOI * (1 / DeltaTime);
    //        
    //        std::cout << "collision happened" << std::endl;
    //    }

    //    for (int n = 0; n < staticBodyCount; n++) {
    //        StaticBody* staticBody = statics[n];

    //        CollisionInfo collision;
    //        if (!staticBody->CheckCollision(body, collision)) { continue; }

    //        Vector2 worldNormal = Vector2Rotate(collision.normal, staticBody->GetRotation());
    //        
    //        Vector2 radial = collision.impact - body->pos;
    //        Vector2 radialPerp = { -radial.y, radial.x };

    //        Vector2 impactVel = body->vel + radialPerp * body->angVel;

    //        float relNormVel = Vector2DotProduct(impactVel, worldNormal);
    //        float biasVel = collision.depth * (0.6f / DeltaTime) * (relNormVel/abs(relNormVel));
    //        float radialPerpNorm = Vector2DotProduct(radialPerp, worldNormal);

    //        float impulseMag = (-1.f * (relNormVel + biasVel)) / (body->invMass + (radialPerpNorm * radialPerpNorm * body->invMOI));
    //        Vector2 impulse = worldNormal * impulseMag;
    //        float angularImpulse = Vector2DotProduct(radialPerp, impulse);

    //        body->acc += impulse * body->invMass * (1 / DeltaTime);
    //        body->angAcc += angularImpulse * body->invMOI * (1 / DeltaTime);

    //        std::cout << "collision happened" << std::endl;
    //    }

    //    body->vel += (body->acc * 0.5 * DeltaTime);
    //    body->angVel += (body->angAcc * 0.5 * DeltaTime);
    //}
}

void Simulation::Draw() {
    for (int i = 0; i < rigidBodyCount; i++) {
        bodies[i]->Draw();
    }

    for (int i = 0; i < staticBodyCount; i++) {
        statics[i]->Draw();
    }

    DrawVectorText(bodies[0]->pos, 20, 50, 20, BLUE);

    DrawVectorText(bodies[0]->vel, 20, 100, 20, BLUE);

    DrawText(std::to_string(bodies[0]->angVel).c_str(), 20, 150, 20, BLUE);
}