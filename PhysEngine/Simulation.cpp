#include "Simulation.h"

#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>

#include "rlshort.h"

#include "Collision.h"


Simulation::Simulation() {
    // Dynamic Objects
    AddRigidBody(new RigidRect({ 900.f, 400.f }, 500.f, 100.f, 100.f, 0.f));
    AddRigidBody(new RigidRect({ 850.f, 450.f }, 500.f, 100.f, 100.f, 0.f));
    //AddRigidBody(new RigidRect({ 1050.f, 480.f }, 200.f, 100.f, 100.f, 0.f));

    bodies[1]->setColor(PURPLE);
    //bodies[2]->setColor(PURPLE);
    //AddRigidBody(new RigidCircle({ 1050.f, 480.f }, 500.f, 50.f));
    //AddRigidBody(new RigidRect({ 1100.f, 200.f }, 500.f, 200.f, 25.f, -PI / 3));
    ////AddRigidBody(new RigidRect({ 800.f, 200.f }, 300.f, 100.f, 100.f, PI / 3));

    //// Static Objects
    //AddRigidBody(new RigidRect({ 700.f, 400.f }, 0.f, 300.f, 50.f, PI / 6));
    //AddRigidBody(new RigidRect({ 1000.f, 600.f }, 0.f, 400.f, 50.f, -PI / 4));
    //AddRigidBody(new RigidCircle({ 900.f, 300.f }, 0.f, 50.f));

    bodies[0]->jointBody = bodies[1];
    bodies[1]->jointBody = bodies[0];
    
    joints[jointCount] = {
        .bodyA = bodies[0],
        .bodyB = bodies[1],
        .localA = {-25.f, 25.f},
        //.localA = {0.f, 0.f},
        .localB = {25.f, -25.f}
    };
    
    jointCount++;

    //bodies[0]->jointBody = bodies[2];
    //bodies[2]->jointBody = bodies[0];

    //joints[jointCount] = {
    //    .bodyA = bodies[0],
    //    .bodyB = bodies[2],
    //    .localA = {150.f, 80.f},
    //    .localB = {0.f, 0.f}
    //};

    //jointCount++;
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
        
        Vector2 normal = normals[minOverlapIndex];
        if (minOverlapIndex < 4) {
            collision.worldNormal = normal;
            Vector2 cornerA = cornersA[minOverlapIndex];

            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(cornersB[i] - cornerA, normal) < 0.f) {
                    collision.depth = abs(Vector2DotProduct(cornerA, normal) - Vector2DotProduct(cornersB[i], normal));
                    collision.pointA = cornersB[i] + normal * collision.depth;
                    collision.pointB = cornersB[i];

                    AddCollision(collision);
                }
            }
        }
        else {
            collision.worldNormal = normal * -1;
            Vector2 cornerB = cornersB[minOverlapIndex - 4];

            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(cornersA[i] - cornerB, normal) < 0.f) {
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




void Simulation::SolveFriction(Collision& collision) {
    RigidBody* A = collision.bodyA;
    RigidBody* B = collision.bodyB;

    Vector2 norm = collision.worldNormal;
    Vector2 tangent = { -norm.y, norm.x };

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

    float JV = Vector2DotProduct(tangent * -1, velA) + (Vector2DotProduct(tangent * -1, radPerpA) * angVelA) + Vector2DotProduct(tangent, velB) + (Vector2DotProduct(tangent, radPerpB) * angVelB);
    float effMass = invMassA + (Vector2DotProduct(tangent * -1, radPerpA) * Vector2DotProduct(tangent * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(tangent, radPerpB) * Vector2DotProduct(tangent, radPerpB) * invMOIB);

    float tangentLambda = -JV / effMass;

    float maxFriction = collision.lambdaSum * friction;
    float newSum = std::clamp(collision.tangentLambdaSum + tangentLambda, -maxFriction, maxFriction);

    tangentLambda = newSum - collision.tangentLambdaSum;
    collision.tangentLambdaSum += tangentLambda;

    A->vel += (tangent * -invMassA) * tangentLambda;
    A->angVel += Vector2DotProduct(tangent * -1, radPerpA) * invMOIA * tangentLambda;

    if (B) {
        B->vel += (tangent * invMassB) * tangentLambda;
        B->angVel += Vector2DotProduct(tangent, radPerpB) * invMOIB * tangentLambda;
    }
}

void Simulation::SolveFrictionPair(Collision& collision1, Collision& collision2) {
    RigidBody* A = collision1.bodyA;
    RigidBody* B = collision1.bodyB;

    Vector2 norm = collision1.worldNormal;
    Vector2 tangent = { -norm.y, norm.x };

    Vector2 velA = A->vel;
    float angVelA = A->angVel;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;

    Vector2 velB = { 0.f, 0.f };
    float angVelB = 0.f;

    float invMassB = 0.f;
    float invMOIB = 0.f;

    Vector2 radA = collision1.pointA - A->pos;
    Vector2 radPerpA = { -radA.y, radA.x };

    Vector2 radB = { 0.f, 0.f };
    Vector2 radPerpB = { 0.f, 0.f };

    Vector2 radC = collision2.pointA - A->pos;
    Vector2 radPerpC = { -radC.y, radC.x };

    Vector2 radD = { 0.f, 0.f };
    Vector2 radPerpD = { 0.f, 0.f };

    if (B) {
        velB = B->vel;
        angVelB = B->angVel;

        invMassB = B->invMass;
        invMOIB = B->invMOI;

        radB = collision1.pointB - B->pos;
        radPerpB = { -radB.y, radB.x };

        radD = collision2.pointB - B->pos;
        radPerpD = { -radD.y, radD.x };
    }

    float JV_1 = Vector2DotProduct(tangent * -1, velA) + (Vector2DotProduct(tangent * -1, radPerpA) * angVelA) + Vector2DotProduct(tangent, velB) + (Vector2DotProduct(tangent, radPerpB) * angVelB);
    float JV_2 = Vector2DotProduct(tangent * -1, velA) + (Vector2DotProduct(tangent * -1, radPerpC) * angVelA) + Vector2DotProduct(tangent, velB) + (Vector2DotProduct(tangent, radPerpD) * angVelB);

    float effMass1 = invMassA + (Vector2DotProduct(tangent * -1, radPerpA) * Vector2DotProduct(tangent * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(tangent, radPerpB) * Vector2DotProduct(tangent, radPerpB) * invMOIB);
    float effMass2 = invMassA + (Vector2DotProduct(tangent * -1, radPerpC) * Vector2DotProduct(tangent * -1, radPerpC) * invMOIA) + invMassB + (Vector2DotProduct(tangent, radPerpD) * Vector2DotProduct(tangent, radPerpD) * invMOIB);

    float tangentLambda1 = -JV_1 / effMass1;
    float tangentLambda2 = -JV_2 / effMass2;

    if (abs(tangentLambda1) < 0.01f && abs(tangentLambda2) < 0.01f) { return; }

    float maxFriction = abs(collision1.lambdaSum + collision2.lambdaSum) * friction * 0.5;

    float combinedSum1 = std::clamp(collision1.tangentLambdaSum + tangentLambda1, -maxFriction, maxFriction);
    float combinedSum2 = std::clamp(collision2.tangentLambdaSum + tangentLambda2, -maxFriction, maxFriction);

    tangentLambda1 = combinedSum1 - collision1.tangentLambdaSum;
    tangentLambda2 = combinedSum2 - collision2.tangentLambdaSum;

    collision1.tangentLambdaSum += tangentLambda1;
    collision2.tangentLambdaSum += tangentLambda2;

    Vector2 Ja = (tangent * -1) * invMassA * tangentLambda1 + (tangent * -1) * invMassA * tangentLambda2;
    float Jb = Vector2DotProduct(tangent * -1, radPerpA) * tangentLambda1 * invMOIA + Vector2DotProduct(tangent * -1, radPerpC) * tangentLambda2 * invMOIA;
    Vector2 Jc = tangent * invMassB * tangentLambda1 + tangent * invMassB * tangentLambda2;
    float Jd = Vector2DotProduct(tangent, radPerpB) * tangentLambda1 * invMOIB + Vector2DotProduct(tangent, radPerpD) * tangentLambda2 * invMOIB;

    A->vel += Ja;
    A->angVel += Jb;

    if (B) {
        B->vel += Jc;
        B->angVel += Jd;
    }
}

void Simulation::SolvePosition(Collision& collision) {
    RigidBody* A = collision.bodyA;
    RigidBody* B = collision.bodyB;

    Vector2 norm = collision.worldNormal;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;

    Vector2 radA = collision.pointA - A->pos;
    Vector2 radPerpA = { -radA.y, radA.x };

    float invMassB = 0.f;
    float invMOIB = 0.f;

    Vector2 radB = { 0.f, 0.f };
    Vector2 radPerpB = { 0.f, 0.f };

    if (B) {
        invMassB = B->invMass;
        invMOIB = B->invMOI;

        radB = collision.pointB - B->pos;
        radPerpB = { -radB.y, radB.x };
    }

    float effMass = invMassA + (Vector2DotProduct(norm * -1, radPerpA) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);
    
    float lambda = std::max(collision.depth - biasSlop, 0.f) * biasFactor/effMass;

    A->pos += (norm * -invMassA) * lambda;
    A->rot += Vector2DotProduct(norm * -1, radPerpA) * invMOIA * lambda;

    if (B) {
        B->pos += (norm * invMassB) * lambda;
        B->rot += Vector2DotProduct(norm, radPerpB) * invMOIB * lambda;
    }
}

void Simulation::SolveImpulse(Collision& collision) {
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

    float JV = Vector2DotProduct(norm * -1, velA) + (Vector2DotProduct(norm * -1, radPerpA) * angVelA) + Vector2DotProduct(norm, velB) + (Vector2DotProduct(norm, radPerpB) * angVelB);
    float effMass = invMassA + (Vector2DotProduct(norm * -1, radPerpA) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);

    float lambda = -JV / effMass;
    float newSum = std::max(collision.lambdaSum + lambda, 0.f);
    
    lambda = newSum - collision.lambdaSum;
    collision.lambdaSum += lambda;

    A->vel += (norm * -invMassA) * lambda;
    A->angVel += Vector2DotProduct(norm * -1, radPerpA) * invMOIA * lambda;

    if (B) {
        B->vel += (norm * invMassB) * lambda;
        B->angVel += Vector2DotProduct(norm, radPerpB) * invMOIB * lambda;
    }
}

void Simulation::SolveImpulsePair(Collision& collision1, Collision& collision2) {
    RigidBody* A = collision1.bodyA;
    RigidBody* B = collision1.bodyB;

    Vector2 norm = collision1.worldNormal;

    Vector2 velA = A->vel;
    float angVelA = A->angVel;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;

    Vector2 velB = { 0.f, 0.f };
    float angVelB = 0.f;

    float invMassB = 0.f;
    float invMOIB = 0.f;

    Vector2 radA = collision1.pointA - A->pos;
    Vector2 radPerpA = { -radA.y, radA.x };

    Vector2 radB = { 0.f, 0.f };
    Vector2 radPerpB = { 0.f, 0.f };

    Vector2 radC = collision2.pointA - A->pos;
    Vector2 radPerpC = { -radC.y, radC.x };

    Vector2 radD = { 0.f, 0.f };
    Vector2 radPerpD = { 0.f, 0.f };

    if (B) {
        velB = B->vel;
        angVelB = B->angVel;

        invMassB = B->invMass;
        invMOIB = B->invMOI;

        radB = collision1.pointB - B->pos;
        radPerpB = { -radB.y, radB.x };

        radD = collision2.pointB - B->pos;
        radPerpD = { -radD.y, radD.x };
    }
    
    float JV_1 = Vector2DotProduct(norm * -1, velA) + (Vector2DotProduct(norm * -1, radPerpA) * angVelA) + Vector2DotProduct(norm, velB) + (Vector2DotProduct(norm, radPerpB) * angVelB);
    float JV_2 = Vector2DotProduct(norm * -1, velA) + (Vector2DotProduct(norm * -1, radPerpC) * angVelA) + Vector2DotProduct(norm, velB) + (Vector2DotProduct(norm, radPerpD) * angVelB);

    float a = invMassA + (Vector2DotProduct(norm * -1, radPerpA) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);
    float b = invMassA + (Vector2DotProduct(norm * -1, radPerpC) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpD) * Vector2DotProduct(norm, radPerpB) * invMOIB);
    float c = invMassA + (Vector2DotProduct(norm * -1, radPerpA) * Vector2DotProduct(norm * -1, radPerpC) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpD) * invMOIB);
    float d = invMassA + (Vector2DotProduct(norm * -1, radPerpC) * Vector2DotProduct(norm * -1, radPerpC) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpD) * Vector2DotProduct(norm, radPerpD) * invMOIB);

    float determinant = a * d - b * c;

    a = a / determinant;
    b = b / determinant;
    c = c / determinant;
    d = d / determinant;

    float lambda1 = d * -JV_1 - b * -JV_2;
    float lambda2 = - c * -JV_1 + a * -JV_2;

    if (abs(lambda1) < 0.01f && abs(lambda2) < 0.01f) { return; }

    float combinedSum = std::max(collision1.lambdaSum + collision2.lambdaSum + lambda1 + lambda2, 0.f);

    lambda1 = combinedSum - collision1.lambdaSum - collision2.lambdaSum - lambda2;
    lambda2 = combinedSum - collision1.lambdaSum - collision2.lambdaSum - lambda1;

    collision1.lambdaSum += lambda1;
    collision2.lambdaSum += lambda2;

    Vector2 Ja = (norm * -1) * invMassA * lambda1 + (norm * -1) * invMassA * lambda2;
    float Jb = Vector2DotProduct(norm * -1, radPerpA) * lambda1 * invMOIA + Vector2DotProduct(norm * -1, radPerpC) * lambda2 * invMOIA;
    Vector2 Jc = norm * invMassB * lambda1 + norm * invMassB * lambda2;
    float Jd = Vector2DotProduct(norm, radPerpB) * lambda1 * invMOIB + Vector2DotProduct(norm, radPerpD) * lambda2 * invMOIB;

    A->vel += Ja;
    A->angVel += Jb;

    if (B) {
        B->vel += Jc;
        B->angVel += Jd;
    }
}

void Simulation::ApplyRestitution(Collision& collision) {
    Vector2 normal = collision.worldNormal;
    float lambda = collision.lambdaSum;

    RigidBody* A = collision.bodyA;
    RigidBody* B = collision.bodyB;

    Vector2 radA = collision.pointA - A->pos;
    Vector2 radPerpA = { -radA.y, radA.x };

    A->vel += normal * -1 * lambda * elasticity * A->invMass;
    A->angVel += Vector2DotProduct(normal * -1, radPerpA) * lambda * elasticity * A->invMOI;

    if (B) {
        Vector2 radB = collision.pointB - B->pos;
        Vector2 radPerpB = { -radB.y, radB.x };

        B->vel += normal * lambda * elasticity * B->invMass;
        B->angVel += Vector2DotProduct(normal, radPerpB) * lambda * elasticity * B->invMOI;
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


void Simulation::SolveJointPosition(Joint& joint) {
    RigidBody* A = joint.bodyA;
    RigidBody* B = joint.bodyB;

    //Vector2 norm = joint.worldNormal;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;

    Vector2 radA = Vector2Rotate(joint.localA, A->rot);
    Vector2 radPerpA = { -radA.y, radA.x };

    float invMassB = B->invMass;
    float invMOIB = B->invMOI;

    Vector2 radB = Vector2Rotate(joint.localB, B->rot);
    Vector2 radPerpB = { -radB.y, radB.x };

    Vector2 pointA = A->pos + radA;
    Vector2 pointB = B->pos + radB;

    Vector2 AtoB = pointA - pointB;
    //float depth = Vector2Length(AtoB);
    //Vector2 norm = Vector2Normalize(B->pos - A->pos);

    //float bias = std::max(depth - biasSlop, 0.f) * (biasFactor / DeltaTime);
    float a = invMassA + invMassB + (radA.y * radA.y * invMOIA) + (radB.y * radB.y * invMOIB);
    float b = -(radA.x * radA.y * invMOIA) - (radB.x * radB.y * invMOIB);
    float c = -(radA.x * radA.y * invMOIA) - (radB.x * radB.y * invMOIB);
    float d = invMassA + invMassB + (radA.x * radA.x * invMOIA) + (radB.x * radB.x * invMOIB);

    float determinant = a * d - b * c;

    a = a / determinant;
    b = b / determinant;
    c = c / determinant;
    d = d / determinant;

    float x = d * AtoB.x - b * AtoB.y;
    float y = -c * AtoB.x + a * AtoB.y;
    
    Vector2 impulse = Vector2{ x, y } * biasFactor;


    //float effMass = invMassA + (Vector2DotProduct(AtoB, radPerpA) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);

    //float lambda = -std::max(depth - biasSlop, 0.f) * biasFactor / effMass;
    //Vector2 lambda = AtoB * biasFactor / effMass;

    A->pos += impulse * -invMassA;
    A->rot += Vector2DotProduct(impulse * -1, radPerpA) * invMOIA;

    B->pos += (impulse * invMassB);
    B->rot += Vector2DotProduct(impulse, radPerpB) * invMOIB;
}

void Simulation::SolveJoint(Joint& joint) {
    RigidBody* A = joint.bodyA;
    RigidBody* B = joint.bodyB;

    Vector2 velA = A->vel;
    float angVelA = A->angVel;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;

    Vector2 radA = Vector2Rotate(joint.localA, A->rot);
    Vector2 radPerpA = { -radA.y, radA.x };

    Vector2 velB = B->vel;
    float angVelB = B->angVel;

    float invMassB = B->invMass;
    float invMOIB = B->invMOI;

    Vector2 radB = Vector2Rotate(joint.localB, B->rot);
    Vector2 radPerpB = { -radB.y, radB.x };

    Vector2 pointA = A->pos + radA;
    Vector2 pointB = B->pos + radB;

    Vector2 AtoB = pointB - pointA;
    //float depth = Vector2Length(AtoB);
    //Vector2 norm = Vector2Normalize(AtoB);

    //Vector2 bias = AtoB * (-biasFactor / DeltaTime);

    Vector2 relVel = (velA * -1) + (radPerpA * -angVelA) + velB + (radPerpB * angVelB);
    //float effMass = invMassA + (Vector2DotProduct(radPerpA, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(radPerpB, radPerpB) * invMOIB);

    relVel = relVel * -1;
    //relVel += bias;

    float a = invMassA + invMassB + (radA.y * radA.y * invMOIA) + (radB.y * radB.y * invMOIB);
    float b = -(radA.x * radA.y * invMOIA) - (radB.x * radB.y * invMOIB);
    float c = -(radA.x * radA.y * invMOIA) - (radB.x * radB.y * invMOIB);
    float d = invMassA + invMassB + (radA.x * radA.x * invMOIA) + (radB.x * radB.x * invMOIB);

    float determinant = a * d - b * c;

    a = a / determinant;
    b = b / determinant;
    c = c / determinant;
    d = d / determinant;

    float x = d * relVel.x - b * relVel.y;
    float y = -c * relVel.x + a * relVel.y;

    Vector2 impulse = { x, y };

    PrintVectorText(impulse);

    A->vel += (impulse * -invMassA);
    A->angVel += Vector2DotProduct(impulse * -1, radPerpA) * invMOIA;

    B->vel += (impulse * invMassB);
    B->angVel += Vector2DotProduct(impulse * 1, radPerpB) * invMOIB;
}

void Simulation::ApplyTorque(Joint& joint, float torque) {
    RigidBody* A = joint.bodyA;
    RigidBody* B = joint.bodyB;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;

    Vector2 radA = Vector2Rotate(joint.localA, A->rot);
    Vector2 radPerpA = { -radA.y, radA.x };

    float invMassB = B->invMass;
    float invMOIB = B->invMOI;

    Vector2 radB = Vector2Rotate(joint.localB, B->rot);
    Vector2 radPerpB = { -radB.y, radB.x };

    Vector2 pointA = A->pos + radA;
    Vector2 pointB = B->pos + radB;

    //Vector2 AtoB = pointB - pointA;
    //float depth = Vector2Length(AtoB);
    //Vector2 norm = Vector2Normalize(AtoB);

    //float force = torque;

    float sqrLengthA = Vector2LengthSqr(radPerpA);
    Vector2 radPerpScaledA = radPerpA / sqrLengthA;

    A->acc += radPerpScaledA * torque * invMassA;
    A->angAcc += invMOIA * torque;

    float sqrLengthB = Vector2LengthSqr(radPerpB);
    Vector2 radPerpScaledB = radPerpB / sqrLengthB;
    
    B->acc += radPerpScaledB * -torque * invMassB;
    B->angAcc += invMOIB * -torque;
    

    //A->angAcc += std::max(Vector2Length(radPerpA), 1.f) * invMOIA * -torque;
    //B->angAcc += std::max(Vector2Length(radPerpB), 1.f) * invMOIB * torque;
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
            ApplyTorque(joints[i], -1000000);
        }
    }
    if (IsKeyDown(KEY_LEFT)) {
        for (int i = 0; i < jointCount; i++) {
            ApplyTorque(joints[i], 1000000);
        }
    }

    // Solve joint positions
    for (int i = 0; i < solverIterations; i++) {
        for (int n = 0; n < jointCount; n++) {
            SolveJointPosition(joints[n]);
        }
    }

    // Solve joint velocity
    for (int i = 0; i < solverIterations; i++) {
        for (int n = 0; n < jointCount; n++) {
            SolveJoint(joints[n]);
        }
    }

    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* A = bodies[i];

        // Boundary collision check
        CheckBoundaryCollision(A);

        // RigidBody collision check
        for (int n = i + 1; n < rigidBodyCount; n++) {
            RigidBody* B = bodies[n];

            if (A->jointBody == B || B->jointBody == A) {
                continue;
            }

            CheckCollision(A, B);
        }
    }
    
    // Sequential Position Solver
    for (int i = 0; i < solverIterations; i++) {
        for (int n = 0; n < collisionCount; n++) {
            SolvePosition(collisions[n]);
        }
    }

    // Sequential Impulse Solver
    for (int i = 0; i < solverIterations; i++) {
        // Apply Friction Impulses
        for (int n = 0; n < collisionCount; n++) {
            if (n + 1 < collisionCount) {
                if (collisions[n].bodyA == collisions[n + 1].bodyA && collisions[n].bodyB == collisions[n + 1].bodyB && collisions[n].worldNormal == collisions[n + 1].worldNormal) {
                    SolveFrictionPair(collisions[n], collisions[n + 1]);
                    n++;
                    continue;
                }
            }

            SolveFriction(collisions[n]);
        }
        // Apply Normal Impulses
        for (int n = 0; n < collisionCount; n++) {
            if (n + 1 < collisionCount) {
                if (collisions[n].bodyA == collisions[n + 1].bodyA && collisions[n].bodyB == collisions[n + 1].bodyB && collisions[n].worldNormal == collisions[n + 1].worldNormal) {
                    SolveImpulsePair(collisions[n], collisions[n + 1]);
                    //SolveFrictionPair(collisions[n], collisions[n + 1]);
                    n++;
                    continue;
                }
            }

            SolveImpulse(collisions[n]);
            //SolveFriction(collisions[n]);
        }
    }

    // Apply Elastic Impulses
    for (int i = 0; i < collisionCount; i++) {
        ApplyRestitution(collisions[i]);
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
}