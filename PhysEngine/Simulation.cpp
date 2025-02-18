#include "Simulation.h"

#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>

#include "rlshort.h"

#include "Collision.h"


Simulation::Simulation() {
    bodies[0] = new RigidRect({ 1100.f, 200.f }, 500.f, 200.f, 50.f, -PI / 3);
    rigidBodyCount++;

    bodies[1] = new RigidCircle({ 700.f, 100.f }, 500.f, 50.f);
    rigidBodyCount++;

    bodies[2] = new RigidRect({ 700.f, 400.f }, 0.f, 300.f, 50.f, PI / 6);
    //bodies[2]->SetStatic();
    rigidBodyCount++;

    bodies[3] = new RigidRect({ 1000.f, 600.f }, 0.f, 400.f, 50.f, -PI / 4);
    //bodies[3]->SetStatic();
    rigidBodyCount++;

    bodies[4] = new RigidRect({ 800.f, 200.f }, 300.f, 100.f, 100.f, PI/3);
    rigidBodyCount++;

    bodies[5] = new RigidCircle({ 900.f, 300.f }, 0.f, 50.f);
    //bodies[5]->SetStatic();
    rigidBodyCount++;
}

Simulation::~Simulation() {
    for (int i = 0; i < rigidBodyCount; i++) {
        delete bodies[i];
    }
}

void Simulation::AddRigidBody(RigidBody* newBody) {
    bodies[rigidBodyCount] = newBody;
    rigidBodyCount++;
}

void Simulation::CheckBoundaryCollision(RigidBody* body) {
    if (body->isStatic) { return; }

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
    collision.worldTangent = { -norm.y, norm.x };
    Vector2 tangent = collision.worldTangent;

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

    float oldSum = collision.tangentLambdaSum;
    collision.tangentLambdaSum = std::clamp(collision.tangentLambdaSum + tangentLambda, -collision.lambdaSum * friction, collision.lambdaSum * friction);

    tangentLambda = collision.tangentLambdaSum - oldSum;

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
    collision1.worldTangent = tangent;
    collision2.worldTangent = tangent;

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

    if (abs(tangentLambda1) < 0.01f && abs(tangentLambda2) < 0.01f) {
        return;
    }

    std::cout << "unclamped: " << tangentLambda1 << ", " << tangentLambda2 << std::endl;

    float maxFriction = abs(collision1.lambdaSum + collision2.lambdaSum) * friction * 0.5;

    float combinedSum1 = std::clamp(collision1.tangentLambdaSum + tangentLambda1, -maxFriction, maxFriction);
    float combinedSum2 = std::clamp(collision2.tangentLambdaSum + tangentLambda2, -maxFriction, maxFriction);

    tangentLambda1 = combinedSum1 - collision1.tangentLambdaSum;
    tangentLambda2 = combinedSum2 - collision2.tangentLambdaSum;

    collision1.tangentLambdaSum += tangentLambda1;
    collision2.tangentLambdaSum += tangentLambda2;

    std::cout << "clamped: " << tangentLambda1 << ", " << tangentLambda2 << std::endl;

    Vector2 Ja = (tangent * -1) * invMassA * tangentLambda1 + (tangent * -1) * invMassA * tangentLambda2;
    float Jb = Vector2DotProduct(tangent * -1, radPerpA) * tangentLambda1 * invMOIA + Vector2DotProduct(tangent * -1, radPerpC) * tangentLambda2 * invMOIA;
    Vector2 Jc = tangent * invMassB * tangentLambda1 + tangent * invMassB * tangentLambda2;
    float Jd = Vector2DotProduct(tangent, radPerpB) * tangentLambda1 * invMOIB + Vector2DotProduct(tangent, radPerpD) * tangentLambda2 * invMOIB;

    PrintVectorText(Ja);
    std::cout << Jb << std::endl;

    A->vel += Ja;
    A->angVel += Jb;

    if (B) {
        B->vel += Jc;
        B->angVel += Jd;
    }
}

void Simulation::ResolveCollision(Collision& collision, float DeltaTime) {
    RigidBody* A = collision.bodyA;
    RigidBody* B = collision.bodyB;
    
    Vector2 norm = collision.worldNormal;
    collision.worldTangent = { -norm.y, norm.x };
    Vector2 tangent = collision.worldTangent;

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

    float bias = -std::max(collision.depth - biasSlop, 0.f) * (biasFactor/DeltaTime);

    float JV = Vector2DotProduct(norm * -1, velA) + (Vector2DotProduct(norm * -1, radPerpA) * angVelA) + Vector2DotProduct(norm, velB) + (Vector2DotProduct(norm, radPerpB) * angVelB);
    float effMass = invMassA + (Vector2DotProduct(norm * -1, radPerpA) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);

    float lambda = -(JV + bias) / effMass;

    float oldSum = collision.lambdaSum;
    collision.lambdaSum = std::max(collision.lambdaSum + lambda, 0.f);
    
    lambda = collision.lambdaSum - oldSum;
    
    A->vel += (norm * -invMassA) * lambda;
    A->angVel += Vector2DotProduct(norm * -1, radPerpA) * invMOIA * lambda;

    if (B) {
        B->vel += (norm * invMassB) * lambda;
        B->angVel += Vector2DotProduct(norm, radPerpB) * invMOIB * lambda;
    }
}

void Simulation::ResolveCollisionPair(Collision& collision1, Collision& collision2, float DeltaTime) {
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

    float bias_1 = -std::max(collision1.depth - biasSlop, 0.f) * (biasFactor / DeltaTime);
    float bias_2 = -std::max(collision2.depth - biasSlop, 0.f) * (biasFactor / DeltaTime);

    float a = invMassA + (Vector2DotProduct(norm * -1, radPerpA) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);
    float b = invMassA + (Vector2DotProduct(norm * -1, radPerpC) * Vector2DotProduct(norm * -1, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpD) * Vector2DotProduct(norm, radPerpB) * invMOIB);
    float c = invMassA + (Vector2DotProduct(norm * -1, radPerpA) * Vector2DotProduct(norm * -1, radPerpC) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpD) * invMOIB);
    float d = invMassA + (Vector2DotProduct(norm * -1, radPerpC) * Vector2DotProduct(norm * -1, radPerpC) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpD) * Vector2DotProduct(norm, radPerpD) * invMOIB);

    float determinant = a * d - b * c;

    a = a / determinant;
    b = b / determinant;
    c = c / determinant;
    d = d / determinant;

    float lambda1 = d * (-JV_1 - bias_1) - b * (-JV_2 - bias_2);
    float lambda2 = - c * (-JV_1 - bias_1) + a * (-JV_2 - bias_2);

    if (abs(lambda1) < 0.01f && abs(lambda2) < 0.01f) {
        return;
    }

    //std::cout << "unclamped: " << lambda1 << ", " << lambda2 << std::endl;

    float combinedSum = std::max(collision1.lambdaSum + collision2.lambdaSum + lambda1 + lambda2, 0.f);

    lambda1 = combinedSum - collision1.lambdaSum - collision2.lambdaSum - lambda2;
    lambda2 = combinedSum - collision1.lambdaSum - collision2.lambdaSum - lambda1;

    collision1.lambdaSum += lambda1;
    collision2.lambdaSum += lambda2;

    //std::cout << "clamped: " << lambda1 << ", " << lambda2 << std::endl;
    
    Vector2 Ja = (norm * -1) * invMassA * lambda1 + (norm * -1) * invMassA * lambda2;
    float Jb = Vector2DotProduct(norm * -1, radPerpA) * lambda1 * invMOIA + Vector2DotProduct(norm * -1, radPerpC) * lambda2 * invMOIA;
    Vector2 Jc = norm * invMassB * lambda1 + norm * invMassB * lambda2;
    float Jd = Vector2DotProduct(norm, radPerpB) * lambda1 * invMOIB + Vector2DotProduct(norm, radPerpD) * lambda2 * invMOIB;

    //PrintVectorText(Ja);
    //std::cout << Jb << std::endl;

    A->vel += Ja;
    A->angVel += Jb;

    if (B) {
        B->vel += Jc;
        B->angVel += Jd;
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
    int iterations = 4;
    for (int i = 0; i < iterations; i++) {
        for (int n = 0; n < collisionCount; n++) {
            if (n + 1 < collisionCount) {
                if (collisions[n].bodyA == collisions[n + 1].bodyA && collisions[n].bodyB == collisions[n + 1].bodyB && collisions[n].worldNormal == collisions[n + 1].worldNormal) {
                    //std::cout << collisions[n].bodyA->angVel << std::endl;
                    ResolveCollisionPair(collisions[n], collisions[n + 1], DeltaTime);
                    //SolveFrictionPair(collisions[n], collisions[n + 1]);

                    //if (i == 3) { std::cout << "------------" << std::endl; }

                    n++;
                    continue;
                }
            }

            ResolveCollision(collisions[n], DeltaTime);
            //SolveFriction(collisions[n]);
        }
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
    }

    for (int i = 0; i < collisionCount; i++) {
        Vector2 normal = collisions[i].worldNormal;
        float lambda = collisions[i].lambdaSum;

        RigidBody* A = collisions[i].bodyA;
        RigidBody* B = collisions[i].bodyB;

        Vector2 radA = collisions[i].pointA - A->pos;
        Vector2 radPerpA = { -radA.y, radA.x };

        A->vel += normal * -1 * lambda * elasticity * A->invMass;
        A->angVel += Vector2DotProduct(normal * -1, radPerpA) * lambda * elasticity * A->invMOI;

        if (B) {
            Vector2 radB = collisions[i].pointB - B->pos;
            Vector2 radPerpB = { -radB.y, radB.x };

            B->vel += normal * lambda * elasticity * B->invMass;
            B->angVel += Vector2DotProduct(normal, radPerpB) * lambda * elasticity * B->invMOI;
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
}

void Simulation::Draw() {
    for (int i = 0; i < rigidBodyCount; i++) {
        bodies[i]->Draw();
    }

    DrawVectorText(bodies[0]->pos, 20, 50, 20, BLUE);
    DrawVectorText(bodies[0]->vel, 20, 100, 20, BLUE);

    DrawText(std::to_string(bodies[0]->angVel).c_str(), 20, 150, 20, BLUE);
}