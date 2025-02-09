#include "Simulation.h"

#include <cmath>
#include <iostream>
#include <string>

#include "rlshort.h"

#include "CollisionInfo.h"

Simulation::Simulation() {
    boundary = { 0.f, 0.f, 1800.f, 900.f };
    gravity = { 0.f, 100.f };

    rigidBodyCount = 0;
    staticBodyCount = 0;

    //bodies[0] = new RigidCircle({ 900.f, 100.f }, 10.f, 25.f);
    //rigidBodyCount++;

    bodies[0] = new RigidRect({ 1100.f, 200.f }, 50.f, 200.f, 50.f, PI/6);
    rigidBodyCount++;

    statics[0] = new StaticRect({ 700.f, 400.f }, 300.f, 50.f, PI/4);
    staticBodyCount++;

    statics[1] = new StaticRect({ 1100.f, 600.f }, 300.f, 50.f, -PI/4);
    staticBodyCount++;

    //statics[2] = new StaticCircle({ 950.f, 400.f }, 50.f);
    //staticBodyCount++;
}

Simulation::~Simulation() {
    for (int i = 0; i < rigidBodyCount; i++) {
        delete bodies[i];
    }

    for (int i = 0; i < staticBodyCount; i++) {
        delete statics[i];
    }
}

bool Simulation::CheckBoundaryCollision(RigidBody* body, CollisionInfo& result) {
    RigidCircle* circle = dynamic_cast<RigidCircle*>(body);
    if (circle) {
        if (circle->pos.y > boundary.w - circle->radius) {
            result.normal = { 0, -1.f };
            result.depth = circle->pos.y + circle->radius - boundary.w;
            return true;
        }
        else if (circle->pos.y < boundary.y + circle->radius) {
            result.normal = { 0, 1.f };
            result.depth = -circle->pos.y + circle->radius + boundary.y;
            return true;
        }
        else if (circle->pos.x < boundary.x + circle->radius) {
            result.normal = { 1.f, 0.f };
            result.depth = -circle->pos.x + circle->radius + boundary.x;
            return true;
        }
        else if (circle->pos.x > boundary.z - circle->radius) {
            result.normal = { -1.f, 0.f };
            result.depth = circle->pos.x + circle->radius - boundary.z;
            return true;
        }

        return false;
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
                result.normal = { 0.f, -1.f };
                result.depth = cornerPos.y - boundary.w;
                result.impact = cornerPos + result.normal * abs(result.depth);
                return true;
            }
            else if (cornerPos.y < boundary.y) {
                result.normal = { 0.f, 1.f };
                result.depth = -cornerPos.y + boundary.y;
                result.impact = cornerPos + result.normal * abs(result.depth);
                return true;
            }
            else if (cornerPos.x < boundary.x) {
                result.normal = { 1.f, 0.f };
                result.depth = -cornerPos.x + boundary.x;
                result.impact = cornerPos + result.normal * abs(result.depth);
                return true;
            }
            else if (cornerPos.x > boundary.z) {
                result.normal = { -1.f, 0.f };
                result.depth = cornerPos.x - boundary.z;
                result.impact = cornerPos + result.normal * abs(result.depth);
                return true;
            }
        }

        return false;
    }

    return false;
}

void Simulation::InitialStep(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* body = bodies[i];

        body->acc += gravity;
        
        body->pos += body->vel * DeltaTime + body->acc * (DeltaTime * DeltaTime * 0.5);
        body->vel += body->acc * DeltaTime; //This assumes acceleration is the same next tick.
        
        body->rot += body->angVel * DeltaTime + body->angAcc * (DeltaTime * DeltaTime * 0.5);
        body->angVel += body->angAcc * DeltaTime;
    }
}

void resolveCollision(RigidBody* A, RigidBody* B) {
    Vector2 velA = A->vel;
    float angVelA = A->angVel;
    Vector2 velB = B->vel;
    float angVelB = B->angVel;

    float invMassA = A->invMass;
    float invMOIA = A->invMOI;
    float invMassB = B->invMass;
    float invMOIB = B->invMOI;

    Vector2 norm;
    Vector2 radA = { 1, 0 };
    Vector2 radPerpA = { -radA.y, radA.x };
    Vector2 radB = { -1, 0 };
    Vector2 radPerpB = { -radB.y, radB.x };

    float JV = Vector2DotProduct(norm * -1, velA) + (Vector2DotProduct(norm, radPerpA) * -angVelA) + Vector2DotProduct(norm, velB) + (Vector2DotProduct(norm, radPerpB) * angVelB);
    float effMass = invMassA + (Vector2DotProduct(norm, radPerpA) * Vector2DotProduct(norm, radPerpA) * invMOIA) + invMassB + (Vector2DotProduct(norm, radPerpB) * Vector2DotProduct(norm, radPerpB) * invMOIB);

    float lambda = -JV / effMass;

    Vector2 deltaVelA = (norm * -invMassA) * lambda;
    float deltaAngVelA = Vector2DotProduct(norm, radPerpA) * -invMOIA * lambda;
    Vector2 deltaVelB = (norm * invMassB) * lambda;
    float deltaAngVelB = Vector2DotProduct(norm, radPerpA) * invMOIB * lambda;
}

void Simulation::Step(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* body = bodies[i];

        body->vel += body->acc * DeltaTime * 0.5;
        body->pos += body->vel * DeltaTime;
        body->acc = { 0.f, 0.f };

        body->angVel += body->angAcc * DeltaTime * 0.5;
        body->rot += body->angVel * DeltaTime;
        body->angAcc = 0.f;

        body->acc += gravity;

        CollisionInfo boundaryCollision;
        if (CheckBoundaryCollision(body, boundaryCollision)) {
            Vector2 radial = boundaryCollision.impact - body->pos;
            Vector2 radialPerp = { -radial.y, radial.x };

            Vector2 impactVel = body->vel + radialPerp * body->angVel;

            float relNormVel = Vector2DotProduct(impactVel, boundaryCollision.normal);
            float biasVel = boundaryCollision.depth * (0.1f / DeltaTime) * (relNormVel / abs(relNormVel));
            float radialPerpNorm = Vector2DotProduct(radialPerp, boundaryCollision.normal);

            float impulseMag = (-1.f * (relNormVel + biasVel)) / (body->invMass + (radialPerpNorm * radialPerpNorm * body->invMOI));
            Vector2 impulse = boundaryCollision.normal * impulseMag;
            float angularImpulse = Vector2DotProduct(radialPerp, impulse);

            body->acc += impulse * body->invMass * (1 / DeltaTime);
            body->angAcc += angularImpulse * body->invMOI * (1 / DeltaTime);
            
            std::cout << "collision happened" << std::endl;
        }

        for (int n = 0; n < staticBodyCount; n++) {
            StaticBody* staticBody = statics[n];

            CollisionInfo collision;
            if (!staticBody->CheckCollision(body, collision)) { continue; }

            Vector2 worldNormal = Vector2Rotate(collision.normal, staticBody->GetRotation());
            
            Vector2 radial = collision.impact - body->pos;
            Vector2 radialPerp = { -radial.y, radial.x };

            Vector2 impactVel = body->vel + radialPerp * body->angVel;

            float relNormVel = Vector2DotProduct(impactVel, worldNormal);
            float biasVel = collision.depth * (0.4f / DeltaTime) * (relNormVel/abs(relNormVel));
            float radialPerpNorm = Vector2DotProduct(radialPerp, worldNormal);

            float impulseMag = (-1.f * (relNormVel + biasVel)) / (body->invMass + (radialPerpNorm * radialPerpNorm * body->invMOI));
            Vector2 impulse = worldNormal * impulseMag;
            float angularImpulse = Vector2DotProduct(radialPerp, impulse);

            body->acc += impulse * body->invMass * (1 / DeltaTime);
            body->angAcc += angularImpulse * body->invMOI * (1 / DeltaTime);

            std::cout << "collision happened" << std::endl;
        }

        body->vel = body->vel + (body->acc * 0.5 * DeltaTime);
        body->angVel = body->angVel + (body->angAcc * 0.5 * DeltaTime);
    }
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