#include "Simulation.h"

#include <cmath>
#include <iostream>

#include "rlshort.h"

#include "CollisionInfo.h"

Simulation::Simulation() {
    boundary = { 0.f, 0.f, 1800.f, 900.f };
    gravity = { 0.f, 200.f };

    rigidBodyCount = 0;
    staticBodyCount = 0;

    //bodies[0] = new RigidCircle({ 900.f, 100.f }, 10.f, 25.f);
    //rigidBodyCount++;

    bodies[0] = new RigidRect({ 900.f, 400.f }, 10.f, 100.f, 50.f, 30.f);
    rigidBodyCount++;

    //statics[0] = new StaticRect({ 700.f, 400.f }, 300.f, 50.f, 45.f);
    //staticBodyCount++;

    //statics[1] = new StaticRect({ 1100.f, 600.f }, 300.f, 50.f, -45.f);
    //staticBodyCount++;

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
            
            return true;
        }
        else if (circle->pos.y < boundary.y + circle->radius) {
            result.normal = { 0, 1.f };

            return true;
        }
        else if (circle->pos.x < boundary.x + circle->radius) {
            result.normal = { 1.f, 0.f };

            return true;
        }
        else if (circle->pos.x > boundary.z - circle->radius) {
            result.normal = { -1.f, 0.f };

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
            Vector2 cornerPos = rect->pos + Vector2Rotate(corners[i], rect->rotation * PI / 180);

            if (Vector2DotProduct(cornerPos - Vector2{ 0, boundary.w }, { 0.f, -1.f }) < 0.f) {
                result.normal = { 0.f, -1.f };
                result.impact = cornerPos;

                rect->pos += result.normal * abs(cornerPos.y - boundary.w);

                return true;
            }
            else if (Vector2DotProduct(cornerPos - Vector2{ 0, 0 }, { 0.f, 1.f }) < 0.f) {
                result.normal = { 0.f, 1.f };
                result.impact = cornerPos;
                
                rect->pos += result.normal * abs(cornerPos.y);

                return true;
            }
            else if (Vector2DotProduct(cornerPos - Vector2{ 0, 0 }, { 1.f, 0.f }) < 0.f) {
                result.normal = { 1.f, 0.f };
                result.impact = cornerPos;
                
                rect->pos += result.normal * abs(cornerPos.x);

                return true;
            }
            else if (Vector2DotProduct(cornerPos - Vector2{ boundary.z, 0 }, { -1.f, 0.f }) < 0.f) {
                result.normal = { -1.f, 0.f };
                result.impact = cornerPos;
                
                rect->pos += result.normal * abs(cornerPos.x - boundary.z);

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

        body->pos += gravity * (DeltaTime * DeltaTime * 0.5);
        body->vel += gravity * DeltaTime; //This assumes acceleration is the same next tick.
    }
}

void Simulation::Step(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidBody* body = bodies[i];

        Vector2 velHalfStep = body->vel + gravity * DeltaTime * 0.5;
        body->pos += velHalfStep * DeltaTime;

        float angularVelHalfStep = body->angularVel;
        body->rotation += angularVelHalfStep * DeltaTime;

        Vector2 acceleration = { 0.f, 0.f };
        float angularAcceleration = 0.f;

        CollisionInfo boundaryCollision;
        if (CheckBoundaryCollision(body, boundaryCollision)) {
            Vector2 velocityNormal = boundaryCollision.normal * Vector2DotProduct(velHalfStep, boundaryCollision.normal);
            //acceleration += (velocityNormal * -2) * (2 / DeltaTime);

            Vector2 force = (velocityNormal * -2) * (2 / DeltaTime) * body->mass;

            Vector2 radial = boundaryCollision.impact - body->pos;
            Vector2 radialUnit = Vector2Normalize(radial);

            Vector2 forceLinear = radialUnit * Vector2DotProduct(force, radialUnit);
            Vector2 forceTangent = force - forceLinear;

            std::cout << "collision happened" << std::endl;

            float torque = Vector2CrossProduct(radial, forceTangent);

            acceleration += forceLinear * (1 / body->mass);
            angularAcceleration += torque * (1 / body->rotationalInertia);
        }

        for (int n = 0; n < staticBodyCount; n++) {
            StaticBody* staticBody = statics[n];

            CollisionInfo collision;
            if (!staticBody->CheckCollision(body, collision)) {
                continue;
            }
            
            Vector2 worldNormal = Vector2Rotate(collision.normal, staticBody->GetRotation() * PI / 180);
            Vector2 velocityNormal = worldNormal * Vector2DotProduct(velHalfStep, worldNormal);
            acceleration += (velocityNormal * -2) * (2 / DeltaTime);
        }

        body->vel = velHalfStep + acceleration * 0.5 * DeltaTime;
        body->angularVel = angularVelHalfStep + angularAcceleration * 0.5 * DeltaTime;

        //DrawVectorText(body->vel, 20, 100, 20, RED);
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
}