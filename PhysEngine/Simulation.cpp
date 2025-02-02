#include "Simulation.h"

#include "rlshort.h"

#include "CollisionInfo.h"

Simulation::Simulation() {
    boundary = { 0.f, 0.f, 1800.f, 900.f };
    gravity = { 0.f, 200.f };

    rigidBodyCount = 0;
    staticBodyCount = 0;

    circles[0] = RigidCircle({ 900.f, 100.f }, 25.f, 10.f);
    rigidBodyCount++;

    statics[0] = new StaticRect({ 700.f, 400.f }, 300.f, 50.f, 45.f);
    staticBodyCount++;

    statics[1] = new StaticRect({ 1100.f, 600.f }, 300.f, 50.f, -45.f);
    staticBodyCount++;

    statics[2] = new StaticCircle({ 950.f, 400.f }, 50.f);
    staticBodyCount++;
}

Simulation::~Simulation() {
    for (int i = 0; i < staticBodyCount; i++) {
        delete statics[i];
    }
}

void Simulation::InitialStep(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidCircle& circle = circles[i];

        circle.pos += gravity * (DeltaTime * DeltaTime * 0.5);
        circle.vel += gravity * DeltaTime; //This assumes acceleration is the same next tick.
    }
}

void Simulation::Step(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidCircle& circle = circles[i];

        Vector2 velHalfStep = circle.vel + gravity * DeltaTime * 0.5;
        circle.pos += velHalfStep * DeltaTime;

        Vector2 acceleration = { 0.f, 0.f };

        if (circle.pos.y > boundary.w - circle.radius) {
            circle.pos.y = boundary.w - circle.radius;
            
            acceleration += Vector2{ 0.f, -velHalfStep.y * 2 } * (2 / DeltaTime);
        }
        else if (circle.pos.y < boundary.y + circle.radius) {
            circle.pos.y = boundary.y + circle.radius;

            acceleration += Vector2{ 0.f, -velHalfStep.y * 2 } * (2 / DeltaTime);
        }
        else if (circle.pos.x < boundary.x + circle.radius) {
            circle.pos.x = boundary.x + circle.radius;

            acceleration += Vector2{ -velHalfStep.x * 2, 0.f } * (2 / DeltaTime);
        }
        else if (circle.pos.x > boundary.z - circle.radius) {
            circle.pos.x = boundary.z - circle.radius;

            acceleration += Vector2{ -velHalfStep.x * 2, 0.f } * (2 / DeltaTime);
        }

        for (int n = 0; n < staticBodyCount; n++) {
            CollisionInfo collision;

            StaticBody* staticBody = statics[n];

            if (!staticBody->CheckCollision(circle, collision)) {
                continue;
            }

            circle.pos = collision.impact;

            Vector2 worldNormal = Vector2Rotate(collision.normal, staticBody->GetRotation() * PI / 180);
            Vector2 velocityNormal = worldNormal * Vector2DotProduct(velHalfStep, worldNormal);
            acceleration += (velocityNormal * -2) * (2 / DeltaTime);
        }

        circle.vel = velHalfStep + acceleration * 0.5 * DeltaTime;

        DrawVectorText(circle.vel, 20, 100, 20, RED);
    }
}

void Simulation::Draw() {
    for (int i = 0; i < rigidBodyCount; i++) {
        circles[i].Draw();
    }

    for (int i = 0; i < staticBodyCount; i++) {
        statics[i]->Draw();
    }

    DrawVectorText(circles[0].pos, 20, 50, 20, BLUE);
}