#include "Simulation.h"

#include "rlshort.h"


Simulation::Simulation() {
    boundary = { 0.f, 0.f, 1800.f, 900.f };
    gravity = { 0.f, 200.f };

    rigidBodyCount = 0;
    staticBodyCount = 0;

    circles[0] = RigidCircle({ 650.f, 100.f }, 25.f, 10.f);
    rigidBodyCount++;

    rects[0] = StaticRect({ 700.f, 400.f }, 300.f, 50.f, 45.f);
    staticBodyCount++;

    rects[1] = StaticRect({ 1100.f, 600.f }, 300.f, 50.f, -45.f);
    staticBodyCount++;
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
            Vector2 collisionNormal;

            StaticRect& rect = rects[n];

            bool isColliding = rect.CheckCollision(circle, collisionNormal);

            if (!isColliding) {
                continue;
            }

            Vector2 toCircle = circle.GetPos() - rect.GetPos();
            toCircle = Vector2Rotate(toCircle, -rect.GetRotation() * PI / 180);

            if (collisionNormal.x == 0) {
                toCircle.y = (circle.GetRadius() + rect.GetHeight() / 2) * collisionNormal.y;
            }
            else if (collisionNormal.y == 0) {
                toCircle.x = (circle.GetRadius() + rect.GetWidth() / 2) * collisionNormal.x;
            }
            else {
                Vector2 corner = { rect.width / 2, rect.height / 2 };

                if (toCircle.x < 0) { corner.x *= -1; }
                if (toCircle.y < 0) { corner.y *= -1; }

                toCircle = corner + collisionNormal * circle.radius;
            }

            toCircle = Vector2Rotate(toCircle, rect.GetRotation() * PI / 180);
            circle.pos = rect.pos + toCircle;

            Vector2 worldNormal = Vector2Rotate(collisionNormal, rect.GetRotation() * PI / 180);
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
        rects[i].Draw();
    }

    DrawVectorText(circles[0].pos, 20, 50, 20, BLUE);
}