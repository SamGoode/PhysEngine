#include "Simulation.h"

#include "rlshort.h"


Simulation::Simulation() {
    boundary = { 0.f, 0.f, 1800.f, 900.f };
    gravity = { 0.f, 200.f };

    rigidBodyCount = 0;
    staticBodyCount = 0;

    circles[0] = RigidCircle({ 700.f, 100.f }, 25.f, 10.f);
    rigidBodyCount++;

    rects[0] = StaticRect({ 700.f, 400.f }, 300.f, 50.f, 43.f);
    staticBodyCount++;

    rects[1] = StaticRect({ 1100.f, 600.f }, 300.f, 50.f, -43.f);
    staticBodyCount++;
}

void Simulation::InitialStep(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidCircle& circle = circles[i];

        circle.prevPos = circle.pos;
        circle.pos += gravity * (DeltaTime * DeltaTime * 0.5);
    }
}

void Simulation::Step(float DeltaTime) {
    for (int i = 0; i < rigidBodyCount; i++) {
        RigidCircle& circle = circles[i];

        Vector2 newPos = (circle.pos * 2) - circle.prevPos + (gravity * DeltaTime * DeltaTime);
        circle.prevPos = circle.pos;
        circle.pos = newPos;

        Vector2 displacement = (circle.pos - circle.prevPos);
        if (circle.pos.y > boundary.w - circle.radius) {
            circle.pos.y = boundary.w - circle.radius;
            circle.prevPos.y = circle.pos.y + displacement.y;
        }
        else if (circle.pos.y < boundary.y + circle.radius) {
            circle.pos.y = boundary.y + circle.radius;
            circle.prevPos.y = circle.pos.y + displacement.y;
        }
        else if (circle.pos.x < boundary.x + circle.radius) {
            circle.pos.x = boundary.x + circle.radius;
            circle.prevPos.x = circle.pos.x + displacement.x;
        }
        else if (circle.pos.x > boundary.z - circle.radius) {
            circle.pos.x = boundary.z - circle.radius;
            circle.prevPos.x = circle.pos.x + displacement.x;
        }

        for (int n = 0; n < staticBodyCount; n++) {
            Vector2 collisionNormal;

            StaticRect& rect = rects[n];

            bool isColliding = rect.CheckCollision(circle, collisionNormal);

            if (!isColliding) {
                continue;
            }

            Vector2 displacement = (circle.pos - circle.prevPos);

            Vector2 toCircle = circle.GetPos() - rect.GetPos();
            toCircle = Vector2Rotate(toCircle, -rect.GetRotation() * PI / 180);

            if (collisionNormal.x == 0) {
                toCircle.y = (circle.GetRadius() + rect.GetHeight() / 2) * collisionNormal.y;
            }
            else {
                toCircle.x = (circle.GetRadius() + rect.GetWidth() / 2) * collisionNormal.x;
            }

            toCircle = Vector2Rotate(toCircle, rect.GetRotation() * PI / 180);
            circle.pos = rect.pos + toCircle;

            Vector2 worldNormal = Vector2Rotate(collisionNormal, rect.GetRotation() * PI / 180);
            Vector2 displacementNormal = worldNormal * Vector2DotProduct(displacement, worldNormal);
            circle.prevPos = circle.pos - (displacement - (displacementNormal * 2));

        }

        //circle.prevPos = circle.pos;
        //circle.pos += circle.cumulativeImpulse * (1 / circle.mass) * DeltaTime;
        circle.cumulativeImpulse = { 0.f, 0.f };
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