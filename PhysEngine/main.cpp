#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include <string>
#include <iostream>
#include <fstream>

#include "ImgurRequest.h"
#include "JPEG.h"

Color convertIntToColor(int rgba) {
    unsigned char r = rgba >> 24;
    unsigned char g = rgba >> 16;
    unsigned char b = rgba >> 8;
    unsigned char a = rgba;

    return { r, g, b, a };
}

Vector2 operator*(const Vector2& a, const float& b) {
    return { a.x * b, a.y * b };
}

Vector2 operator+(const Vector2& a, const Vector2& b) {
    return { a.x + b.x, a.y + b.y };
}

Vector2 operator-(const Vector2& a, const Vector2& b) {
    return { a.x - b.x, a.y - b.y };
}

Vector2& operator+=(Vector2& a, const Vector2& b) {
    a = a + b;
    return a;
}

class RigidCircle {
public:
    Vector2 prevPos;
    Vector2 pos;

    Vector2 cumulativeImpulse;

    float radius;
    float mass;

public:
    RigidCircle() {}

    RigidCircle(Vector2 _pos, float _radius, float _mass) {
        prevPos = _pos;
        pos = _pos;

        cumulativeImpulse = { 0.f, 0.f };

        radius = _radius;
        mass = _mass;
    }

    const Vector2& GetPos() const {
        return pos;
    }
    
    float GetRadius() {
        return radius;
    }

    float GetRadius() const {
        return radius;
    }

    float GetMass() {
        return mass;
    }

    const Vector2& GetImpulse() {
        return cumulativeImpulse;
    }

    void ApplyImpulse(Vector2 impulse) {
        cumulativeImpulse += impulse;
    }

    void SetPos(Vector2 newPos) {
        pos = newPos;
    }

    void Update(float DeltaTime) {

        cumulativeImpulse = { 0.f, 0.f };
    }

    void Draw() {
        DrawCircle(pos.x, pos.y, radius, BLUE);
    }
};

class StaticRect {
private:
    Vector2 pos;
    float width;
    float height;

    float rotation;

public:
    StaticRect() {}

    StaticRect(Vector2 _pos, float _width, float _height, float _rotation) {
        pos = _pos;
        width = _width;
        height = _height;

        rotation = _rotation;
    }

    const Vector2& GetPos() {
        return pos;
    }

    float GetWidth() {
        return width;
    }

    float GetHeight() {
        return height;
    }

    float GetRotation() {
        return rotation;
    }

    bool CheckCollision(const RigidCircle& circle, Vector2& normal) {
        Vector2 toCircle = circle.GetPos() - pos;
        float radius = circle.GetRadius();

        toCircle = Vector2Rotate(toCircle, -rotation * PI/180);

        if (toCircle.x == 0) {
            if (toCircle.y > 0) {
                normal = { 0.f, 1.f };
            }
            else {
                normal = { 0.f, -1.f };
            }
        }
        else if (abs(toCircle.y) / abs(toCircle.x) > height / width) {
            if (toCircle.y > 0) {
                normal = { 0.f, 1.f };
            }
            else {
                normal = { 0.f, -1.f };
            }
        }
        else {
            if (toCircle.x > 0) {
                normal = { 1.f, 0.f };
            }
            else {
                normal = { -1.f, 0.f };
            }
        }
        
        return (abs(toCircle.x) < width / 2 + radius && abs(toCircle.y) < height / 2 + radius);
    }

    void Draw() {
        DrawRectanglePro({ pos.x, pos.y, width, height }, {width/2, height/2}, rotation, GREEN);
    }

    void Draw(Color color) {
        DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation, color);
    }
};

class Simulation {
private:
    Vector2 gravity = { 0.f, 100.f };

    RigidCircle circles[64];
    StaticRect rects[64];

    int rigidBodyCount;
    int staticBodyCount;

public:
    Simulation() {
        rigidBodyCount = 0;
        staticBodyCount = 0;

        circles[0] = RigidCircle({ 900.f, 100.f }, 25.f, 10.f);
        rigidBodyCount++;

        rects[0] = StaticRect({900.f, 600.f}, 400.f, 50.f, -43.f);
        staticBodyCount++;
    }

    void Step(float DeltaTime) {
        for (int i = 0; i < rigidBodyCount; i++) {
            RigidCircle& circle = circles[i];

            Vector2 circleVel = circle.pos - circle.prevPos; // fix later?
            circle.prevPos = circle.pos;
            circle.pos += circleVel;

            circle.pos += gravity * DeltaTime * DeltaTime;

            if (circle.GetPos().y > 900 - circle.GetRadius()) {
                Vector2 vel = (circle.pos - circle.prevPos) * (1 / DeltaTime);

                circle.pos.y = 900 - circle.radius;

                circle.ApplyImpulse(Vector2{ 0.f, -vel.y } * circle.mass);
            }

            for (int n = 0; n < staticBodyCount; n++) {
                Vector2 collisionNormal;

                StaticRect& rect = rects[n];

                bool isColliding = rect.CheckCollision(circle, collisionNormal);

                if (!isColliding) {
                    continue;
                }

                Vector2 vel = (circle.pos - circle.prevPos) * (1 / DeltaTime);

                Vector2 toCircle = circle.GetPos() - rect.GetPos();
                toCircle = Vector2Rotate(toCircle, -rect.GetRotation() * PI / 180);

                if (collisionNormal.x == 0) {
                    toCircle.y = (circle.GetRadius() + rect.GetHeight() / 2) * collisionNormal.y;
                }
                else {
                    toCircle.x = (circle.GetRadius() + rect.GetWidth() / 2) * collisionNormal.x;
                }

                toCircle = Vector2Rotate(toCircle, rect.GetRotation() * PI / 180);
                circle.SetPos(rect.GetPos() + toCircle);

                Vector2 worldNormal = Vector2Rotate(collisionNormal, rect.GetRotation() * PI / 180);
                Vector2 normalVel = worldNormal * Vector2DotProduct(vel, worldNormal);

                circle.ApplyImpulse(normalVel * circle.mass);
            }

            circle.pos += circle.cumulativeImpulse * (1 / circle.mass) * DeltaTime;
            circle.cumulativeImpulse = { 0.f, 0.f };
        }
    }

    void Draw() {
        for (int i = 0; i < rigidBodyCount; i++) {
            circles[i].Draw();
        }

        for (int i = 0; i < staticBodyCount; i++) {
            rects[i].Draw();
        }
    }
};

int main() {
    int screenWidth = 1800;
    int screenHeight = 900;

    InitWindow(screenWidth, screenHeight, "PhysEngine");

    SetTargetFPS(240);

    Simulation sim = Simulation();

    while (!WindowShouldClose()) {
        // Updates
        float DeltaTime = GetFrameTime();

        Vector2 mouse = GetMousePosition();

        sim.Step(DeltaTime);
        
        // Drawing
        BeginDrawing();

        ClearBackground(RAYWHITE);

        sim.Draw();

        DrawFPS(10, 10);

        EndDrawing();
    }
}
