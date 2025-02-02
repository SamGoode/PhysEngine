#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"


class StaticRect {
public:
    class Vector2 pos;
    float width;
    float height;

    float rotation;

public:
    StaticRect() {}

    StaticRect(Vector2 _pos, float _width, float _height, float _rotation);

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

    bool CheckCollision(const class RigidCircle& circle, Vector2& normal);

    void Draw();

    void Draw(class Color color);
};