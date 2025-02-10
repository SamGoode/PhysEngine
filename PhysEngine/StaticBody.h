#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include <cmath>

std::pair<float, float> MinMaxProjection(Vector2 corners[4], Vector2 axis);


class StaticBody {
public:
    Vector2 pos;
    float rotation;

public:
    StaticBody() {}

    const Vector2& GetPos() { return pos; }
    float GetRotation() { return rotation; }

    virtual bool CheckCollision(class RigidBody* body, struct CollisionInfo& result) = 0;

    virtual void Draw() = 0;
    virtual void Draw(class Color color) = 0;
};


class StaticCircle : public StaticBody {
public:
    float radius;

public:
    StaticCircle() {}
    StaticCircle(Vector2 _pos, float _radius);

    float GetRadius() { return radius; }

    //virtual bool CheckCollision(class RigidBody* body, CollisionInfo& result) override;

    virtual void Draw() override;
    virtual void Draw(class Color color) override;
};


class StaticRect : public StaticBody {
public:
    float width;
    float height;

public:
    StaticRect() {}
    StaticRect(Vector2 _pos, float _width, float _height, float _rotation);

    float GetWidth() { return width; }
    float GetHeight() { return height; }

    //virtual bool CheckCollision(class RigidBody* body, CollisionInfo& result) override;

    virtual void Draw() override;
    virtual void Draw(class Color color) override;
};