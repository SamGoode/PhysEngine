#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"



class StaticBody {
public:
    Vector2 pos;

public:
    StaticBody() {}

    StaticBody(Vector2 _pos) { pos = _pos; }

    const Vector2& GetPos() { return pos; }

    virtual bool CheckCollision(const class RigidCircle& circle, struct CollisionInfo& result) = 0;

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

    virtual bool CheckCollision(const class RigidCircle& circle, CollisionInfo& result) override;

    virtual void Draw() override;

    virtual void Draw(class Color color) override;
};


class StaticRect : public StaticBody {
public:
    float width;
    float height;
    float rotation;

public:
    StaticRect() {}

    StaticRect(Vector2 _pos, float _width, float _height, float _rotation);

    float GetWidth() { return width; }

    float GetHeight() { return height; }

    float GetRotation() { return rotation; }

    virtual bool CheckCollision(const class RigidCircle& circle, CollisionInfo& result) override;

    virtual void Draw() override;

    virtual void Draw(class Color color) override;
};