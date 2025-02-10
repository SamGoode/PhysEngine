#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

class RigidBody {
public:
    Vector2 pos;
    Vector2 vel;
    Vector2 acc;
    float invMass;

    float rot;
    float angVel;
    float angAcc;
    float invMOI;

    bool isStatic;

public:
    RigidBody() {}

    const Vector2& GetPos() const { return pos; }
    void SetPos(Vector2 newPos) { pos = newPos; }

    void SetStatic() {
        vel = { 0.f, 0.f };
        acc = { 0.f, 0.f };
        invMass = 0.f;

        angVel = 0.f;
        angAcc = 0.f;
        invMOI = 0.f;

        isStatic = true;
    }

    virtual void Draw() = 0;
};


class RigidCircle : public RigidBody {
public:
    float radius;

public:
    RigidCircle() {}
    RigidCircle(Vector2 _pos, float _mass, float _radius);

    float GetRadius() const { return radius; }

    virtual void Draw() override;
};

class RigidRect : public RigidBody {
public:
    float width;
    float height;

public:
    RigidRect() {}
    RigidRect(Vector2 _pos, float _mass, float _width, float _height, float _rotation);

    virtual void Draw() override;
};