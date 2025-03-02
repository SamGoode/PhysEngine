#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

class RigidBody {
public:
    bool isStatic;

    Vector2 pos;
    Vector2 vel;
    Vector2 acc;
    float invMass;

    float rot;
    float angVel;
    float angAcc;
    float invMOI;

    Color color = BLUE;

public:
    RigidBody() {}

    void SetColor(Color newColor) { color = newColor; }

    void ApplyImpulse(Vector2 impulse, Vector2 hitPos);
    void ApplyAngularImpulse(float angularImpulse);

    virtual int GetID() = 0;
    virtual void Draw() = 0;
};


class RigidRect : public RigidBody {
public:
    float width;
    float height;

public:
    RigidRect() {}
    RigidRect(Vector2 _pos, float _mass, float _width, float _height, float _rotation);

    virtual int GetID() override { return 0; };
    virtual void Draw() override;
};


class RigidCircle : public RigidBody {
public:
    float radius;

public:
    RigidCircle() {}
    RigidCircle(Vector2 _pos, float _mass, float _radius);

    virtual int GetID() override { return 1; };
    virtual void Draw() override;
};

