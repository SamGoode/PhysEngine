#pragma once

#include <limits>

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"


class RigidBody {
public:
    bool isStatic = true;

    Vector2 pos = Vector2(FLT_MAX, FLT_MAX);
    Vector2 vel = Vector2(0.f, 0.f);
    Vector2 acc = Vector2(0.f, 0.f);
    float invMass = 0.f;

    float rot = 0.f;
    float angVel = 0.f;
    float angAcc = 0.f;
    float invMOI = 0.f;

    Color color = BLUE;

public:
    RigidBody() {}

    void SetColor(Color newColor) { color = newColor; }

    void ApplyImpulse(Vector2 impulse, Vector2 hitPos);
    void ApplyAngularImpulse(float angularImpulse);

    void Update(float DeltaTime);
    void PrepUpdate(float DeltaTime);

    virtual int GetID() = 0;
    virtual void Draw(float scale) = 0;
};


class RigidRect : public RigidBody {
public:
    float width = 0.f;
    float height = 0.f;

public:
    RigidRect() {}
    RigidRect(Vector2 _pos, float _mass, float _width, float _height, float _rotation);

    virtual int GetID() override { return 0; };
    virtual void Draw(float scale) override;
};


class RigidCircle : public RigidBody {
public:
    float radius = 0.f;

public:
    RigidCircle() {}
    RigidCircle(Vector2 _pos, float _mass, float _radius);

    virtual int GetID() override { return 1; };
    virtual void Draw(float scale) override;
};

