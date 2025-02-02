#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"


//class RigidBody {
//public:
//    Vector2 prevPos;
//    Vector2 pos;
//
//    float radius;
//    float mass;
//
//public:
//    RigidCircle() {}
//
//    RigidCircle(struct Vector2 _pos, float _radius, float _mass);
//
//    const Vector2& GetPos() const { return pos; }
//
//    float GetRadius() const { return radius; }
//
//    float GetMass() { return mass; }
//
//    void SetPos(Vector2 newPos) { pos = newPos; }
//
//    void Draw();
//};

class RigidCircle {
public:
    Vector2 pos;
    Vector2 vel;

    float radius;
    float mass;

public:
    RigidCircle() {}

    RigidCircle(Vector2 _pos, float _radius, float _mass);

    const Vector2& GetPos() const { return pos; }

    float GetRadius() const { return radius; }

    float GetMass() { return mass; }

    void SetPos(Vector2 newPos) { pos = newPos; }

    void Draw();
};