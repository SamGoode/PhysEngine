#include "RigidBody.h"

#include "rlshort.h"


RigidCircle::RigidCircle(Vector2 _pos, float _mass, float _radius) {
    pos = _pos;
    vel = { 0.f, 0.f };
    acc = { 0.f, 0.f };
    invMass = 1 / _mass;

    rot = 0.f;
    angVel = 0.f;
    angAcc = 0.f;
    invMOI = 1 / (_mass * _radius * _radius * _radius * _radius * 0.25f);

    radius = _radius;
}

void RigidCircle::Draw() {
    DrawCircle(pos.x, pos.y, radius, BLUE);
}

RigidRect::RigidRect(Vector2 _pos, float _mass, float _width, float _height, float _rotation) {
    pos = _pos;
    vel = { 0.f, 0.f };
    acc = { 0.f, 0.f };
    invMass = 1 / _mass;

    rot = _rotation;
    angVel = 0.f;
    angAcc = 0.f;
    invMOI = 1 / (_mass * _width * _width + _height * _height * (1.f / 12.f));

    width = _width;
    height = _height;
}

void RigidRect::Draw() {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rot * 180/PI, BLUE);
}
