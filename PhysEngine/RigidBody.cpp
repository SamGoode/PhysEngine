#include "RigidBody.h"

#include "rlshort.h"


RigidCircle::RigidCircle(Vector2 _pos, float _mass, float _radius) {
    pos = _pos;
    vel = { 0.f, 0.f };
    mass = _mass;

    rotation = 0.f;
    angularVel = 0.f;
    rotationalInertia = _mass * _radius * _radius * 0.5;

    radius = _radius;
}

void RigidCircle::Draw() {
    DrawCircle(pos.x, pos.y, radius, BLUE);
}

RigidRect::RigidRect(Vector2 _pos, float _mass, float _width, float _height, float _rotation) {
    pos = _pos;
    vel = { 0.f, 0.f };
    mass = _mass;

    rotation = _rotation;
    angularVel = 0.f;
    rotationalInertia = _mass * (_width * _width + _height * _height) / 12;

    width = _width;
    height = _height;
}

void RigidRect::Draw() {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation, BLUE);
}
