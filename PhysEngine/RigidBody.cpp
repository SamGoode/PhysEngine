#include "RigidBody.h"

#include "rlshort.h"


RigidCircle::RigidCircle(Vector2 _pos, float _mass, float _radius) {
    pos = _pos;
    mass = _mass;
    radius = _radius;

    vel = { 0.f, 0.f };
}

void RigidCircle::Draw() {
    DrawCircle(pos.x, pos.y, radius, BLUE);
}

RigidRect::RigidRect(Vector2 _pos, float _mass, float _width, float _height, float _rotation) {
    pos = _pos;
    mass = _mass;

    width = _width;
    height = _height;
    rotation = _rotation;

    vel = { 0.f, 0.f };
}

void RigidRect::Draw() {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation, BLUE);
}
