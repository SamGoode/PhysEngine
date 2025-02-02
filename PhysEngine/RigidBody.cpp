#include "RigidBody.h"

#include "rlshort.h"


RigidCircle::RigidCircle(Vector2 _pos, float _radius, float _mass) {
    pos = _pos;
    vel = { 0.f, 0.f };

    radius = _radius;
    mass = _mass;
}

void RigidCircle::Draw() {
    DrawCircle(pos.x, pos.y, radius, BLUE);
}