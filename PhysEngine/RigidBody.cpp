#include "RigidBody.h"

#include "rlshort.h"


void RigidBody::ApplyImpulse(Vector2 impulse, Vector2 hitPos) {
    Vector2 rad = hitPos - pos;
    Vector2 radPerp = { -rad.y, rad.x };

    vel += impulse * invMass;
    angVel += Vector2DotProduct(impulse, radPerp) * invMOI;
}

void RigidBody::ApplyAngularImpulse(float angularImpulse) {
    angVel += angularImpulse * invMOI;
}

void RigidBody::Update(float DeltaTime) {
    if (isStatic) { return; }

    vel += acc * DeltaTime * 0.5;
    pos += vel * DeltaTime;
    acc = { 0.f, 0.f };

    angVel += angAcc * DeltaTime * 0.5;
    rot += angVel * DeltaTime;
    angAcc = 0.f;
}

void RigidBody::PrepUpdate(float DeltaTime) {
    if (isStatic) { return; }

    vel += acc * DeltaTime * 0.5;
    angVel += angAcc * DeltaTime * 0.5;
}


RigidRect::RigidRect(Vector2 _pos, float _mass, float _width, float _height, float _rotation) {
    isStatic = (_mass <= 0.f);
    
    pos = _pos;
    vel = { 0.f, 0.f };
    acc = { 0.f, 0.f };
    invMass = isStatic ? 0.f : (1.f / _mass);

    rot = _rotation;
    angVel = 0.f;
    angAcc = 0.f;
    invMOI = isStatic ? 0.f : 12.f / (_mass * (_width * _width + _height * _height));

    width = _width;
    height = _height;
}

void RigidRect::Draw() {
    //Color color = BLUE;
    if (isStatic) { color = GREEN; }
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rot * 180 / PI, color);
}


RigidCircle::RigidCircle(Vector2 _pos, float _mass, float _radius) {
    isStatic = (_mass <= 0.f);
    
    pos = _pos;
    vel = { 0.f, 0.f };
    acc = { 0.f, 0.f };
    invMass = isStatic ? 0.f : (1.f / _mass);

    rot = 0.f;
    angVel = 0.f;
    angAcc = 0.f;
    invMOI = isStatic ? 0.f : 1 / (_mass * _radius * _radius * 0.5f);

    radius = _radius;
}

void RigidCircle::Draw() {
    //Color color = PURPLE;
    if (isStatic) { color = GREEN; }
    DrawCircle(pos.x, pos.y, radius, color);

    Vector2 radial = Vector2Rotate({ 1.f, 0.f }, rot) * radius;

    Vector2 startPos = pos - radial;
    Vector2 endPos = pos + radial;

    DrawLine(startPos.x, startPos.y, endPos.x, endPos.y, BLACK);
}


