#include "StaticBody.h"

#include "rlshort.h"

#include "RigidBody.h"


StaticRect::StaticRect(Vector2 _pos, float _width, float _height, float _rotation) {
    pos = _pos;
    width = _width;
    height = _height;

    rotation = _rotation;
}

bool StaticRect::CheckCollision(const RigidCircle& circle, Vector2& normal) {
    Vector2 toCircle = circle.GetPos() - pos;
    float radius = circle.GetRadius();

    toCircle = Vector2Rotate(toCircle, -rotation * PI / 180);

    if (toCircle.x == 0) {
        if (toCircle.y > 0) {
            normal = { 0.f, 1.f };
        }
        else {
            normal = { 0.f, -1.f };
        }
    }
    else if (abs(toCircle.y) / abs(toCircle.x) > height / width) {
        if (toCircle.y > 0) {
            normal = { 0.f, 1.f };
        }
        else {
            normal = { 0.f, -1.f };
        }
    }
    else {
        if (toCircle.x > 0) {
            normal = { 1.f, 0.f };
        }
        else {
            normal = { -1.f, 0.f };
        }
    }

    return (abs(toCircle.x) < width / 2 + radius && abs(toCircle.y) < height / 2 + radius);
}

void StaticRect::Draw() {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation, GREEN);
}

void StaticRect::Draw(Color color) {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation, color);
}