#include "StaticBody.h"

#include <cmath>
#include "rlshort.h"

#include "RigidBody.h"

#include "CollisionInfo.h"


StaticCircle::StaticCircle(Vector2 _pos, float _radius) {
    pos = _pos;
    rotation = 0.f;
    radius = _radius;
}

bool StaticCircle::CheckCollision(const RigidCircle& circle, CollisionInfo& result) {
    float radii = (circle.radius + radius);
    if (Vector2DistanceSqr(circle.pos, pos) > radii * radii) {
        return false;
    }

    result.normal = Vector2Normalize(circle.pos - pos);
    result.impact = pos + (result.normal * radii);
    return true;
}

void StaticCircle::Draw() {
    DrawCircle(pos.x, pos.y, radius, GREEN);
}

void StaticCircle::Draw(Color color) {
    DrawCircle(pos.x, pos.y, radius, color);
}

StaticRect::StaticRect(Vector2 _pos, float _width, float _height, float _rotation) {
    pos = _pos;
    width = _width;
    height = _height;

    rotation = _rotation;
}

bool StaticRect::CheckCollision(const RigidCircle& circle, CollisionInfo& result) {
    Vector2 toCircle = circle.GetPos() - pos;
    float radius = circle.GetRadius();

    toCircle = Vector2Rotate(toCircle, -rotation * PI / 180);

    if (abs(toCircle.x) > width / 2 + radius || abs(toCircle.y) > height / 2 + radius) {
        return false;
    }

    if (abs(toCircle.x) > width / 2 && abs(toCircle.y) > height / 2) {
        Vector2 corner = { width / 2, height / 2 };

        if (toCircle.x < 0) { corner.x *= -1; }
        if (toCircle.y < 0) { corner.y *= -1; }

        if (Vector2DistanceSqr(corner, toCircle) > radius * radius) {
            return false;
        }

        result.normal = Vector2Normalize(toCircle - corner);
        Vector2 toImpact = corner + (result.normal * circle.radius);
        toImpact = Vector2Rotate(toImpact, rotation * PI / 180);
        result.impact = pos + toImpact;

        return true;
    }

    if (toCircle.x == 0) {
        if (toCircle.y > 0) {
            result.normal = { 0.f, 1.f };
        }
        else {
            result.normal = { 0.f, -1.f };
        }
    }
    else if (abs(toCircle.y) / abs(toCircle.x) > height / width) {
        if (toCircle.y > 0) {
            result.normal = { 0.f, 1.f };
        }
        else {
            result.normal = { 0.f, -1.f };
        }
    }
    else {
        if (toCircle.x > 0) {
            result.normal = { 1.f, 0.f };
        }
        else {
            result.normal = { -1.f, 0.f };
        }
    }

    Vector2 toImpact = toCircle;
    
    if (result.normal.x == 0) {
        toImpact.y = (circle.radius + height / 2) * result.normal.y;
    }
    else if (result.normal.y == 0) {
        toImpact.x = (circle.radius + width / 2) * result.normal.x;
    }

    toImpact = Vector2Rotate(toImpact, rotation * PI / 180);
    result.impact = pos + toImpact;

    return true;
}

void StaticRect::Draw() {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation, GREEN);
}

void StaticRect::Draw(Color color) {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation, color);
}


