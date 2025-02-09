#include "StaticBody.h"

#include <cmath>
#include <limits>
#include "rlshort.h"

#include "RigidBody.h"

#include "CollisionInfo.h"

#include <iostream>


StaticCircle::StaticCircle(Vector2 _pos, float _radius) {
    pos = _pos;
    rotation = 0.f;
    radius = _radius;
}

bool StaticCircle::CheckCollision(RigidBody* body, CollisionInfo& result) {
    RigidCircle* circle = dynamic_cast<RigidCircle*>(body);
    if (circle) {
        float radii = (circle->radius + radius);
        if (Vector2DistanceSqr(circle->pos, pos) > radii * radii) {
            return false;
        }

        result.normal = Vector2Normalize(circle->pos - pos);
        result.impact = pos + (result.normal * radii);
        return true;
    }

    RigidRect* rect = dynamic_cast<RigidRect*>(body);
    if (rect) {
        Vector2 toCircle = pos - rect->pos;
        toCircle = Vector2Rotate(toCircle, -rect->rot);

        if (abs(toCircle.x) > rect->width / 2 + radius || abs(toCircle.y) > rect->height / 2 + radius) {
            return false;
        }

        if (abs(toCircle.x) > rect->width / 2 && abs(toCircle.y) > rect->height / 2) {
            Vector2 corner = { rect->width / 2, rect->height / 2 };

            if (toCircle.x < 0) { corner.x *= -1; }
            if (toCircle.y < 0) { corner.y *= -1; }

            if (Vector2DistanceSqr(corner, toCircle) > radius * radius) {
                return false;
            }

            result.normal = Vector2Normalize((rect->pos + Vector2Rotate(corner, rect->rot) - pos));
            result.impact = pos + result.normal * radius;
            result.depth = Vector2Distance(result.impact, rect->pos + Vector2Rotate(corner, rect->rot));

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
        else if (abs(toCircle.y) / abs(toCircle.x) > rect->height / rect->width) {
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
            toImpact.y = (rect->height / 2) * result.normal.y;
            result.depth = (toImpact.y - toCircle.y);
        }
        else if (result.normal.y == 0) {
            toImpact.x = (rect->width / 2) * result.normal.x;
            result.depth = (toImpact.x - toCircle.x);
        }

        toImpact = Vector2Rotate(toImpact, rect->rot);
        result.impact = rect->pos + toImpact;

        Vector2 worldNormal = Vector2Rotate(result.normal, rect->rot) * -1;
        result.normal = worldNormal;

        return true;
    }

    return false;
}

void StaticCircle::Draw() {
    DrawCircle(pos.x, pos.y, radius, GREEN);
}

void StaticCircle::Draw(Color color) {
    DrawCircle(pos.x, pos.y, radius, color);
}

StaticRect::StaticRect(Vector2 _pos, float _width, float _height, float _rotation) {
    pos = _pos;
    rotation = _rotation;

    width = _width;
    height = _height;
}

std::pair<float, float> MinMaxProjection(Vector2 corners[4], Vector2 axis) {
    float projection = Vector2DotProduct(corners[0], axis);
    float max = projection;
    float min = projection;

    for (int i = 1; i < 4; i++) {
        projection = Vector2DotProduct(corners[i], axis);

        if (projection > max) {
            max = projection;
        }
        if (projection < min) {
            min = projection;
        }
    }

    return std::pair(min, max);
}

bool StaticRect::CheckCollision(RigidBody* body, CollisionInfo& result) {
    RigidCircle* circle = dynamic_cast<RigidCircle*>(body);
    if (circle) {
        Vector2 toCircle = circle->GetPos() - pos;
        float radius = circle->radius;

        toCircle = Vector2Rotate(toCircle, -rotation);

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
            Vector2 toImpact = corner + (result.normal * circle->radius);
            toImpact = Vector2Rotate(toImpact, rotation);
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
            toImpact.y = (circle->radius + height / 2) * result.normal.y;
        }
        else if (result.normal.y == 0) {
            toImpact.x = (circle->radius + width / 2) * result.normal.x;
        }

        toImpact = Vector2Rotate(toImpact, rotation);
        result.impact = pos + toImpact;
    
        return true;
    }

    RigidRect* rect = dynamic_cast<RigidRect*>(body);
    if (rect) {
        Vector2 corners[4] = {
            -rect->width / 2, -rect->height / 2,
            rect->width / 2, -rect->height / 2,
            rect->width / 2, rect->height / 2,
            -rect->width / 2, rect->height / 2,
        };

        Vector2 m_corners[4] = {
            -width / 2, -height / 2,
            width / 2, -height / 2,
            width / 2, height / 2,
            -width / 2, height / 2,
        };

        Vector2 normals[8];
        for (int i = 0; i < 4; i++) {
            corners[i] = rect->pos + Vector2Rotate(corners[i], rect->rot);
            normals[i] = Vector2Rotate({ 0.f, -1.f }, rect->rot + (PI * 0.5 * i));

            m_corners[i] = pos + Vector2Rotate(m_corners[i], rotation);
            normals[i + 4] = Vector2Rotate({ 0.f, -1.f }, rotation + (PI * 0.5 * i));
        }

        int minOverlapIndex = -1;
        float minOverlap = FLT_MAX;

        for (int i = 0; i < 8; i++) {
            Vector2 normal = normals[i];

            std::pair r1 = MinMaxProjection(corners, normal);
            std::pair r2 = MinMaxProjection(m_corners, normal);

            if (r1.second < r2.first || r2.second < r1.first) {
                return false;
            }

            float overlap = std::min(abs(r1.second - r2.first), abs(r1.first - r2.second));
            if (overlap < minOverlap) {
                minOverlapIndex = i;
                minOverlap = overlap;
            }
        }

        result.worldNormal = normals[minOverlapIndex];
        result.depth = minOverlap;

        if (minOverlapIndex < 4) {            
            Vector2 corner = corners[minOverlapIndex];

            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(m_corners[i] - corner, result.worldNormal) < 0.f) {
                    result.impact = m_corners[i];
                    break;
                }
            }

            result.depth *= -1;
            result.worldNormal = result.worldNormal * -1;
            result.normal = Vector2Rotate(result.worldNormal, -rotation);
        }
        else {
            Vector2 corner = m_corners[minOverlapIndex - 4];

            for (int i = 0; i < 4; i++) {
                if (Vector2DotProduct(corners[i] - corner, result.worldNormal) < 0.f) {
                    result.impact = corners[i]; //+ result.worldNormal * minOverlap;
                    break;
                }
            }

            result.normal = Vector2Rotate(result.worldNormal, -rotation);
        }

        return true;
    }

    return false;
}

void StaticRect::Draw() {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation * 180/PI, GREEN);
}

void StaticRect::Draw(Color color) {
    DrawRectanglePro({ pos.x, pos.y, width, height }, { width / 2, height / 2 }, rotation * 180/PI, color);
}


