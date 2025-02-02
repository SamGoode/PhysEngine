#pragma once

#include "raylib.h"

struct CollisionInfo {
    Vector2 normal;
    Vector2 worldNormal;
    Vector2 impact;
};