#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"


Vector2 operator*(const Vector2& a, const float& b);
Vector2 operator/(const Vector2& a, const float& b);
Vector2 operator+(const Vector2& a, const Vector2& b);
Vector2 operator-(const Vector2& a, const Vector2& b);
Vector2& operator+=(Vector2& a, const Vector2& b);

bool operator==(const Vector2& a, const Vector2& b);

void DrawVectorText(Vector2 a, int posX, int posY, int fontSize, Color color);
void PrintVectorText(Vector2 a);

void DrawCircle(float centerX, float centerY, float radius, Color color);