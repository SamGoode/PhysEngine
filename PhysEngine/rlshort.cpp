#include "rlshort.h"


Vector2 operator*(const Vector2& a, const float& b) { return { a.x * b, a.y * b }; }
Vector2 operator+(const Vector2& a, const Vector2& b) { return { a.x + b.x, a.y + b.y }; }
Vector2 operator-(const Vector2& a, const Vector2& b) { return { a.x - b.x, a.y - b.y }; }
Vector2& operator+=(Vector2& a, const Vector2& b) { a = a + b; return a; }

void DrawVectorText(Vector2 a, int posX, int posY, int fontSize, Color color) { DrawText((std::to_string(a.x) + ", " + std::to_string(a.y)).c_str(), posX, posY, fontSize, color); }