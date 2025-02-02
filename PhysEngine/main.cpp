#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"

#include <string>

#include "Simulation.h"


int main() {
    int screenWidth = 1800;
    int screenHeight = 900;

    InitWindow(screenWidth, screenHeight, "PhysEngine");

    SetTargetFPS(240);

    Simulation sim = Simulation();

    sim.InitialStep(0.005f);

    while (!WindowShouldClose()) {
        // Updates
        float DeltaTime = GetFrameTime();

        Vector2 mouse = GetMousePosition();

        if (DeltaTime > 0) {
            sim.Step(DeltaTime);
        }
        
        // Drawing
        BeginDrawing();

        ClearBackground(RAYWHITE);

        sim.Draw();

        DrawFPS(10, 10);

        EndDrawing();
    }
}
