#include "raylib.h"
#include <vector>

int main() {
    // --- Initialization ---
    const int screenW = 800;
    const int screenH = 450;
    InitWindow(screenW, screenH, "Jump to Survive!");

    // --- Player Setup ---
    Rectangle player = { 100, screenH - 80, 40, 60 };
    float vy = 0;
    const float gravity      = 1000.0f;
    const float jumpImpulse  = -450.0f;

    // --- Ground Definition ---
    const int groundY = screenH - 20;

    // --- Obstacle Management ---
    std::vector<Rectangle> obstacles;
    float spawnTimer     = 0.0f;
    float spawnInterval  = 1.2f;
    const float obstacleSpeed    = 300.0f;

    const float minSpawnInterval = 0.8f;
    const float maxSpawnInterval = 1.6f;

    const int   minObsWidth      = 40;
    const int   maxObsWidth      = 120;

    // --- Game State Variables ---
    int  score    = 0;
    bool gameOver = false;

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (!gameOver) {
            // Jump logic
            if (IsKeyPressed(KEY_SPACE) && player.y + player.height >= groundY) {
                vy = jumpImpulse;
            }
            vy += gravity * dt;
            player.y += vy * dt;
            if (player.y + player.height > groundY) {
                player.y = groundY - player.height;
                vy = 0;
            }

            // Spawn obstacles with random width & interval
            spawnTimer += dt;
            if (spawnTimer >= spawnInterval) {
                spawnTimer = 0.0f;
                // recalc next interval
                spawnInterval = GetRandomValue(int(minSpawnInterval*100), int(maxSpawnInterval*100)) / 100.0f;
                // random width
                int w = GetRandomValue(minObsWidth, maxObsWidth);
                obstacles.push_back({ float(screenW), float(groundY - 40), float(w), 40.0f });
            }

            // Move & collide obstacles
            for (int i = 0; i < (int)obstacles.size(); i++) {
                obstacles[i].x -= obstacleSpeed * dt;
                if (CheckCollisionRecs(player, obstacles[i])) {
                    gameOver = true;
                }
            }
            // Remove off-screen & score
            if (!obstacles.empty() && obstacles.front().x + obstacles.front().width < 0) {
                obstacles.erase(obstacles.begin());
                score++;
            }
        }
        else {
            if (IsKeyPressed(KEY_R)) {
                // reset everything
                player.y = screenH - 80;
                vy = 0;
                obstacles.clear();
                spawnTimer    = 0.0f;
                spawnInterval = 1.2f;
                score         = 0;
                gameOver      = false;
            }
        }

        // --- Drawing ---
        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawRectangle(0, groundY, screenW, 20, DARKGRAY);
        DrawRectangleRec(player, BLUE);
        for (auto &obs : obstacles) DrawRectangleRec(obs, RED);

        DrawText(TextFormat("Score: %d", score), 10, 10, 20, BLACK);

        if (gameOver) {
            DrawText("GAME OVER! Press R to restart", 200, screenH/2 - 20, 20, MAROON);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
