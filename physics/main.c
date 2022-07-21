#include <stdio.h>
#include "include/physicshaven.h"


int main()
{

    int screenWidth = 1400, screenHeight = 900;

    InitWindow(screenWidth, screenHeight, "PHYSICS HAVEN");
    printf("A rigid body is %d bytes", sizeof(RidgidBody));
    Image icon = LoadImage("icon.png");
    SetWindowIcon(icon);
    SetTargetFPS(200);

    //RidgidBody b1 = RidgidBodyNewBox(Vector2Make(screenWidth/2, 500), Vector2Make(100,100), 100.f);
    RidgidBody b2 = RidgidBodyNewBox(Vector2Make(screenWidth/2, 650), Vector2Make(1000, 50), 100.0f); b2.isStatic = true; b2.rotation = 38*DEG2RAD;
    RidgidBody b3 = RidgidBodyNewBox(Vector2Make(1200, screenHeight/2), Vector2Make(200, 1000), 100.0f); b3.isStatic = true;

    RidgidBody b1 = RidgidBodyNewPoly(Vector2Make(screenWidth/2, 50), 50, 5, 100.f); b1.elasticity = 0.99;

    RidgidBody b4 = RidgidBodyNewPoly(Vector2Make(screenWidth/2 + 100, 10), 30, 7, 50.f);
    RidgidBody b5 = RidgidBodyNewBox(Vector2Make(screenWidth/2, -100), Vector2Make(20, 20), 30.0f); 
    RidgidBody b6 = RidgidBodyNewBox(Vector2Make(screenWidth/2, -100), Vector2Make(30, 40), 40.0f); 

    
    
    float speed = 8;
    RidgidBody *bodies[] = {&b1, &b2, &b3, &b4, &b5, &b6};



    while (!WindowShouldClose())
    {
        // Update Stuff
        float direction = 0;
        
        if (IsKeyPressed(KEY_R))
        {
            b1.position = Vector2Make(screenWidth/2, 50);
            b1.velocity = Vector2Zero();

            b4.position = Vector2Make(screenWidth/2 + 100, 50);
            b4.velocity = Vector2Zero();
        }

        //Drawing Stuff
        BeginDrawing();
            ClearBackground(BLACK);
            DrawText(TextFormat("FPS: %02i", GetFPS()), 0, 0, 50, SKYBLUE);
            for (int i = 0; i < 6; i++) DrawRidgidBodyOutline(*bodies[i]);

            SimulatePhysics(bodies, 6);
        EndDrawing();
    }
    
}