/******************************************************************************************************
 * Haven Physics V0.12

 * A Custom Physics Engine Written By Isaac McCracken 
 * Features:
    * Convex Polygon Colision 
    * Rigid Body Elascitity 
******************************************************************************************************/

#pragma once
#include "raylib.h"
#include "raymath.h"
#include <stdlib.h>
#include <stdio.h>

#define GRAVITY  0.3f


#ifndef HAVENAPI
    #if defined(__cplusplus)
        #define HAVENAPI extern "C"
    #else
        #define HAVENAPI extern
    #endif
#endif
/*--------------------------------------------------------------------------------------------------------*/
// Struct and Enum Definitions
/*--------------------------------------------------------------------------------------------------------*/



// Collider Type, for Body Interactions
typedef enum 
{
    CIRCLE,
    POLYGON,
} ColliderType;

typedef struct Mat2
{
    union 
    {
        struct 
        {
            float m00;
            float m01;
            float m10;
            float m11;
        };

        struct
        {
            Vector2 xCol;
            Vector2 yCol;
        };  
    };
} Mat2;

// Polygon, 2 list of points one contains the original and the other is the transformed points;
typedef struct Polygon
{
    Vector2 *points; // Transformed Points
    Vector2 *model; // Original Points
    unsigned int pointCount; // Count for loops
} Polygon;

// Rigid Body, Dynamic Rotating Bodies.
typedef struct RidgidBody
{
    // Linear Componants
    Vector2 force;
    Vector2 velocity;
    Vector2 position;

    // Angular Componants
    float torque;
    float angularVelocity;
    float rotation;
    

    float mass;
    float inertia;
    float elasticity;

    ColliderType type;
    union 
    {
        float radius;
        Polygon *poly;
    };
    
    // Add static Bodies later
    bool isStatic; 

} RidgidBody;

typedef struct ManifoldData
{
    RidgidBody bodyA;
    RidgidBody bodyB;

    Vector2 Normal;
    float penetration;

    Vector2 contactPoints[2];


}ManifoldData, *Manifold;


/*--------------------------------------------------------------------------------------------------------*/
// Global Variables
/*--------------------------------------------------------------------------------------------------------*/

float deltaTime = 0;




/*--------------------------------------------------------------------------------------------------------*/
// Function Declarations
/*--------------------------------------------------------------------------------------------------------*/

Polygon PolygonNew(int sides, float radius);
void FindAxisOfLeastPenetration(float *penetration, Vector2 *normal, Polygon *polygon1, Polygon *polygon2);
void UpdatePolygon(Polygon *polygon, Vector2 position, float rotation);
void SolveRigidBodyCollision(RidgidBody *bodyA, RidgidBody *bodyB);



/*--------------------------------------------------------------------------------------------------------*/
// Vector2 Math Operations
/*--------------------------------------------------------------------------------------------------------*/

float Vector2CrossProduct(Vector2 v1, Vector2 v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

// Makes a Vector Given Componants
Vector2 Vector2Make(float x, float y)
{
    Vector2 result = {x, y};
    return result;
}

Vector2 Vector2FromAngle(float angle)
{
    Vector2 result = {cosf(angle), sinf(angle)};
    return result;
}

Vector2 Vector2ArrayMean(Vector2 *vectors, int length)
{
    Vector2 result = {0, 0};
    for (size_t i = 0; i < length; i++)
    {
        result.x += vectors[i].x;
        result.y += vectors[i].y;
    }
    result.x /= length;
    result.y /= length;
    return result;
}

const char* Vector2ToString(Vector2 v)
{
    return TextFormat("(%.2f, %.2f)", v.x, v.y);
}


/*--------------------------------------------------------------------------------------------------------*/
// Initializer Functions
/*--------------------------------------------------------------------------------------------------------*/


RidgidBody RidgidBodyNewCircle(Vector2 position, float radius, float mass)
{
    RidgidBody result = {0};
    result.position = position;
    result.radius = radius;
    result.mass = mass;
    result.elasticity = 0.6f;
    result.type = CIRCLE;

    return result;
}

RidgidBody RidgidBodyNewPoly(Vector2 position, float radius, int sides, float mass)
{
    RidgidBody result = {0};
    result.position = position;
    result.mass = mass;
    result.elasticity = 0.6f;
    result.type = POLYGON;
    result.poly = (Polygon*)malloc(sizeof(Polygon));
    *result.poly = PolygonNew(sides, radius); 
    

    return result;
}

RidgidBody RidgidBodyNewBox(Vector2 position, Vector2 size, float mass)
{
    RidgidBody result = {0};
    result.position = position;
    result.mass = mass;
    result.elasticity = 0.6f;
    result.type = POLYGON;
    result.poly = (Polygon*)malloc(sizeof(Polygon));
    result.poly->pointCount = 4;
    result.poly->points = (Vector2*)malloc(sizeof(Vector2)*4);
    result.poly->model = (Vector2*)malloc(sizeof(Vector2)*4);

        result.poly->model[0] = Vector2Make(-size.x/2, -size.y/2); 
        result.poly->model[1] = Vector2Make(size.x/2, -size.y/2);
        result.poly->model[2] = Vector2Make(size.x/2, size.y/2);
        result.poly->model[3] = Vector2Make(-size.x/2, size.y/2);

    return result;
}

/*--------------------------------------------------------------------------------------------------------*/
// Collision Detection
/*--------------------------------------------------------------------------------------------------------*/
void SimulatePhysics(RidgidBody **bodies, int length)
{
    deltaTime = GetFrameTime() * 60;
    for (size_t i = 0; i < length; i++)
    {
        if (!(bodies[i]->isStatic))
        {
            //f = mg
            bodies[i]->force.y = GRAVITY * bodies[i]->mass;
            // v = v0 + f/m * dt
            bodies[i]->velocity = Vector2Add(bodies[i]->velocity, Vector2Scale(bodies[i]->force, 1/bodies[i]->mass * deltaTime));
            
            bodies[i]->position = Vector2Add(bodies[i]->position, Vector2Scale(bodies[i]->velocity, deltaTime));

        }
        UpdatePolygon(bodies[i]->poly, bodies[i]->position, bodies[i]->rotation);

        for (size_t j = 0; j < length; j++)
        {
            if (j == i) continue;

            SolveRigidBodyCollision(bodies[i], bodies[j]);
            UpdatePolygon(bodies[i]->poly, bodies[i]->position, bodies[i]->rotation);
        }
    }
    
}


void SolveRigidBodyCollision(RidgidBody *bodyA, RidgidBody *bodyB)
{
    RidgidBody *body1 = bodyA;
    RidgidBody *body2 = bodyB;

        //Switch Pointers if the wrong one is static

    
    float penetration = 0;
    Vector2 normal = Vector2Zero();

    FindAxisOfLeastPenetration(&penetration, &normal, body1->poly, body2->poly);
    if (body2->isStatic)
    {
        body1 = bodyB;
        body2 = bodyA;
    }

    // Order: Cirle, Polygon, Other 
    //Assume are both Polygon !Temporary
    if (penetration > 0)
    {
        if (body1->isStatic && !(body2->isStatic))
        {
            body2->position = Vector2Add(body2->position, Vector2Scale(normal, penetration));
            UpdatePolygon(body2->poly, body2->position, body2->rotation);
            body2->velocity = Vector2Scale(Vector2Reflect(body2->velocity,  normal), body2->elasticity);
            DrawLineV(body2->position, Vector2Add(body2->position, body2->velocity), RED);


            
        }
        else if (!(body1->isStatic) && !(body2->isStatic))
        {
            body1->position = Vector2Add(body1->position, Vector2Scale(normal, penetration/2));
            body2->position = Vector2Add(body2->position, Vector2Scale(normal, -penetration/2));

            float b1InverseMass = 1 / body1->mass;
            float b2InverseMass = 1 / body2->mass;

            float velocityAlongNormal = Vector2DotProduct(Vector2Subtract(body2->velocity, body1->velocity), normal);
            float e = fminf(body1->elasticity, body2->elasticity);
            float j = -(1+e) * velocityAlongNormal;
            j /= b1InverseMass + b2InverseMass;

            Vector2 impulse = Vector2Scale(normal, j);
            body1->velocity = Vector2Subtract(body1->velocity, Vector2Scale(impulse, b1InverseMass));
            body2->velocity = Vector2Add(body2->velocity, Vector2Scale(impulse, b2InverseMass));
        }
        
    }
}

// Pass in a float and a Vector to receive the axis of least penetration
void FindAxisOfLeastPenetration(float *penetration, Vector2 *normal, Polygon *polygon1, Polygon *polygon2)
{
    *penetration = INFINITY;
    Polygon *polygonA = polygon1;
    Polygon *polygonB = polygon2;

    for (int shape = 0; shape < 2; shape++)
    {
        if (shape == 1) {polygonA = polygon2; polygonB = polygon1;}

        for (int side = 0; side < polygonA->pointCount; side++)
        {
            int next = (side + 1) % polygonA->pointCount;

            Vector2 axisNormal = {-(polygonA->points[next].y - polygonA->points[side].y), polygonA->points[next].x - polygonA->points[side].x};

            float maxA = -INFINITY; float minA = INFINITY;
            for (int point = 0; point < polygonA->pointCount; point++)
            {
                float projected = Vector2DotProduct(polygonA->points[point], axisNormal);
                maxA = fmaxf(maxA, projected);
                minA = fminf(minA, projected);
            }

            float maxB = -INFINITY; float minB = INFINITY;
            for (int point = 0; point < polygonB->pointCount; point++)
            {
                float projected = Vector2DotProduct(polygonB->points[point], axisNormal);
                maxB = fmaxf(maxB, projected);
                minB = fminf(minB, projected);
            }

            float axisPenetration = fminf(maxB - minA, maxA - minB);
            axisPenetration /= Vector2Length(axisNormal);

            if (!(maxB >= minA && maxA >= minB)) 
            {
                *penetration = 0;
                *normal = Vector2Zero();
                return;
            }
            else if (axisPenetration <= *penetration)
            {
                *penetration = axisPenetration;
                *normal = axisNormal;
            }
        }
    }
    
    *normal = Vector2Normalize(*normal);

    Vector2 centreA = Vector2ArrayMean(polygon1->points, polygon1->pointCount);
    Vector2 centreB = Vector2ArrayMean(polygon2->points, polygon2->pointCount);
    Vector2 direction = Vector2Subtract(centreB, centreA);
    
    if (Vector2DotProduct(direction, *normal) >= 0)
    {
        *normal = Vector2Negate(*normal);
    }
    
    DrawLineV(centreA, Vector2Add(Vector2Scale(*normal, *penetration), centreA), GREEN);
}

// Mat2 Functions

Mat2 Mat2New(Vector2 xCol, Vector2 yCol)
{
    Mat2 result = {0}; 
    result.xCol = xCol;
    result.yCol = yCol;

    return result;
}

Mat2 Mat2FromAngle(float radian)
{
    Mat2 result = {0};

    float c = cosf(radian);
    float s = sinf(radian);

    result.m00 = c; result.m01 = -s;
    result.m10 = -s; result.m11 = -c;

    return result;
}




// Misc Functions

Polygon PolygonNew(int sides, float radius)
{
    Polygon result = { 0 };
    result.points = (Vector2*)malloc(sizeof(Vector2)*sides);
    result.model = (Vector2*)malloc(sizeof(Vector2)*sides);
    result.pointCount = sides;
    float theta = 2*PI/sides;
    
    for (size_t i = 0; i < sides; i++)
    {
        Vector2 temp = Vector2Rotate(Vector2Make(radius, 0), theta*i);  
        result.points[i] = temp; result.model[i] = temp; 
        
    }
    
    return result;
}

void UpdatePolygon(Polygon *polygon, Vector2 position, float rotation)
{
    for (size_t i = 0; i < polygon->pointCount; i++)
    {
        polygon->points[i] = Vector2Add(Vector2Rotate(polygon->model[i], rotation), position);
    
    }
    
}


void DrawPolygonV(Vector2 position, Polygon polygon, Color colour)
{
    for (size_t i = 0; i < polygon.pointCount; i++)
    {
        int j = (i + 1) % polygon.pointCount;
        DrawLineV(polygon.points[i], polygon.points[j], colour);

    }
    
}

void DrawRidgidBodyOutline(RidgidBody body)
{
    switch (body.type)
    {
    case CIRCLE:
        DrawCircleLines(body.position.x, body.position.y, body.radius, SKYBLUE);
        break;
    case POLYGON:
        DrawPolygonV(body.position, *body.poly, SKYBLUE);
        break;
    default:
        break;
    }
}
