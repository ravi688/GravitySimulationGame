#include<graphics.h>
#include<conio.h>
#include<dos.h>

#include <math.h>
#include <iostream>

static const float DEG2RAD = 3.14159f / 180.0f;

// 2 dimensional vector struct
struct Vec2
{
	float x, y;
	
	Vec2(float _x, float _y) : x(_x), y(_y) { }
	Vec2() : x(0), y(0)  { }
	
	Vec2 operator *(float s) 
	{ 
		return { x * s, y * s };
	}
	Vec2 operator +(const Vec2& v)
	{
		return { v.x + x, v.y + y };	
	}
	
	const Vec2& operator +=(const Vec2& v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}
	const Vec2& operator *=(float s)
	{
		x *= s;
		y *= s;
		return *this;
	}
};


static int width;
static int height;
static float worldWidth;
static float worldHeight;

struct TrajectoryData
{
	Vec2 initialVelocity;
	Vec2 initialPosition;
	Vec2 gravity;
	float timeOfFlight;
};


static void drawTrajectory(const TrajectoryData& data,
					int count,
					int* const buffer)
{
	setcolor(YELLOW);
	setlinestyle(DOTTED_LINE, 1, NORM_WIDTH);
	
	float deltaTime = data.timeOfFlight / count;
	float time = 0;
	const float WORLD_TO_SCREEN_X = (float)width / worldWidth;
	const float WORLD_TO_SCREEN_Y = (float)height / worldHeight;
	for(int i = 0; i < count; i++, time += deltaTime)
	{
		int& x = buffer[i * 2];
		int& y = buffer[i * 2 + 1];
		
		// Motion under gravity
		// position = initial_position * time + acceleration * time * time * 0.5f
		float worldX = data.initialPosition.x + data.initialVelocity.x * time + data.gravity.x * time * time * 0.5f;
		float worldY = data.initialPosition.y + data.initialVelocity.y * time + data.gravity.y * time * time * 0.5f;
		
		// convert the world position to screen pixel coordinates
		x = worldX * WORLD_TO_SCREEN_X;
		y = height - worldY * WORLD_TO_SCREEN_Y;
	}
	drawpoly(count, buffer);
}

#if 0
int main()
{
	// initialze the graphics
   int gd = DETECT, gm;
   initgraph(&gd, &gm, "C:\\TC\\BGI");
   
   // set the color
   setcolor(getmaxcolor());
   
   // get the screen width and height
   width = getmaxx();
   height = getmaxy(); 
   
   // setup the world
   worldWidth = 100;
   worldHeight = worldWidth * (float)height / width;
   
   float ballRadius = 20;
   float floorHeight = 20;
   float sideWallOffset = 20;
   
   std::cout << "World: " << worldWidth << ", " << worldHeight << std::endl;
   
   // screen update time (in seconds)
   float deltaTime = (float)1 / 30;
   
   // initial data
   Vec2 gravity (0, -9.8f); 		// downward acceleration due to gravitational force
   float angleOfThrow = 45 * DEG2RAD;
   float range = worldWidth - 5;
   float speed = sqrt(range * abs(gravity.y) / sin(2 * angleOfThrow));
   float timeOfFlight = range / (speed * cos(angleOfThrow));
   
   Vec2 initialVelocity (speed * cos(angleOfThrow), speed * sin(angleOfThrow));
   Vec2 initialPosition (((sideWallOffset + ballRadius) / width) * worldWidth, 
   						((floorHeight + ballRadius) / height) * worldHeight);
   float restitution = 0.8f;
   
   int vertexCount = 20;
   int vertices[vertexCount * 2];
   
   // data at runtime
   Vec2 velocity = initialVelocity; 
   Vec2 position = initialPosition;
   
   std::cout << "Max X: " << getmaxx() << ", Max Y: " << getmaxy() << std::endl;
   
   // render loop
   while(true)
   {
   		// clear the viewport
   		clearviewport();
   		
   		// Display a nice text
   		outtext("Bouncy Ball Example");
   		
   		// floor
   		bar(5, height - floorHeight, width - 5, height - floorHeight + 1);
   		
   		// sidewall
   		bar(sideWallOffset, 5, sideWallOffset + 1, height + 5);
   		
   		// draw the trajectory
   		TrajectoryData data { };
   		data.initialVelocity = initialVelocity;
   		data.initialPosition = initialPosition;
   		data.gravity = gravity;
   		data.timeOfFlight = timeOfFlight;
   		
   		drawTrajectory(data, vertexCount, vertices);
   		
   		// simulate physics (kinematics)
   		velocity += gravity * deltaTime;
   		position += velocity * deltaTime;	
   		
   		// calculate the screen (pixel) coordinates
   		float y = (1 - (position.y / worldHeight)) * height;
   		float x = (position.x / worldWidth) * width;
   		
   		setlinestyle(SOLID_LINE, 1, NORM_WIDTH);
   		setcolor(WHITE);
   	
   		// draw the ball
		arc(x, y, 0, 360, ballRadius);
   		
   		// if the ball collides with the floor then reverse the y-component of the velocity
   		if(y >= ((height - floorHeight) - ballRadius))
   		{
   			velocity.y *= -1 * restitution;
		}
   		
   		// add a slight delay to be able to see correctly
   		delay(deltaTime * 1000);
   }
   getch();
   closegraph();
   
   return 0;
}

#endif
