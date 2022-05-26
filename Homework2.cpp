#include<graphics.h>
#include<conio.h>
#include<dos.h>
#include <stdlib.h>

#include <math.h>
#include <iostream>

#define TO_MILLI_SECONDS(x) ((x) * 1000)
#define TO_KILO_METERS(y) ((y) * 1000)

static const float DEG2RAD = 3.14159f / 180.0f;
static const float GRAVITATIONAL_CONSTANT = 10.0f; 	// 6.67430f * 10 ^ -11 kg
static const float SUN_MASS = 100000.0f;			// 1.988 * 10 ^ 30 kg

static const float PLANET_MASS_MAX = 180.0f; 			// 1.89813 * 10 ^ 27 kg mass of the Jupitor
static const float PLANET_MASS_MIN = 30.0f; 			// 3.301 * 10 ^ 23 kg 	mass of the Mercury
static const int PLANET_RADIUS_MAX = 20;
static const int PLANET_RADIUS_MIN = 5;
static const int SUN_RADIUS = 30;

static inline float min(float v1, float v2) { return (v1 > v2) ? v2 : v1; }
static inline int min(int v1, int v2) { return (v1 > v2) ? v2 : v1; }
static inline int sign(float value) { return (value >= 0) ? 1: -1; }

struct Vec2Int
{
	int x, y;
	
	Vec2Int(const Vec2Int& v): x(v.x), y(v.y) { }
	Vec2Int() : x(0), y(0) { }
	Vec2Int(int _x, int _y) : x(_x), y(_y) { }
	
	Vec2Int operator *(float s) 
	{ 
		return { x * s, y * s };
	}
	Vec2Int operator +(const Vec2Int& v)
	{
		return { v.x + x, v.y + y };	
	}
	
	const Vec2Int& operator +=(const Vec2Int& v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}
	const Vec2Int& operator *=(float s)
	{
		x *= s;
		y *= s;
		return *this;
	}
};

// 2 dimensional vector struct
struct Vec2
{
	float x, y;
	
	Vec2(const Vec2& v) : x(v.x), y(v.y) { }
	
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
	Vec2 operator -(const Vec2& v)
	{
		return { v.x - x, v.y - y };
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
	
	Vec2 normalized()
	{
		return (*this) * (1 / magnitude());
	}
	
	float sqrMagnitude() const { return x * x + y * y; }
	float magnitude() const { return sqrt(x * x + y * y); }
};

// transform
struct Transform
{
	private:
		Vec2 position;		// rectangular coordinates, origin is at the center of the screen
		float rotation; 	// euler angle rotation, +ve is anticlockwise and -ve is clockwise
	
	public:
		Transform()
		{
			position = Vec2(0, 0);
			rotation = 0;
		}
		// setters
		void setPosition(const Vec2 position)  { this->position = position; }
		void setRotation(const float rotation) { this->rotation = rotation; }
		
		// getters
		Vec2 getPosition() const { return position; }
		float getRotation() const { return rotation; }
};


// rigidbody
struct Rigidbody
{
	private:
		
		// transform for this rigidbody
		Transform* transform;
		
		// mass of this rigidbody
		float mass;
		
		// current velocity of this rigidoby
		Vec2 velocity;
		
		// current acceleration of this rigidbody
		Vec2 acceleration;
		
	public:
		Rigidbody(Transform* _transform, float _mass): transform(_transform), mass(_mass) { }
		Rigidbody(const Rigidbody&) = delete;
		Rigidbody& operator =(const Rigidbody&) = delete;
		
		void update(float deltaTime)
		{
			velocity += acceleration * deltaTime;
			transform->setPosition(velocity * deltaTime + transform->getPosition());
			acceleration = Vec2(0, 0);
		}
		
		// Force relative to the body
		void applyForce(Vec2 force)
		{
			// f = m * a (newton's law)
			acceleration = force * (1 / mass);	
		}
		
		// setters
		void setMass(float mass) { this->mass = mass; }
		
		// getters
		float getMass() const { return mass; }
		Vec2 getVelocity() const { return velocity; }
		Vec2 getAcceleration() const { return acceleration; }
		Transform* getTransform() const { return transform; }
};

// circle collider
struct CircleCollider
{
	private:
		
		// Radius of this collider
		float radius;
		
		// Rigidbody for this collider
		Rigidbody* rigidbody;
		
	public:
		CircleCollider(Rigidbody* _rigidbody, float _radius = 1) : radius(_radius), rigidbody(_rigidbody) { }
		CircleCollider(const CircleCollider&) = delete;
		CircleCollider& operator=(const CircleCollider&) = delete;
		
		// getters
		Rigidbody* getRigidbody() { return rigidbody; }
		float getRadius() const { return radius; }
};


// collision resolver
struct CollisionResolver
{
	private:
		typedef CircleCollider* PtrCircleCollider;
	
		// buffer
		PtrCircleCollider* colliders;
		// number of colliders in the collision resolution
		int colliderCount;
		
		// capacity of the colliders buffer
		int capacity;
		
		void resizeBuffer(int newCapacity)
		{
			// if the new capacity equals to the previous capacity then do nothing
			if(newCapacity == capacity) return;
			
			// allocate another buffer
			PtrCircleCollider* newColliders = new PtrCircleCollider[newCapacity];
			
			if(colliders != NULL)
			{
				// copy the already existing collider references/ptrs
				int copyCount = min(newCapacity, colliderCount);
				for(int i = 0; i < copyCount; i++)
					newColliders[i] = colliders[i];
					
				// deallocate the previous buffer
				delete[] colliders;
			}
			
			// set the rest of empty blocks to NULL
			for(int i = colliderCount; i < newCapacity; i++)
				newColliders[i] = NULL;
			
			// replace the old buffer with the new one
			colliders = newColliders;
		}
	
	public:
		CollisionResolver(int _capacity = 10) : colliders(NULL), colliderCount(0)
		{
			resizeBuffer(_capacity);
		}
		CollisionResolver(const CollisionResolver&) = delete;
		CollisionResolver& operator =(const CollisionResolver&) = delete;
		
		~CollisionResolver()
		{
			if(colliders != NULL)
				delete[] colliders;
			colliders = NULL;
		}
		
		void addCollider(CircleCollider* collider)
		{
			// if the capacity is not enough to accomodate the new collider then increase the capacity 2 times
			if(capacity < (colliderCount + 1))
			{
				int newCapacity = capacity;
				if(newCapacity == 0)
					newCapacity = 2;
				else
					newCapacity *= 2;
				resizeBuffer(newCapacity);
			}
			
			// add the collider
			colliders[colliderCount] = collider;
			++colliderCount;
		}
		
		void removeCollider(CircleCollider* collider)
		{
			for(int i = 0; i < colliderCount; i++)
			{
				if(colliders[i] == collider)
				{
					// shift the rest of the colliders to the left obscuring the collider to be removed
					for(int j = min(i + 1, capacity - 1); j < capacity; j++)
						colliders[i - 1] = colliders[i];
					colliderCount--;
					return;
				}
			}
			std::cout << "[Warning]: you're trying to remove a collider which doesn't exist in the Collision Resolution Buffer\n";
		}
		
		const PtrCircleCollider* getColliderBuffer() const { return colliders; }
		int getColliderCount() const { return colliderCount; }
		
		void resolve()
		{
			
		}
};


// Gravity Simulator
struct GravitySimulator
{
	private:
		typedef Rigidbody* PtrRigidbody;
	
		// buffer
		PtrRigidbody* rigidbodies;
		// distance buffer
		float* distances;
		// capacity of the distance buffer
		int combinationCapacity;
		// number of unique pairs of rigibodies
		int combinationCount;
		// number of rigidbodies
		int rigidbodyCount;
		
		// capacity of the rigidbody buffer
		int capacity;
		
		void resizeBuffer(int newCapacity)
		{
			// if the new capacity equals to the previous capacity then do nothing
			if(newCapacity == capacity) return;
			
			// resize the distance buffer
			int newCombinationCapacity = (int)((newCapacity - 1) * newCapacity * 0.5f + 0.001f);
			if(newCombinationCapacity > 0)
			{
				float* newDistances = new float[newCombinationCapacity];
				if(distances != NULL)
					delete[] distances;
				distances = newDistances;
			}
			
			// allocate another buffer
			PtrRigidbody* newRigidbodies = new PtrRigidbody[newCapacity];
			
			if(rigidbodies != NULL)
			{
				// copy the already existing rigidbody references/ptrs
				int copyCount = min(newCapacity, rigidbodyCount);
				for(int i = 0; i < copyCount; i++)
					newRigidbodies[i] = rigidbodies[i];
				rigidbodyCount = copyCount;
				// deallocate the previous buffer
				delete[] rigidbodies;
			}
			
			// set the rest of empty blocks to NULL
			for(int i = rigidbodyCount; i < newCapacity; i++)
				newRigidbodies[i] = NULL;
			
			// replace the old buffer with the new one
			rigidbodies = newRigidbodies;
		}
	
	public:
		GravitySimulator(int _capacity = 10) : rigidbodies(NULL), rigidbodyCount(0)
		{
			resizeBuffer(_capacity);
		}
		GravitySimulator(const GravitySimulator&) = delete;
		GravitySimulator& operator =(const GravitySimulator&) = delete;
		
		~GravitySimulator()
		{
			if(rigidbodies != NULL)
				delete[] rigidbodies;
			rigidbodies = NULL;
		}
		
		void addRigidbody(Rigidbody* rigidbody)
		{
			// if the capacity is not enough to accomodate the new rigidbody then increase the capacity 2 times
			if(capacity < (rigidbodyCount + 1))
			{
				int newCapacity = capacity;
				if(newCapacity == 0)
					newCapacity = 2;
				else
					newCapacity *= 2;
				resizeBuffer(newCapacity);
			}
			
			// add the rigidbody
			rigidbodies[rigidbodyCount] = rigidbody;
			++rigidbodyCount;
			
			combinationCount = (int)((rigidbodyCount - 1) * rigidbodyCount * 0.5f + 0.001f);
			std::cout << "Combination Count: " << combinationCount << "\n";
		}
		
		void removeRigidbody(Rigidbody* rigidbody)
		{
			for(int i = 0; i < rigidbodyCount; i++)
			{
				if(rigidbodies[i] == rigidbody)
				{
					// shift the rest of the colliders to the left obscuring the collider to be removed
					for(int j = min(i + 1, capacity - 1); j < capacity; j++)
						rigidbodies[i - 1] = rigidbodies[i];
					rigidbodyCount--;
					combinationCount = (int)((rigidbodyCount - 1) * rigidbodyCount * 0.5f + 0.001f);
					return;
				}
			}
			std::cout << "[Warning]: you're trying to remove a rigidbody which doesn't exist in the Rigidbody Buffer\n";
		}
		
		void calculateDistances()
		{
			/*
				@ @ @ @ @ @
				
			*/
//			for(int i = 0; i < (rigidbodyCount - 1); i++)
//				distances[i] = 
		}
		
		void simulate(float deltaTime)
		{
			for(int i = 0; i < rigidbodyCount; i++)
			{
				Vec2 force(0, 0);
				Rigidbody* body1 = rigidbodies[i];
				for(int j = 0; j < rigidbodyCount; j++)
				{
					if(i == j) continue;
					/*
					 ForceMagnitude = GRAVITATIONAL_CONSTANT * MASS1 * MASS2 / (DISTANCE * DISTANCE);
					*/
					Rigidbody* body2 = rigidbodies[j];
					Vec2 radiusVector = body1->getTransform()->getPosition() - body2->getTransform()->getPosition();
					float squaredDistance = radiusVector.sqrMagnitude();
					float forceMagnitude = GRAVITATIONAL_CONSTANT * body1->getMass() * body2->getMass() / squaredDistance;
					force += radiusVector.normalized() * forceMagnitude;
				}
				body1->applyForce(force);
				body1->update(deltaTime);
			}
		}
};


struct CirclePhysicalObject
{
	private:
		Rigidbody* rigidbody;
		CircleCollider* collider;
		Transform* transform;
	public:
		CirclePhysicalObject(const CirclePhysicalObject&) = delete;
		CirclePhysicalObject& operator =(const CirclePhysicalObject&) = delete;
		
		CirclePhysicalObject(float radius)
		{
			transform = new Transform();
			rigidbody = new Rigidbody(transform, 1);
			collider = new CircleCollider(rigidbody, radius);
		}
		~CirclePhysicalObject()
		{
			delete rigidbody;
			delete collider;
			delete transform;
		}
		
		// getters
		Transform* getTransform() const { return transform; }
		Rigidbody* getRigidbody() const { return rigidbody; }
		CircleCollider* getCollider() const { return collider; }
};

struct Context
{
private:
	Vec2Int screenSize; 		// size of the window in pixel coordinates
	Vec2 worldSize; 			// size of the world in meters (Rectangular coordinates)


public:	
	Context(Vec2Int _screenSize, float worldWidth) : screenSize(_screenSize)
	{
		worldSize.x = worldWidth;
		worldSize.y = worldWidth * screenSize.y / screenSize.x;
	}
	
	// getters
	Vec2Int getScreenSize() const { return screenSize; }
	Vec2 getWorldSize() const { return worldSize; }
	
	Vec2 generateRandomPoint() const
	{
		int xvalue = rand();
		int yvalue = rand();
		static int i = 0; 
		static Vec2Int dirs[4] = 
		{
			{ 1, 1 },
			{ 1, -1 },
			{ -1, -1 },
			{ -1, 1 }
		};
		Vec2Int dir = dirs[i % 4]; ++i;	
		return { dir.x * (xvalue % ((int)(worldSize.x * 0.5f))), dir.y * (yvalue % ((int)(worldSize.y * 0.5f))) };
	}
	
	// converts screen coordinates into world coordinates
	Vec2 screenToWorldCoordinates(int xScreen, int yScreen) const
	{
		yScreen *= -1;
		xScreen += screenSize.x * 0.5f;
		yScreen += screenSize.y * 0.5f;
		float x = worldSize.x * xScreen / screenSize.x;
		float y = worldSize.y * yScreen / screenSize.y;
		return { x , y };
	}
	
	Vec2Int worldToScreenCoordinates(const Vec2& world)
	{
		return worldToScreenCoordinates(world.x, world.y);
	}
	// converts world coordinates into screen coordinates
	Vec2Int worldToScreenCoordinates(float xWorld, float yWorld) const
	{
		int x = screenSize.x * xWorld / worldSize.x;
		int y = -screenSize.y * yWorld / worldSize.y;
		return { x + screenSize.x * 0.5f, y + screenSize.y * 0.5f };
	}
};

static void drawTrajectory(int count, int* const buffer)
{
	setcolor(YELLOW);
	setlinestyle(DOTTED_LINE, 1, NORM_WIDTH);
	
	drawpoly(count, buffer);
}


void renderObjects(Context* context, CircleCollider* const* colliders, int colliderCount)
{
	for(int i = 0; i < colliderCount; i++)
	{
		// render each object
		Vec2 position = colliders[i]->getRigidbody()->getTransform()->getPosition();
   		Vec2Int screenPos = context->worldToScreenCoordinates(position);
   		arc(screenPos.x, screenPos.y, 0, 360, colliders[i]->getRadius());
   	}
}


#if 1
int main()
{
	// initialze the graphics
   int gd = DETECT, gm;
   initgraph(&gd, &gm, "C:\\TC\\BGI");
   
   // set the color
   setcolor(getmaxcolor());
   
   Context context({ getmaxx(), getmaxy() }, 1000);
   CollisionResolver collisionResolver;
   GravitySimulator gravitySimulator;

   // screen update time (in seconds)
   float deltaTime = (float)1 / 30;
   
   CirclePhysicalObject* sun = new CirclePhysicalObject(SUN_RADIUS);
   Transform* transform = sun->getCollider()->getRigidbody()->getTransform();
   transform->setPosition({ 0, 0});
   transform->setRotation(0);
   sun->getRigidbody()->setMass(SUN_MASS);
   collisionResolver.addCollider(sun->getCollider());
   gravitySimulator.addRigidbody(sun->getRigidbody());
   
   // render loop
   while(true)
   {
   		// clear the viewport
   		clearviewport();
   		
   		// Display a nice text
   		outtext("Gravity Simulation Game");
   		
   		// floor
//   		bar(5, height - floorHeight, width - 5, height - floorHeight + 1);
   		
   		// sidewall
//   		bar(sideWallOffset, 5, sideWallOffset + 1, height + 5);
   		
   		setlinestyle(SOLID_LINE, 1, NORM_WIDTH);
   		setcolor(WHITE);
   		
   		if(kbhit())
   		{
   			int key = getch();
   			if(key == KEY_UP)
   			{
   				CirclePhysicalObject* planet = new CirclePhysicalObject(PLANET_RADIUS_MIN);
   				
				Transform* transform = planet->getTransform();
   				transform->setPosition(context.generateRandomPoint());
   				transform->setRotation(0);
   				
				Rigidbody* rigidbody = planet->getRigidbody();
   				rigidbody->setMass(PLANET_MASS_MIN);
   	
   				gravitySimulator.addRigidbody(rigidbody);
   				collisionResolver.addCollider(planet->getCollider());
   			}
		}
   		
   		// simulate gravitational force
   		gravitySimulator.simulate(deltaTime);
   		
   		// resolve collision
   		collisionResolver.resolve();
   	
   		// render the objects
   		CircleCollider* const* colliders = collisionResolver.getColliderBuffer();
   		int colliderCount = collisionResolver.getColliderCount();
		renderObjects(&context, colliders, colliderCount);
   	
   		// add a slight delay to be able to see correctly
   		delay(TO_MILLI_SECONDS(deltaTime));
   }
   getch();
   closegraph();
   
   return 0;
}

#endif
