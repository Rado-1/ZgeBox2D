/*
ZGEBox2D Library
Copyright (c) 2013-2014 Radovan Cervenka

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software
in a product, an acknowledgment in the product documentation would be
appreciated but is not required.

2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.

3. This notice may not be removed or altered from any source distribution.
*/

/// The main file used to compile Windows DLL and Android shared library

#pragma unmanaged

// Includes

#include "Box2D\Box2D.h"

// Definitions

#define WIN32_LEAN_AND_MEAN

#ifdef _WIN32
#define export extern "C" __declspec(dllexport)
#else
#define export extern "C"
#endif

#define PI2 6.28318530718f

#define FALSE 0
#define TRUE 1

// utils
#define angleToRadians(angle) angle * PI2
#define angleFromRadians(angle) angle / PI2
#define intToBool(int_value) int_value!=0

#define CHECK_INIT if(!g_IsInitialized) return 0;
#define CHECK_INIT_VOID if(!g_IsInitialized) return;

// Types

const int32 MAX_CONTACT_POINTS = 2048;

struct ContactPoint {
	b2Body* bodyA;
	b2Body* bodyB;
	b2Vec2 position;
	b2Vec2 normal;
	b2PointState state;
	//float32 normalImpulse;
	//float32 tangentImpulse;
	//float32 separation;
};

// Callbacks

// Raycast callback
class ZGERayCastCallback : public b2RayCastCallback {
public:

	void Init() {
		m_body = NULL;
	}

	// finds closest fixture
	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
						  const b2Vec2& normal, float32 fraction) {
			m_body = fixture->GetBody();
			m_point = point;
			m_normal = normal;
			return fraction;
	}

	b2Body* m_body;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

// Collision callback
class ZGEContactListener : public b2ContactListener {
public:

	void Init() {
		m_pointCount = 0;
	}

	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {
		const b2Manifold* manifold = contact->GetManifold();

		if (manifold->pointCount == 0) {
			return;
		}

		b2Body* bodyA = contact->GetFixtureA()->GetBody();
		b2Body* bodyB = contact->GetFixtureB()->GetBody();

		b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
		b2GetPointStates(state1, state2, oldManifold, manifold);

		b2WorldManifold worldManifold;
		contact->GetWorldManifold(&worldManifold);

		for (int32 i = 0; i < manifold->pointCount && m_pointCount < MAX_CONTACT_POINTS; ++i) {
			ContactPoint* cp = m_points + m_pointCount;
			cp->bodyA = bodyA;
			cp->bodyB = bodyB;
			cp->position = worldManifold.points[i];
			cp->normal = worldManifold.normal;
			cp->state = state2[i];
			//cp->normalImpulse = manifold->points[i].normalImpulse;
			//cp->tangentImpulse = manifold->points[i].tangentImpulse;
			//cp->separation = worldManifold.separations[i];
			++m_pointCount;
		}
	}

	ContactPoint m_points[MAX_CONTACT_POINTS];
	int32 m_pointCount;
};

// Query callback
class ZGEQueryCallback : public b2QueryCallback {
public:

	void Init(const b2Vec2& point) {
		m_point = point;
		m_body = NULL;
	}

	bool ReportFixture(b2Fixture* fixture) {
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody)
			if (fixture->TestPoint(m_point))
			{
				m_body = body;

				// we are done, terminate the query
				return false;
			}

			// continue the query
			return true;
	}

	b2Vec2 m_point;
	b2Body* m_body;
};

// Globals

b2World* g_World;
ZGERayCastCallback g_RayCastCallback;
ZGEContactListener g_ContactListener;
ZGEQueryCallback g_QueryCallback;
int32 g_CurrentContactPoint;
b2Body* g_GhostBody;
bool g_IsInitialized;

// World

export void zb2InitWorld(float gravityX, float gravityY) {
	g_World = new b2World(b2Vec2(gravityX, gravityY));
	g_GhostBody = NULL;
	g_IsInitialized = true;
}

export void zb2DestroyWorld() {
	CHECK_INIT_VOID
	delete g_World;
	g_IsInitialized = false;
}

export void zb2AllowCollisionDetection(bool allow) {
	CHECK_INIT_VOID
	if(allow) g_ContactListener.Init();
	g_World->SetContactListener(allow ? &g_ContactListener : NULL);
}

export void zb2Step(float timeStep, int velocityIterations, int positionIterations) {
	CHECK_INIT_VOID
	g_ContactListener.m_pointCount = 0;
	g_World->Step(timeStep, velocityIterations, positionIterations);
}

export void zb2ClearForces() {
	CHECK_INIT_VOID
	g_World->ClearForces();
}

export void zb2SetAllowSleeping(bool allow) {
	CHECK_INIT_VOID
	g_World->SetAllowSleeping(allow);
}

export void zb2SetGravity(float x, float y) {
	CHECK_INIT_VOID
	g_World->SetGravity(b2Vec2(x, y));
}

export void zb2SetWarmStarting(bool enable) {
	CHECK_INIT_VOID
	g_World->SetWarmStarting(enable);
}

export int zb2GetBodyCount() {
	CHECK_INIT
	return g_World->GetBodyCount();
}

export void zb2DestroyAllBodies() {
	CHECK_INIT_VOID

	b2Body* tmp;
	b2Body* body = g_World->GetBodyList();
	while(body) {
		tmp = body->GetNext();
		g_World->DestroyBody(body);
		body = tmp;
	}
	g_GhostBody = NULL;
}

// Bodies

export b2Body* zb2CreateEmptyBody(float x, float y, int type) {
	CHECK_INIT

	b2BodyDef bodyDef;
	bodyDef.type = (b2BodyType)type;
	bodyDef.position = b2Vec2(x, y);

	return g_World->CreateBody(&bodyDef);
}

export b2Body* zb2CreateBox(float x, float y, float width, float height,
	float angle, int type) {

	CHECK_INIT

	b2BodyDef bodyDef;
	bodyDef.type = (b2BodyType)type;
	bodyDef.position.Set(x, y);
	bodyDef.angle = angleToRadians(angle);
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2PolygonShape box;
	box.SetAsBox(width, height);

	body->CreateFixture(&box, 1.0f);

	return body;
}

export b2Body* zb2CreateCircle(float x, float y, float radius, int type) {
	CHECK_INIT

	b2BodyDef bodyDef;
	bodyDef.type = (b2BodyType)type;
	bodyDef.position.Set(x, y);
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2CircleShape circle;
	circle.m_radius = radius; 

	body->CreateFixture(&circle, 1.0f);

	return body;
}

export b2Body* zb2CreatePolygon(float x, float y, b2Vec2* vertices, int count,
	float angle, int type) {

	CHECK_INIT

	b2BodyDef bodyDef;
	bodyDef.type = (b2BodyType)type;
	bodyDef.position.Set(x, y);
	bodyDef.angle = angleToRadians(angle);
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2PolygonShape polygon;
	polygon.Set(vertices, count);

	body->CreateFixture(&polygon, 1.0f);

	return body;
}

export b2Body* zb2CreateEdge(float x1, float y1, float x2, float y2) {
	CHECK_INIT

	b2BodyDef bodyDef;
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2EdgeShape edge;
	edge.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));

	body->CreateFixture(&edge, 1.0f);

	return body;
}

export b2Body* zb2CreateChain(b2Vec2* vertices, int count, bool isLoop) {
	CHECK_INIT

	b2BodyDef bodyDef;
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2ChainShape chain;
	if(isLoop) chain.CreateLoop(vertices, count);
	else chain.CreateChain(vertices, count);

	body->CreateFixture(&chain, 1.0f);

	return body;
}

// compound bodies

export void zb2AddBox(b2Body* body, float x, float y, float width, float height, float angle) {
	CHECK_INIT_VOID

	b2PolygonShape box;
	box.SetAsBox(width, height, b2Vec2(x, y), angleToRadians(angle));
	body->CreateFixture(&box, 1.0f);
}

export void zb2AddCircle(b2Body* body, float x, float y, float radius) {
	CHECK_INIT_VOID

	b2CircleShape circle;
	circle.m_radius = radius;
	circle.m_p = b2Vec2(x, y);
	
	body->CreateFixture(&circle, 1.0f);
}

export void zb2AddPolygon(b2Body* body, b2Vec2* vertices, int count) {
	CHECK_INIT_VOID
	
	b2PolygonShape polygon;
	polygon.Set(vertices, count);

	body->CreateFixture(&polygon, 1.0f);
}

export void zb2AddEdge(b2Body* body, float x1, float y1, float x2, float y2) {
	CHECK_INIT_VOID
	
	b2EdgeShape edge;
	edge.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));

	body->CreateFixture(&edge, 1.0f);
}

export void zb2AddChain(b2Body* body, b2Vec2* vertices, int count, bool isLoop) {
	CHECK_INIT_VOID
	
	b2ChainShape chain;
	if(isLoop) chain.CreateLoop(vertices, count);
	else chain.CreateChain(vertices, count);

	body->CreateFixture(&chain, 1.0f);
}

export void zb2DestroyBody(b2Body* body) {
	CHECK_INIT_VOID
	g_World->DestroyBody(body);
}

// getters

export float zb2GetPositionX(b2Body* body) {
	CHECK_INIT
	return body->GetPosition().x;
}

export float zb2GetPositionY(b2Body* body) {
	CHECK_INIT
	return body->GetPosition().y;
}

export float zb2GetAngle(b2Body* body) {
	CHECK_INIT
	return angleFromRadians(body->GetAngle());
}

export void zb2GetTransform(b2Body* body, float &x, float &y, float &angle) {
	CHECK_INIT_VOID

	b2Vec2 position = body->GetPosition();
	x = position.x;
	y = position.y;
	angle = angleFromRadians(body->GetAngle());
}

export float zb2GetMass(b2Body* body) {
	CHECK_INIT
	return body->GetMass();
}

export int zb2IsAwake(b2Body* body) {
	CHECK_INIT
	return body->IsAwake() ? TRUE : FALSE;
}

// setters

export void zb2SetPosition(b2Body* body, float x, float y) {
	CHECK_INIT_VOID
	body->SetTransform(b2Vec2(x, y), body->GetAngle());
}

export void zb2SetRotation(b2Body* body, float angle) {
	CHECK_INIT_VOID
	body->SetTransform(body->GetPosition(), angleToRadians(angle));
}

export void zb2SetTransform(b2Body* body, float x, float y, float angle) {
	CHECK_INIT_VOID
	body->SetTransform(b2Vec2(x, y), angleToRadians(angle));
}

export void zb2SetLinearVelocity(b2Body* body, float x, float y) {
	CHECK_INIT_VOID
	body->SetLinearVelocity(b2Vec2(x, y));
}

export void zb2SetAngularVelocity(b2Body* body, float omega) {
	CHECK_INIT_VOID
	body->SetAngularVelocity(omega);
}

export void zb2SetLinearDamping(b2Body* body, float linearDamping) {
	CHECK_INIT_VOID
	body->SetLinearDamping(linearDamping);
}

export void zb2SetAngularDamping(b2Body* body, float angularDamping) {
	CHECK_INIT_VOID
	body->SetAngularDamping(angularDamping);
}

export void zb2SetDensity(b2Body* body, float density) {
	CHECK_INIT_VOID

	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetDensity(density), fixture = fixture->GetNext());
}

export void zb2SetFriction(b2Body* body, float friction) {
	CHECK_INIT_VOID

	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetFriction(friction), fixture = fixture->GetNext());
}

export void zb2SetRestitution(b2Body* body, float restitution) {
	CHECK_INIT_VOID

	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetRestitution(restitution), fixture = fixture->GetNext());
}

export void zb2SetGravityScale(b2Body* body, float gravityScale) {
	CHECK_INIT_VOID
	body->SetGravityScale(gravityScale);
}

export void zb2SetFixedRotation(b2Body* body, bool hasFixedRotation) {
	CHECK_INIT_VOID
	body->SetFixedRotation(hasFixedRotation);
}

export void zb2SetSleepingAllowed(b2Body* body, bool isSleepingAllowed) {
	CHECK_INIT_VOID
	body->SetSleepingAllowed(isSleepingAllowed);
}

export void zb2SetBullet(b2Body* body, bool isBullet) {
	CHECK_INIT_VOID
	body->SetBullet(isBullet);
}

export void zb2SetSensor(b2Body* body, bool isSensor) {
	CHECK_INIT_VOID

	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetSensor(isSensor), fixture = fixture->GetNext());
}

export void zb2SetActive(b2Body* body, bool isActive) {
	CHECK_INIT_VOID
	body->SetActive(isActive);
}

export void zb2SetMass(b2Body* body, float mass) {
	CHECK_INIT_VOID

	b2MassData data;
	body->GetMassData(&data);
	data.mass = mass;

	body->SetMassData(&data);
}

export void zb2SetMassData(b2Body* body, float mass, float x, float y,
	float rotationalInertia) {

	CHECK_INIT_VOID

	b2MassData data;
	data.mass = mass;
	data.center = b2Vec2(x, y);
	data.I = rotationalInertia;

	body->SetMassData(&data);
}

export void zb2ResetMassData(b2Body* body) {
	CHECK_INIT_VOID
	body->ResetMassData();
}

// changing shapes

export void zb2ChangeCircleRadius(b2Body* body, float radius) {
	CHECK_INIT_VOID
	body->GetFixtureList()->GetShape()->m_radius = radius;
}

export void zb2ChangeEdge(b2Body* body, float x1, float y1, float x2, float y2) {
	CHECK_INIT_VOID
	((b2EdgeShape*) body->GetFixtureList()->GetShape())->Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
}

// Joints

export b2Joint* zb2CreateDistanceJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy, float length,
	float frequency, float dampingRatio, bool isCollided) {

	CHECK_INIT
	
	b2DistanceJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.length = length;
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export b2Joint* zb2CreateDistanceJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy, float length,
	float frequency, float dampingRatio, bool isCollided) {

	CHECK_INIT

	b2DistanceJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(anchorAx, anchorAy), b2Vec2(anchorBx, anchorBy));
	jointDef.length = length;
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdateDistanceJoint(b2DistanceJoint* joint, float length,
	float frequency, float dampingRatio) {

	CHECK_INIT_VOID
	joint->SetLength(length);
}

export b2Joint* zb2CreatePrismaticJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float axisAx, float axisAy, float referenceAngle,
	bool enableLimit, float lower, float upper,
	bool enableMotor, float maxMotorForce, float motorSpeed, bool isCollided) {

	CHECK_INIT

	b2PrismaticJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.localAxisA = b2Vec2(axisAx, axisAy);
	jointDef.referenceAngle = angleToRadians(referenceAngle);
	jointDef.enableLimit = enableLimit;
	jointDef.lowerTranslation = lower;
	jointDef.upperTranslation = upper;
	jointDef.enableMotor = enableMotor;
	jointDef.maxMotorForce = maxMotorForce;
	jointDef.motorSpeed = motorSpeed;
	jointDef.collideConnected = isCollided;

	return  g_World->CreateJoint(&jointDef);
}

export b2Joint* zb2CreatePrismaticJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY, float axisX, float axisY, 
	bool enableLimit, float lower, float upper,
	bool enableMotor, float maxMotorForce, float motorSpeed, bool isCollided) {

	CHECK_INIT

	b2PrismaticJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(anchorX, anchorY), b2Vec2(axisX, axisY));
	jointDef.enableLimit = enableLimit;
	jointDef.lowerTranslation = lower;
	jointDef.upperTranslation = upper;
	jointDef.enableMotor = enableMotor;
	jointDef.maxMotorForce = maxMotorForce;
	jointDef.motorSpeed = motorSpeed;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdatePrismaticJoint(b2PrismaticJoint* joint,
	bool enableLimit, float lower, float upper,
	bool enableMotor, float32 maxMotorForce, float motorSpeed) {

	CHECK_INIT_VOID
	joint->EnableLimit(enableLimit);
	joint->SetLimits(lower, upper);
	joint->EnableMotor(enableMotor);
	joint->SetMaxMotorForce(maxMotorForce);
	joint->SetMotorSpeed(motorSpeed);
}

export float zb2GetPrismaticTranslation(b2PrismaticJoint* joint) {
	CHECK_INIT
	return joint->GetJointTranslation();
}

export float zb2GetPrismaticSpeed(b2PrismaticJoint* joint) {
	CHECK_INIT
	return joint->GetJointSpeed();
}

export b2Joint* zb2CreateRevoluteJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float referenceAngle, bool enableLimit, float lowerAngle, float upperAngle,
	bool enableMotor, float motorSpeed, float maxMotorTorque, bool isCollided) {

	CHECK_INIT

	b2RevoluteJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.referenceAngle = angleToRadians(referenceAngle);
	jointDef.enableLimit = enableLimit;
	jointDef.lowerAngle = angleToRadians(lowerAngle);
	jointDef.upperAngle = angleToRadians(upperAngle);
	jointDef.enableMotor = enableMotor;
	jointDef.motorSpeed = motorSpeed;
	jointDef.maxMotorTorque = maxMotorTorque;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export b2Joint* zb2CreateRevoluteJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY, bool enableLimit, float lowerAngle, float upperAngle,
	bool enableMotor, float motorSpeed, float maxMotorTorque, bool isCollided) {

	CHECK_INIT

	b2RevoluteJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(anchorX, anchorY)); 
	jointDef.enableLimit = enableLimit;
	jointDef.lowerAngle = angleToRadians(lowerAngle);
	jointDef.upperAngle = angleToRadians(upperAngle);
	jointDef.enableMotor = enableMotor;
	jointDef.motorSpeed = motorSpeed;
	jointDef.maxMotorTorque = maxMotorTorque;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdateRevoluteJoint(b2RevoluteJoint* joint,
	bool enableLimit, float lowerAngle, float upperAngle,
	bool enableMotor, float motorSpeed, float maxMotorTorque) {

	CHECK_INIT_VOID
	joint->EnableLimit(enableLimit);
	joint->SetLimits(angleToRadians(lowerAngle), angleToRadians(upperAngle));
	joint->EnableMotor(enableMotor);
	joint->SetMotorSpeed(motorSpeed);
	joint->SetMaxMotorTorque(maxMotorTorque);
}

export float zb2GetRevoluteAngle(b2RevoluteJoint* joint) {
	CHECK_INIT
	return joint->GetJointAngle();
}

export b2Joint* zb2CreateWeldJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float referenceAngle, float frequency, float dampingRatio, bool isCollided) {

	CHECK_INIT

	b2WeldJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.referenceAngle = angleToRadians(referenceAngle);
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export b2Joint* zb2CreateWeldJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY, float frequency, float dampingRatio,
	bool isCollided) {

	CHECK_INIT

	b2WeldJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(anchorX, anchorY));
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdateWeldJoint(b2WeldJoint* joint, float frequency, float dampingRatio) {
	CHECK_INIT_VOID
	joint->SetFrequency(frequency);
	joint->SetDampingRatio(dampingRatio);
}

export b2Joint* zb2CreateRopeJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float maxLength, bool isCollided) {

	CHECK_INIT

	b2RopeJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.maxLength = maxLength;
	jointDef.collideConnected = isCollided;	

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdateRopeJoint(b2RopeJoint* joint, float maxLength) {
	CHECK_INIT_VOID
	joint->SetMaxLength(maxLength);
}

export b2Joint* zb2CreatePulleyJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float groundAx, float groundAy, float groundBx, float groundBy,
	float lengthA, float lengthB, float ratio, bool isCollided) {

	CHECK_INIT

	b2PulleyJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.groundAnchorA = b2Vec2(groundAx, groundAy);
	jointDef.groundAnchorB = b2Vec2(groundBx, groundBy);
	jointDef.lengthA = lengthA;
	jointDef.lengthB = lengthB;
	jointDef.ratio = ratio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export b2Joint* zb2CreatePulleyJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float groundAx, float groundAy, float groundBx, float groundBy,
	float ratio, bool isCollided) {

	CHECK_INIT

	b2PulleyJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(groundAx, groundAy), b2Vec2(groundBx, groundBy),
		b2Vec2(anchorAx, anchorAy), b2Vec2(anchorBx, anchorBy), ratio);
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}


export float zb2GetPulleyLengthA(b2PulleyJoint* joint) {
	CHECK_INIT
	return joint->GetLengthA();
}

export float zb2GetPulleyLengthB(b2PulleyJoint* joint) {
	CHECK_INIT
	return joint->GetLengthB();
}

export b2Joint* zb2CreateGearJoint(b2Joint* joint1, b2Joint* joint2,
	float ratio, bool isCollided) {

	CHECK_INIT

	b2GearJointDef jointDef;
	jointDef.joint1 = joint1;
	jointDef.joint2 = joint2;
	jointDef.ratio = ratio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdateGearJoint(b2GearJoint* joint, float ratio) {
	CHECK_INIT_VOID
	joint->SetRatio(ratio);
}

export b2Joint* zb2CreateWheelJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,	
	float axisAx, float axisAy,
	bool enableMotor, float motorSpeed, float maxMotorTorque,
	float frequency, float dampingRatio, bool isCollided) {

	CHECK_INIT

	b2WheelJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.localAxisA = b2Vec2(axisAx, axisAy);
	jointDef.enableMotor = enableMotor;
	jointDef.motorSpeed = motorSpeed;
	jointDef.maxMotorTorque = maxMotorTorque;
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export b2Joint* zb2CreateWheelJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY,float axisX, float axisY,
	bool enableMotor, float motorSpeed, float maxMotorTorque,
	float frequency, float dampingRatio, bool isCollided) {

	CHECK_INIT

	b2WheelJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(anchorX, anchorY), b2Vec2(axisX, axisY));
	jointDef.enableMotor = enableMotor;
	jointDef.motorSpeed = motorSpeed;
	jointDef.maxMotorTorque = maxMotorTorque;
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdateWheelJoint(b2WheelJoint* joint,
	bool enableMotor, float motorSpeed, float maxMotorTorque,
	float frequency, float dampingRatio) {

	CHECK_INIT_VOID
	joint->EnableMotor(enableMotor);
	joint->SetMotorSpeed(motorSpeed);
	joint->SetMaxMotorTorque(maxMotorTorque);
	joint->SetSpringFrequencyHz(frequency);
	joint->SetSpringDampingRatio(dampingRatio);
}

export float zb2GetWheelTranslation(b2WheelJoint* joint) {
	CHECK_INIT
	return joint->GetJointTranslation();
}

export float zb2GetWheelTranslationSpeed(b2WheelJoint* joint) {
	CHECK_INIT
	return joint->GetJointSpeed();
}

export b2Joint* zb2CreateMotorJoint(b2Body* bodyA, b2Body* bodyB,
	float linearOffsetX, float linearOffsetY, float angularOffset,
	float maxForce, float maxTorque, float correctionFactor, bool isCollided) {

	CHECK_INIT

	b2MotorJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.linearOffset = b2Vec2(linearOffsetX, linearOffsetY);
	jointDef.angularOffset = angleToRadians(angularOffset);
	jointDef.maxForce = maxForce;
	jointDef.maxTorque = maxTorque;
	jointDef.correctionFactor = correctionFactor;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export b2Joint* zb2CreateMotorJointWorld(b2Body* bodyA, b2Body* bodyB,
	float maxForce, float maxTorque, float correctionFactor, bool isCollided) {

	CHECK_INIT

	b2MotorJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB);
	jointDef.maxForce = maxForce;
	jointDef.maxTorque = maxTorque;
	jointDef.correctionFactor = correctionFactor;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

export void zb2UpdateMotorJoint(b2MotorJoint* joint,
	float linearOffsetX, float linearOffsetY, float angularOffset,
	float maxForce, float maxTorque, float correctionFactor) {

	CHECK_INIT_VOID

	joint->SetLinearOffset(b2Vec2(linearOffsetX, linearOffsetY));
	joint->SetAngularOffset(angularOffset);
	joint->SetMaxForce(maxForce);
	joint->SetMaxTorque(maxTorque);
	joint->SetCorrectionFactor(correctionFactor); 
}

export b2Joint* zb2CreateMouseJoint(b2Body* body, float targetX, float targetY,
	float maxForce,	float frequency, float dampingRatio) {

	CHECK_INIT

	b2MouseJointDef jointDef;
	b2Joint* mouseJoint;

	if(!g_GhostBody) {
		b2BodyDef bodyDef;
		g_GhostBody = g_World->CreateBody(&bodyDef);
	}

	jointDef.bodyA = g_GhostBody;
	jointDef.bodyB = body;
	jointDef.target = b2Vec2(targetX, targetY);
	jointDef.maxForce = maxForce;
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;

	mouseJoint = g_World->CreateJoint(&jointDef);
	body->SetAwake(true);

	return mouseJoint;
}

export void zb2UpdateMouseJoint(b2MouseJoint* joint, float targetX, float targetY) {
	CHECK_INIT_VOID
	joint->SetTarget(b2Vec2(targetX, targetY));
}

export void zb2DestroyJoint(b2Joint* joint) {
	CHECK_INIT_VOID
	g_World->DestroyJoint(joint);
}

export void zb2GetJoints(b2Body* body, b2Joint** jointList, int &count) {
	
	CHECK_INIT_VOID

	int i = 0;
	for(b2JointEdge* je = body->GetJointList(); je; je = je->next)
		jointList[i++] = je->joint;

	count = i;
}

// Custom user data

export void zb2SetUserModelToBody(b2Body* body, void* userModel) {
	CHECK_INIT_VOID
	body->SetUserData(userModel);
}

export void zb2SetUserModelToJoint(b2Joint* joint, void* userModel) {
	CHECK_INIT_VOID
	joint->SetUserData(userModel);
}

export void* zb2GetUserModelFromBody(b2Body* body) {
	CHECK_INIT
	return body->GetUserData();
}

export void* zb2GetUserModelFromJoint(b2Joint* joint) {
	CHECK_INIT
	return joint->GetUserData();
}

// Queries

export int zb2TestPoint(b2Body* body, float x, float y) {

	CHECK_INIT

	bool result = false;
	b2Vec2 point(x, y);
	b2Fixture* fixture = body->GetFixtureList(); 

	// iterate through fixtures until point is inside
	for(; fixture && !result; result = fixture->TestPoint(point), fixture = fixture->GetNext());

	return result ? TRUE : FALSE;
}

export int zb2GetBodyAtPoint(float x, float y, b2Body* &body) {
	CHECK_INIT

	b2Vec2 point(x, y);
	
	// make a small box
	b2AABB aabb;
	const b2Vec2 d(0.001f, 0.001f);
	aabb.lowerBound = point - d;
	aabb.upperBound = point + d;

	// query the world for overlapping shapes
	g_QueryCallback.Init(point);
	g_World->QueryAABB(&g_QueryCallback, aabb);

	if(g_QueryCallback.m_body)
		return FALSE;
	else {
		body = g_QueryCallback.m_body;
		return TRUE;
	}
}

export b2Body* zb2RayCast(float x1, float y1, float x2, float y2,
	float &x, float &y, float &normalX, float &normalY) {

	CHECK_INIT

	g_RayCastCallback.Init();
	g_World->RayCast(&g_RayCastCallback, b2Vec2(x1, y1), b2Vec2(x2, y2));

	CHECK_INIT
	if(g_RayCastCallback.m_body) {
		x = g_RayCastCallback.m_point.x;
		y = g_RayCastCallback.m_point.y;
		normalX = g_RayCastCallback.m_normal.x;
		normalY = g_RayCastCallback.m_normal.y;
	}
	
	return g_RayCastCallback.m_body;
}

// Collisions

export void zb2ResetContacts() {
	CHECK_INIT_VOID
	g_CurrentContactPoint = 0;
}

// return 0 - no other contact; 1 - next contact returned
export int zb2GetNextContact(b2Body* &bodyA, b2Body* &bodyB, float &posX, float &posY,
	float &normX, float &normY, int &state) {
	
	CHECK_INIT
	if(g_CurrentContactPoint >= g_ContactListener.m_pointCount)
		return FALSE;
	else {
		ContactPoint* point = g_ContactListener.m_points + g_CurrentContactPoint;

		bodyA = point->bodyA;
		bodyB = point->bodyB;
		posX = point->position.x;
		posY = point->position.y;
		normX = point->normal.x;
		normY = point->normal.y;
		state = point->state;

		++g_CurrentContactPoint;

		return TRUE;
	}
}

export int zb2GetContactCount() {
	CHECK_INIT
	return g_ContactListener.m_pointCount;
}

export int zb2IsCollided(b2Body* body) {
	CHECK_INIT

	b2ContactEdge* c = body->GetContactList();

	for(; c; c = c->next)
		if(c->contact->IsTouching()) return TRUE;

	return FALSE;
}

export int zb2GetBodyContactCount(b2Body* body) {
	CHECK_INIT

	b2ContactEdge* c = body->GetContactList();
	int i = 0;

	for(; c; c = c->next)
		if(c->contact->IsTouching()) ++i;

	return i;
}

export void zb2SetBodyFilteringFlags(b2Body* body, int categoryBits,
	int maskBits, int groupIndex) {

	CHECK_INIT_VOID

	b2Filter filter;
	filter.categoryBits = categoryBits;
	filter.maskBits = maskBits;
	filter.groupIndex = groupIndex;

	b2Fixture* fixture = body->GetFixtureList();

	for(; fixture; fixture = fixture->GetNext())
		fixture->SetFilterData(filter);
}