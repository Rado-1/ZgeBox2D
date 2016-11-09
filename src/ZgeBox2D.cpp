/*
ZgeBox2D Library
Copyright (c) 2013-2016 Radovan Cervenka

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


// Includes

#include "Box2D\Box2D.h"

// Definitions

#define WIN32_LEAN_AND_MEAN

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C"
#endif

#define FALSE 0
#define TRUE 1

#define PI2 6.28318530718f

// utils
#define angleToRadians(angle) angle * PI2
#define angleFromRadians(angle) angle / PI2

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
class ZgeRayCastCallback : public b2RayCastCallback {
public:

	void Init() {
		m_body = nullptr;
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
class ZgeContactListener : public b2ContactListener {
public:

	void Init() {
		m_pointCount = 0;
	}

	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {
		const b2Manifold* manifold = contact->GetManifold();

		if(manifold->pointCount == 0) {
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

		// disable contact if at one fixture is a sensor
		if(contact->GetFixtureA()->GetUserData() || contact->GetFixtureB()->GetUserData())
			contact->SetEnabled(false);
	}

	ContactPoint m_points[MAX_CONTACT_POINTS];
	int32 m_pointCount;
};

// Query callback
class ZgeQueryCallback : public b2QueryCallback {
public:

	void Init(const b2Vec2& point) {
		m_point = point;
		m_body = nullptr;
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
ZgeRayCastCallback g_RayCastCallback;
ZgeContactListener g_ContactListener;
ZgeQueryCallback g_QueryCallback;
int32 g_CurrentContactPoint;
b2Body* g_GhostBody;
b2ContactEdge* g_CurrentContactEdge;

// World

EXPORT void zb2InitWorld(float gravityX, float gravityY) {
	g_World = new b2World(b2Vec2(gravityX, gravityY));
	g_GhostBody = nullptr;
}

EXPORT void zb2DestroyWorld() {
	delete g_World;
}

EXPORT void zb2AllowCollisionDetection(bool allow) {
	if(allow) g_ContactListener.Init();
	g_World->SetContactListener(allow ? &g_ContactListener : nullptr);
}

EXPORT void zb2Step(float timeStep, int velocityIterations, int positionIterations) {
	g_ContactListener.m_pointCount = 0;
	g_World->Step(timeStep, velocityIterations, positionIterations);
}

EXPORT void zb2ClearForces() {
	g_World->ClearForces();
}

EXPORT void zb2SetAllowSleeping(bool allow) {
	g_World->SetAllowSleeping(allow);
}

EXPORT void zb2SetGravity(float x, float y) {
	g_World->SetGravity(b2Vec2(x, y));
}

EXPORT void zb2SetWarmStarting(bool enable) {
	g_World->SetWarmStarting(enable);
}

EXPORT int zb2GetBodyCount() {
	return g_World->GetBodyCount();
}

EXPORT void zb2DestroyAllBodies() {
	b2Body* tmp;
	b2Body* body = g_World->GetBodyList();
	while(body) {
		tmp = body->GetNext();
		g_World->DestroyBody(body);
		body = tmp;
	}
	g_GhostBody = nullptr;
}

// Bodies

EXPORT b2Body* zb2CreateEmptyBody(float x, float y, int type) {
	b2BodyDef bodyDef;
	bodyDef.type = (b2BodyType)type;
	bodyDef.position = b2Vec2(x, y);

	return g_World->CreateBody(&bodyDef);
}

EXPORT b2Body* zb2CreateBox(float x, float y, float width, float height,
	float angle, int type) {

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

EXPORT b2Body* zb2CreateCircle(float x, float y, float radius, int type) {
	b2BodyDef bodyDef;
	bodyDef.type = (b2BodyType)type;
	bodyDef.position.Set(x, y);
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2CircleShape circle;
	circle.m_radius = radius;

	body->CreateFixture(&circle, 1.0f);

	return body;
}

EXPORT b2Body* zb2CreatePolygon(float x, float y, b2Vec2* vertices, int count,
	float angle, int type) {

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

EXPORT b2Body* zb2CreateEdge(float x1, float y1, float x2, float y2) {
	b2BodyDef bodyDef;
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2EdgeShape edge;
	edge.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));

	body->CreateFixture(&edge, 0.0f);

	return body;
}

EXPORT b2Body* zb2CreateChain(b2Vec2* vertices, int count, bool isLoop) {
	b2BodyDef bodyDef;
	b2Body* body = g_World->CreateBody(&bodyDef);

	b2ChainShape chain;
	if(isLoop) chain.CreateLoop(vertices, count);
	else chain.CreateChain(vertices, count);

	body->CreateFixture(&chain, 0.0f);

	return body;
}

// compound bodies

EXPORT void zb2AddBox(b2Body* body, float x, float y, float width, float height, float angle) {
	b2PolygonShape box;
	box.SetAsBox(width, height, b2Vec2(x, y), angleToRadians(angle));
	body->CreateFixture(&box, 1.0f);
}

EXPORT void zb2AddCircle(b2Body* body, float x, float y, float radius) {
	b2CircleShape circle;
	circle.m_radius = radius;
	circle.m_p = b2Vec2(x, y);

	body->CreateFixture(&circle, 1.0f);
}

EXPORT void zb2AddPolygon(b2Body* body, b2Vec2* vertices, int count) {
	b2PolygonShape polygon;
	polygon.Set(vertices, count);

	body->CreateFixture(&polygon, 1.0f);
}

EXPORT void zb2AddEdge(b2Body* body, float x1, float y1, float x2, float y2) {
	b2EdgeShape edge;
	edge.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));

	body->CreateFixture(&edge, 0.0f);
}

EXPORT void zb2AddChain(b2Body* body, b2Vec2* vertices, int count, bool isLoop) {
	b2ChainShape chain;
	if(isLoop) chain.CreateLoop(vertices, count);
	else chain.CreateChain(vertices, count);

	body->CreateFixture(&chain, 0.0f);
}

EXPORT void zb2DestroyBody(b2Body* body) {
	g_World->DestroyBody(body);
}

// getters

EXPORT float zb2GetPositionX(b2Body* body) {
	return body->GetPosition().x;
}

EXPORT float zb2GetPositionY(b2Body* body) {
	return body->GetPosition().y;
}

EXPORT float zb2GetAngle(b2Body* body) {
	return angleFromRadians(body->GetAngle());
}

EXPORT void zb2GetTransform(b2Body* body, float &x, float &y, float &angle) {
	b2Vec2 position = body->GetPosition();
	x = position.x;
	y = position.y;
	angle = angleFromRadians(body->GetAngle());
}

EXPORT void zb2GetLinearVelocity(b2Body* body, float &x, float &y) {
	b2Vec2 v = body->GetLinearVelocity();
	x = v.x;
	y = v.y;
}

EXPORT float zb2GetAngularVelocity(b2Body* body) {
	return body->GetAngularVelocity();
}

EXPORT float zb2GetMass(b2Body* body) {
	return body->GetMass();
}

EXPORT int zb2IsAwake(b2Body* body) {
	return body->IsAwake() ? TRUE : FALSE;
}

// setters

EXPORT void zb2SetPosition(b2Body* body, float x, float y) {
	body->SetTransform(b2Vec2(x, y), body->GetAngle());
}

EXPORT void zb2SetRotation(b2Body* body, float angle) {
	body->SetTransform(body->GetPosition(), angleToRadians(angle));
}

EXPORT void zb2SetTransform(b2Body* body, float x, float y, float angle) {
	body->SetTransform(b2Vec2(x, y), angleToRadians(angle));
}

EXPORT void zb2SetLinearVelocity(b2Body* body, float x, float y) {
	body->SetLinearVelocity(b2Vec2(x, y));
}

EXPORT void zb2SetAngularVelocity(b2Body* body, float omega) {
	body->SetAngularVelocity(omega);
}

EXPORT void zb2SetLinearDamping(b2Body* body, float linearDamping) {
	body->SetLinearDamping(linearDamping);
}

EXPORT void zb2SetAngularDamping(b2Body* body, float angularDamping) {
	body->SetAngularDamping(angularDamping);
}

EXPORT void zb2SetDensity(b2Body* body, float density) {
	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetDensity(density), fixture = fixture->GetNext());
	body->ResetMassData();
}

EXPORT void zb2SetFriction(b2Body* body, float friction) {
	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetFriction(friction), fixture = fixture->GetNext());
}

EXPORT void zb2SetRestitution(b2Body* body, float restitution) {
	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetRestitution(restitution), fixture = fixture->GetNext());
}

EXPORT void zb2SetGravityScale(b2Body* body, float gravityScale) {
	body->SetGravityScale(gravityScale);
}

EXPORT void zb2SetFixedRotation(b2Body* body, bool hasFixedRotation) {
	body->SetFixedRotation(hasFixedRotation);
}

EXPORT void zb2SetSleepingAllowed(b2Body* body, bool isSleepingAllowed) {
	body->SetSleepingAllowed(isSleepingAllowed);
}

EXPORT void zb2SetBullet(b2Body* body, bool isBullet) {
	body->SetBullet(isBullet);
}

EXPORT void zb2SetCollisionSensor(b2Body* body, bool isSensor) {
	// do not use SetSensor(), because then the collisions points
	// would not be returned; user data is used instead
	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetUserData(&isSensor), fixture = fixture->GetNext());
}

EXPORT void zb2SetSensor(b2Body* body, bool isSensor) {
	b2Fixture* fixture = body->GetFixtureList();
	for(; fixture; fixture->SetSensor(isSensor), fixture = fixture->GetNext());
}

EXPORT void zb2SetActive(b2Body* body, bool isActive) {
	body->SetActive(isActive);
}

EXPORT void zb2SetMass(b2Body* body, float mass) {
	b2MassData data;
	body->GetMassData(&data);
	data.mass = mass;

	body->SetMassData(&data);
}

EXPORT void zb2SetMassData(b2Body* body, float mass, float x, float y,
	float rotationalInertia) {

	b2MassData data;
	data.mass = mass;
	data.center = b2Vec2(x, y);
	data.I = rotationalInertia;

	body->SetMassData(&data);
}

EXPORT void zb2ResetMassData(b2Body* body) {
	body->ResetMassData();
}

// apply forces

EXPORT void zb2ApplyForce(b2Body* body, float forceX, float forceY,
	float pointX, float pointY, bool wake) {

	body->ApplyForce(b2Vec2(forceX, forceY), b2Vec2(pointX, pointY), wake);
}

EXPORT void zb2ApplyForceToCenter(b2Body* body, float forceX, float forceY, bool wake) {
	body->ApplyForceToCenter(b2Vec2(forceX, forceY), wake);
}

EXPORT void zb2ApplyTorque(b2Body* body, float torque, bool wake) {
	body->ApplyTorque(torque, wake);
}

EXPORT void zb2ApplyLinearImpulse(b2Body* body, float impulseX, float impulseY,
	float pointX, float pointY, bool wake) {

	body->ApplyLinearImpulse(b2Vec2(impulseX, impulseY), b2Vec2(pointX, pointY), wake);
}

EXPORT void zb2ApplyAngularImpulse(b2Body* body, float impulse, bool wake) {
	body->ApplyAngularImpulse(impulse, wake);
}

// changing shapes

EXPORT void zb2ChangeCircleRadius(b2Body* body, float radius) {
	body->GetFixtureList()->GetShape()->m_radius = radius;
	body->ResetMassData();
}

EXPORT void zb2ChangeEdge(b2Body* body, float x1, float y1, float x2, float y2) {
	((b2EdgeShape*) body->GetFixtureList()->GetShape())->Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
}

// Joints

EXPORT b2Joint* zb2CreateDistanceJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy, float length,
	float frequency, float dampingRatio, bool isCollided) {

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

EXPORT b2Joint* zb2CreateDistanceJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy, float length,
	float frequency, float dampingRatio, bool isCollided) {

	b2DistanceJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(anchorAx, anchorAy), b2Vec2(anchorBx, anchorBy));
	jointDef.length = length;
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

EXPORT void zb2UpdateDistanceJoint(b2DistanceJoint* joint, float length,
	float frequency, float dampingRatio) {

	joint->SetLength(length);
}

EXPORT b2Joint* zb2CreatePrismaticJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float axisAx, float axisAy, float referenceAngle,
	bool enableLimit, float lower, float upper,
	bool enableMotor, float maxMotorForce, float motorSpeed, bool isCollided) {

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

EXPORT b2Joint* zb2CreatePrismaticJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY, float axisX, float axisY,
	bool enableLimit, float lower, float upper,
	bool enableMotor, float maxMotorForce, float motorSpeed, bool isCollided) {

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

EXPORT void zb2UpdatePrismaticJoint(b2PrismaticJoint* joint,
	bool enableLimit, float lower, float upper,
	bool enableMotor, float32 maxMotorForce, float motorSpeed) {

	joint->EnableLimit(enableLimit);
	joint->SetLimits(lower, upper);
	joint->EnableMotor(enableMotor);
	joint->SetMaxMotorForce(maxMotorForce);
	joint->SetMotorSpeed(motorSpeed);
}

EXPORT float zb2GetPrismaticTranslation(b2PrismaticJoint* joint) {
	return joint->GetJointTranslation();
}

EXPORT float zb2GetPrismaticSpeed(b2PrismaticJoint* joint) {
	return joint->GetJointSpeed();
}

EXPORT b2Joint* zb2CreateRevoluteJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float referenceAngle, bool enableLimit, float lowerAngle, float upperAngle,
	bool enableMotor, float motorSpeed, float maxMotorTorque, bool isCollided) {

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

EXPORT b2Joint* zb2CreateRevoluteJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY, bool enableLimit, float lowerAngle, float upperAngle,
	bool enableMotor, float motorSpeed, float maxMotorTorque, bool isCollided) {

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

EXPORT void zb2UpdateRevoluteJoint(b2RevoluteJoint* joint,
	bool enableLimit, float lowerAngle, float upperAngle,
	bool enableMotor, float motorSpeed, float maxMotorTorque) {

	joint->EnableLimit(enableLimit);
	joint->SetLimits(angleToRadians(lowerAngle), angleToRadians(upperAngle));
	joint->EnableMotor(enableMotor);
	joint->SetMotorSpeed(motorSpeed);
	joint->SetMaxMotorTorque(maxMotorTorque);
}

EXPORT float zb2GetRevoluteAngle(b2RevoluteJoint* joint) {
	return angleFromRadians(joint->GetJointAngle());
}

EXPORT b2Joint* zb2CreateWeldJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float referenceAngle, float frequency, float dampingRatio, bool isCollided) {

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

EXPORT b2Joint* zb2CreateWeldJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY, float frequency, float dampingRatio,
	bool isCollided) {

	b2WeldJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(anchorX, anchorY));
	jointDef.frequencyHz = frequency;
	jointDef.dampingRatio = dampingRatio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

EXPORT void zb2UpdateWeldJoint(b2WeldJoint* joint, float frequency, float dampingRatio) {
	joint->SetFrequency(frequency);
	joint->SetDampingRatio(dampingRatio);
}

EXPORT b2Joint* zb2CreateRopeJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float maxLength, bool isCollided) {

	b2RopeJointDef jointDef;
	jointDef.bodyA = bodyA;
	jointDef.bodyB = bodyB;
	jointDef.localAnchorA = b2Vec2(anchorAx, anchorAy);
	jointDef.localAnchorB = b2Vec2(anchorBx, anchorBy);
	jointDef.maxLength = maxLength;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

EXPORT void zb2UpdateRopeJoint(b2RopeJoint* joint, float maxLength) {
	joint->SetMaxLength(maxLength);
}

EXPORT b2Joint* zb2CreatePulleyJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float groundAx, float groundAy, float groundBx, float groundBy,
	float lengthA, float lengthB, float ratio, bool isCollided) {

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

EXPORT b2Joint* zb2CreatePulleyJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float groundAx, float groundAy, float groundBx, float groundBy,
	float ratio, bool isCollided) {

	b2PulleyJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB, b2Vec2(groundAx, groundAy), b2Vec2(groundBx, groundBy),
		b2Vec2(anchorAx, anchorAy), b2Vec2(anchorBx, anchorBy), ratio);
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

EXPORT float zb2GetPulleyLengthA(b2PulleyJoint* joint) {
	return joint->GetLengthA();
}

EXPORT float zb2GetPulleyLengthB(b2PulleyJoint* joint) {
	return joint->GetLengthB();
}

EXPORT b2Joint* zb2CreateGearJoint(b2Joint* joint1, b2Joint* joint2,
	float ratio, bool isCollided) {

	b2GearJointDef jointDef;
	jointDef.joint1 = joint1;
	jointDef.joint2 = joint2;
	jointDef.ratio = ratio;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

EXPORT void zb2UpdateGearJoint(b2GearJoint* joint, float ratio) {
	joint->SetRatio(ratio);
}

EXPORT b2Joint* zb2CreateWheelJoint(b2Body* bodyA, b2Body* bodyB,
	float anchorAx, float anchorAy, float anchorBx, float anchorBy,
	float axisAx, float axisAy,
	bool enableMotor, float motorSpeed, float maxMotorTorque,
	float frequency, float dampingRatio, bool isCollided) {

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

EXPORT b2Joint* zb2CreateWheelJointWorld(b2Body* bodyA, b2Body* bodyB,
	float anchorX, float anchorY,float axisX, float axisY,
	bool enableMotor, float motorSpeed, float maxMotorTorque,
	float frequency, float dampingRatio, bool isCollided) {

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

EXPORT void zb2UpdateWheelJoint(b2WheelJoint* joint,
	bool enableMotor, float motorSpeed, float maxMotorTorque,
	float frequency, float dampingRatio) {

	joint->EnableMotor(enableMotor);
	joint->SetMotorSpeed(motorSpeed);
	joint->SetMaxMotorTorque(maxMotorTorque);
	joint->SetSpringFrequencyHz(frequency);
	joint->SetSpringDampingRatio(dampingRatio);
}

EXPORT float zb2GetWheelTranslation(b2WheelJoint* joint) {
	return joint->GetJointTranslation();
}

EXPORT float zb2GetWheelAngle(b2WheelJoint* joint) {
	return angleFromRadians(joint->GetJointAngle());
}

EXPORT float zb2GetWheelLinearSpeed(b2WheelJoint* joint) {
	return joint->GetJointLinearSpeed();
}

EXPORT float zb2GetWheelAngularSpeed(b2WheelJoint* joint) {
	return joint->GetJointAngularSpeed();
}

EXPORT b2Joint* zb2CreateMotorJoint(b2Body* bodyA, b2Body* bodyB,
	float linearOffsetX, float linearOffsetY, float angularOffset,
	float maxForce, float maxTorque, float correctionFactor, bool isCollided) {

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

EXPORT b2Joint* zb2CreateMotorJointWorld(b2Body* bodyA, b2Body* bodyB,
	float maxForce, float maxTorque, float correctionFactor, bool isCollided) {

	b2MotorJointDef jointDef;
	jointDef.Initialize(bodyA, bodyB);
	jointDef.maxForce = maxForce;
	jointDef.maxTorque = maxTorque;
	jointDef.correctionFactor = correctionFactor;
	jointDef.collideConnected = isCollided;

	return g_World->CreateJoint(&jointDef);
}

EXPORT void zb2UpdateMotorJoint(b2MotorJoint* joint,
	float linearOffsetX, float linearOffsetY, float angularOffset,
	float maxForce, float maxTorque, float correctionFactor) {

	joint->SetLinearOffset(b2Vec2(linearOffsetX, linearOffsetY));
	joint->SetAngularOffset(angularOffset);
	joint->SetMaxForce(maxForce);
	joint->SetMaxTorque(maxTorque);
	joint->SetCorrectionFactor(correctionFactor);
}

EXPORT b2Joint* zb2CreateMouseJoint(b2Body* body, float targetX, float targetY,
	float maxForce,	float frequency, float dampingRatio) {

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

EXPORT void zb2UpdateMouseJoint(b2MouseJoint* joint, float targetX, float targetY) {
	joint->SetTarget(b2Vec2(targetX, targetY));
}

EXPORT void zb2DestroyJoint(b2Joint* joint) {
	g_World->DestroyJoint(joint);
}

EXPORT void zb2GetJoints(b2Body* body, b2Joint** jointList, int &count) {

	int i = 0;
	for(b2JointEdge* je = body->GetJointList(); je; je = je->next)
		jointList[i++] = je->joint;

	count = i;
}

// Custom user data

EXPORT void zb2SetUserModelToBody(b2Body* body, void* userModel) {
	body->SetUserData(userModel);
}

EXPORT void zb2SetUserModelToJoint(b2Joint* joint, void* userModel) {
	joint->SetUserData(userModel);
}

EXPORT void* zb2GetUserModelFromBody(b2Body* body) {
	return body->GetUserData();
}

EXPORT void* zb2GetUserModelFromJoint(b2Joint* joint) {
	return joint->GetUserData();
}

// Queries

EXPORT int zb2TestPoint(b2Body* body, float x, float y) {

	bool result = false;
	b2Vec2 point(x, y);
	b2Fixture* fixture = body->GetFixtureList();

	// iterate through fixtures until point is inside
	for(; fixture && !result; result = fixture->TestPoint(point), fixture = fixture->GetNext());

	return result ? TRUE : FALSE;
}

EXPORT int zb2GetBodyAtPoint(float x, float y, b2Body* &body) {
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

EXPORT b2Body* zb2RayCast(float x1, float y1, float x2, float y2,
	float &x, float &y, float &normalX, float &normalY) {

	g_RayCastCallback.Init();
	g_World->RayCast(&g_RayCastCallback, b2Vec2(x1, y1), b2Vec2(x2, y2));

	if(g_RayCastCallback.m_body) {
		x = g_RayCastCallback.m_point.x;
		y = g_RayCastCallback.m_point.y;
		normalX = g_RayCastCallback.m_normal.x;
		normalY = g_RayCastCallback.m_normal.y;
	}

	return g_RayCastCallback.m_body;
}

// Collisions

EXPORT void zb2ResetContacts() {
	g_CurrentContactPoint = 0;
}

// return 0 - no other contact; 1 - next contact returned
EXPORT int zb2GetNextContact(b2Body* &bodyA, b2Body* &bodyB, float &posX, float &posY,
	float &normX, float &normY, int &state) {

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

EXPORT int zb2GetContactCount() {
	return g_ContactListener.m_pointCount;
}

EXPORT int zb2IsCollided(b2Body* body) {
	b2ContactEdge* c = body->GetContactList();

	for(; c; c = c->next)
		if(c->contact->IsTouching()) return TRUE;

	return FALSE;
}

EXPORT int zb2GetBodyContactCount(b2Body* body) {
	b2ContactEdge* c = body->GetContactList();
	int i = 0;

	for(; c; c = c->next)
		if(c->contact->IsTouching()) ++i;

	return i;
}

EXPORT int zb2GetNextCollidedBody(b2Body* body, b2Body* &other, int &getFirst) {

	if(getFirst)
		g_CurrentContactEdge = body->GetContactList();
	else
		g_CurrentContactEdge = g_CurrentContactEdge->next;

	for(; g_CurrentContactEdge; g_CurrentContactEdge = g_CurrentContactEdge->next)
		if(g_CurrentContactEdge->contact->IsTouching()) break;

	getFirst = FALSE;
	other = g_CurrentContactEdge ? g_CurrentContactEdge->other : nullptr;

	return g_CurrentContactEdge != nullptr ? TRUE : FALSE;
}

EXPORT void zb2SetBodyFilteringFlags(b2Body* body, int categoryBits,
	int maskBits, int groupIndex) {

	b2Filter filter;
	filter.categoryBits = categoryBits;
	filter.maskBits = maskBits;
	filter.groupIndex = groupIndex;

	b2Fixture* fixture = body->GetFixtureList();

	for(; fixture; fixture = fixture->GetNext())
		fixture->SetFilterData(filter);
}