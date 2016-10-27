LOCAL_PATH := $(call my-dir)/../..

include $(CLEAR_VARS)

LOCAL_MODULE := ZgeBox2D

MY_BOX2D_PATH := Box2D/Box2D
LOCAL_C_INCLUDES := $(LOCAL_PATH)/$(MY_BOX2D_PATH)
LOCAL_CPPFLAGS += -fexceptions -frtti -std=c++11
LOCAL_LDLIBS := -L$(SYSROOT)/usr/lib -ldl -lm -lstdc++
LOCAL_CPP_FEATURES += exceptions

APP_OPTIM := release

# uncomment for ARMv7-A
LOCAL_CFLAGS := -march=armv7-a -mfloat-abi=softfp

TARGET_PLATFORM := android-8

LOCAL_SRC_FILES := \
	src/ZgeBox2D.cpp\
	$(MY_BOX2D_PATH)/Box2D/Common/b2BlockAllocator.cpp\
	$(MY_BOX2D_PATH)/Box2D/Common/b2Draw.cpp\
	$(MY_BOX2D_PATH)/Box2D/Common/b2Math.cpp\
	$(MY_BOX2D_PATH)/Box2D/Common/b2Settings.cpp\
	$(MY_BOX2D_PATH)/Box2D/Common/b2StackAllocator.cpp\
	$(MY_BOX2D_PATH)/Box2D/Common/b2Timer.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/Shapes/b2ChainShape.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/Shapes/b2CircleShape.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/Shapes/b2EdgeShape.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/Shapes/b2PolygonShape.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2BroadPhase.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2CollideCircle.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2CollideEdge.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2CollidePolygon.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2Collision.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2Distance.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2DynamicTree.cpp\
	$(MY_BOX2D_PATH)/Box2D/Collision/b2TimeOfImpact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2CircleContact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2Contact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2ContactSolver.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Contacts/b2PolygonContact.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2DistanceJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2FrictionJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2GearJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2Joint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2MotorJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2MouseJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2PrismaticJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2PulleyJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2RevoluteJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2RopeJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2WeldJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/Joints/b2WheelJoint.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/b2Body.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/b2ContactManager.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/b2Fixture.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/b2Island.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/b2World.cpp\
	$(MY_BOX2D_PATH)/Box2D/Dynamics/b2WorldCallbacks.cpp\
	$(MY_BOX2D_PATH)/Box2D/Rope/b2Rope.cpp

include $(BUILD_SHARED_LIBRARY)
