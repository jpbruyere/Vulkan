/*
* Vulkan Example - Physical based rendering with image based lighting
*
* Note: Requires the separate asset pack (see data/README.md)
*
* Copyright (C) 2016-2017 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

// For reference see http://blog.selfshadow.com/publications/s2013-shading-course/karis/s2013_pbs_epic_notes_v2.pdf

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <vector>
#include <map>
#include <chrono>
#include <sys/time.h>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <gli/gli.hpp>

#include <vulkan/vulkan.h>
#include "base.h"
#include "VulkanBuffer.hpp"
#include "VulkanTexture.hpp"
#include "VulkanModel.hpp"
#include "ModelGroup.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#include "btvkdebugdrawer.h"

#define ENABLE_VALIDATION false

#define BUMP_BDY_ID 1
#define BALL_BDY_ID 2
#define DAMP_BDY_ID 3
#define TARG_BDY_ID 4
#define DOOR_BDI_ID 5
#define SPIN_BDI_ID 6


constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

std::map<const btCollisionObject*,std::vector<btManifoldPoint*>> objectsCollisions;

void myTickCallback(btDynamicsWorld *dynamicsWorld, btScalar timeStep) {
   objectsCollisions.clear();
   int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
   for (int i = 0; i < numManifolds; i++) {
       btPersistentManifold *contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
       const btCollisionObject *objA = contactManifold->getBody0();
       if (objA->getUserIndex()==0)
           continue;
       const btCollisionObject *objB = contactManifold->getBody1();
       if (objB->getUserIndex()==0)
           continue;
       std::vector<btManifoldPoint*>& collisionsA = objectsCollisions[objA];
       std::vector<btManifoldPoint*>& collisionsB = objectsCollisions[objB];
       int numContacts = contactManifold->getNumContacts();
       for (int j = 0; j < numContacts; j++) {
           btManifoldPoint& pt = contactManifold->getContactPoint(j);
           collisionsA.push_back(&pt);
           collisionsB.push_back(&pt);
       }
   }
}

/*void customNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo)
{
        btCollisionObject* colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
        btCollisionObject* colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

        if (dispatcher.needsCollision(colObj0,colObj1))
        {
            //dispatcher will keep algorithms persistent in the collision pair
            if (!collisionPair.m_algorithm)

                collisionPair.m_algorithm = dispatcher.findAlgorithm (colObj0,colObj1, dispatcher.getManifoldByIndexInternal(0), BT_CONTACT_POINT_ALGORITHMS);
            }

            if (collisionPair.m_algorithm)
            {
                btManifoldResult contactPointResult(colObj0,colObj1);

                if (dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
                {
                    //discrete collision detection query
                    collisionPair.m_algorithm->processCollision(colObj0,colObj1,dispatchInfo,&contactPointResult);
                } else
                {
                    //continuous collision detection query, time of impact (toi)
                    float toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
                    if (dispatchInfo.m_timeOfImpact > toi)
                        dispatchInfo.m_timeOfImpact = toi;

                }
            }
        }

}*/
void customNearCallback(btBroadphasePair& collisionPair,
  btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {

    // Do your collision logic here
    btCollisionObject* colObj0 = (btCollisionObject*)collisionPair.m_pProxy0->m_clientObject;
    btCollisionObject* colObj1 = (btCollisionObject*)collisionPair.m_pProxy1->m_clientObject;

    if (dispatcher.needsCollision(colObj0,colObj1))
    {
        btCollisionObjectWrapper obj0Wrap(0,colObj0->getCollisionShape(),colObj0,colObj0->getWorldTransform(),-1,-1);
        btCollisionObjectWrapper obj1Wrap(0,colObj1->getCollisionShape(),colObj1,colObj1->getWorldTransform(),-1,-1);


        //dispatcher will keep algorithms persistent in the collision pair
        if (!collisionPair.m_algorithm)
        {
            collisionPair.m_algorithm = dispatcher.findAlgorithm(&obj0Wrap,&obj1Wrap,0, BT_CONTACT_POINT_ALGORITHMS);
        }

        if (collisionPair.m_algorithm)
        {
            btManifoldResult contactPointResult(&obj0Wrap,&obj1Wrap);

            if (dispatchInfo.m_dispatchFunc == 		btDispatcherInfo::DISPATCH_DISCRETE)
            {
                //discrete collision detection query

                collisionPair.m_algorithm->processCollision(&obj0Wrap,&obj1Wrap,dispatchInfo,&contactPointResult);
            } else
            {
                //continuous collision detection query, time of impact (toi)
                btScalar toi = collisionPair.m_algorithm->calculateTimeOfImpact(colObj0,colObj1,dispatchInfo,&contactPointResult);
                if (dispatchInfo.m_timeOfImpact > toi)
                    dispatchInfo.m_timeOfImpact = toi;

            }
        }
    }
    // Only dispatch the Bullet collision information if you want the physics to continue
    //dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}
void check_collisions (btDynamicsWorld *dynamicsWorld, void* vkapp);

float flipperStrength = 0.0005f;
float damperStrength = 0.04f;
float bumperStrength = 0.02f;


class VulkanExample : public VulkanExampleVk
{
public:
    struct MovingObject {
        btRigidBody* body = nullptr;
        uint32_t instanceIdx;
    } worldObjs [15];

    struct StaticObject {
        btCollisionShape* shape = nullptr;
        btRigidBody* body = nullptr;
    };
    struct Target {
        vks::ModelGroup::InstanceData* pInstance;
        bool state; //true when reached
        uint points; //points when hit
        btRigidBody* body;
    };

    class TargetGroup
    {
    public:
         std::vector<Target> targets;
         uint32_t targetCount = 0;
         clock_t reachedTime;
         float resetDelay = 1;  //reset delay in seconds
         int reachedTarget = 0; //number of reached target

         void tryReset (clock_t curTime) {
             if (diffclock(curTime, reachedTime) < resetDelay)
                 return;
             for(std::vector<Target>::iterator it = targets.begin(); it != targets.end(); ++it) {
                 it->pInstance->modelMat = glm::mat4();
                 it->body->setActivationState(ACTIVE_TAG);
                 it->state = false;
             }
             reachedTarget = 0;
         }
         void addTarget (Target _target) {
             targets.push_back(_target);
             targetCount++;
         }
         TargetGroup() {}
    };


    VulkanExample() : VulkanExampleVk(ENABLE_VALIDATION)
    {
        title = "PBR with image based lighting";

        camera.type = Camera::CameraType::firstperson;
        camera.movementSpeed = 1.0f;
        camera.setPerspective(60.0f, (float)width / (float)height, 0.1f, 256.0f);
        camera.rotationSpeed = 0.05f;

        camera.setRotation({ -35.f, 180.0f, 0.9f });
        //camera.setRotation({ -89.f, 180.0f, 0.9f });
        camera.setPosition({ 0.f, 0.3f, 0.64f });
        //camera.setPosition({-0.0920,0.2,0.3980});
        //camera.setPosition({0.0,0.4,0.});

        settings.overlay = true;
    }

    ~VulkanExample()
    {
        delete dynamicsWorld;
        delete solver;
        delete dispatcher;
        delete collisionConfiguration;
        delete broadphase;

        //models->instanceBuff.unmap();
        modGrp->destroy();
        delete modGrp;
    }

    uint32_t texSize = 1024;

    int worldObjBall = 0;
    int worldObjFlip = 1;
    int worldObjDoor1 = 4;
    int worldObjSpinner = 5;

    //instance indices of static object that need an initial transformation
    //  equal to their physical counterpart matrix
    int instSpinnerFrameIdx;
    int instBumperIdx;

    float ballSize  = 0.027;
    float sloap     = 7.f * M_PI/180.0f;

    btVector3 upVector = btVector3(0, cos(sloap), sin(sloap));
    btVector3 pushDir = btVector3(0,0,-1);

    float   iterations          = 10;
    float   timeStep            = 1.0f / 200;
    float   fixedTimeStepDiv    = 1.0f / 600.0;
    float   maxSubsteps         = 10.f;
    int     subSteps            = 0;

    bool    splitImpulse        = false;
    const float inpulse     = 2.f;
    const int   target      = 2;
    bool        leftFlip    = false;
    bool        rightFlip   = false;


    btBroadphaseInterface*              broadphase = nullptr;
    btDefaultCollisionConfiguration*    collisionConfiguration = nullptr;
    btCollisionDispatcher*              dispatcher = nullptr;
    btSequentialImpulseConstraintSolver* solver = nullptr;
    btDiscreteDynamicsWorld*            dynamicsWorld = nullptr;

    btVKDebugDrawer* debugDrawer;

    std::vector<btRigidBody*> btStaticObjs;

    int movingObjectCount = 6;

    TargetGroup leftTargetGroup;

    uint player_points = 0;
    uint bumper_points = 100;
    uint damper_points = 10;
    uint sinner_points = 1;
    uint leftTarget_points = 500;


    void loadAssets()
    {
        VulkanExampleVk::loadAssets();

        int modBallIdx = -1, modTestIdx = -1, modFlipIdx = -1, modStaticsIdx = -1,
        modTargetsIdx = -1, modDoor1Idx = -1;

        modBallIdx = modGrp->addModel(getAssetPath() + "models/geosphere1.obj", ballSize);
        //modFlipIdx = modGrp->addModel(getAssetPath() + "models/pinball-flip-hr.obj");
        modFlipIdx  = modGrp->addModel(getAssetPath() + "models/pinball-flip.obj");
        modDoor1Idx = modGrp->addModel(getAssetPath() + "models/pinball-door1.obj");
        modTargetsIdx = modGrp->addModel(getAssetPath() + "models/pinball-target-hr.obj");
        //modTestIdx = modGrp->addModel(getAssetPath() + "models/pinball.obj");
        modStaticsIdx = modGrp->addModel(getAssetPath() + "models/pinball-static-hr.obj");

        std::vector<std::string> mapDic = {
            getAssetPath() + "pinball-back.png",
            getAssetPath() + "flipper.png",
            getAssetPath() + "lcd.png",
            getAssetPath() + "test.png",
        };

        modGrp->addMaterial (0.9f,0.9f,0.9f,1, 0.9f,0.16f,0.f);
        modGrp->addMaterial (0, 0.001f, 0.9f, 0);
        modGrp->addMaterial (1, 0.3f, 0.4f,0);
        modGrp->addMaterial (0.749585f, 0.756114f, 0.754256f, 1.0f, 0.7f, 0.02f, 0);
        modGrp->addMaterial (3, 0.f, 0.8f, 0);
        //modGrp->addMaterial(1.0f,0.0f,1.0f,1.0f, 1.f, 1.f, 0);

        worldObjs[worldObjBall].instanceIdx = modGrp->addInstance(modBallIdx, 0);

        worldObjs[worldObjFlip].instanceIdx = modGrp->addInstance(modFlipIdx, 2);
        worldObjs[worldObjFlip+1].instanceIdx = modGrp->addInstance(modFlipIdx, 2);
        worldObjs[worldObjFlip+2].instanceIdx = modGrp->addInstance(modFlipIdx, 2);

        worldObjs[worldObjDoor1].instanceIdx = modGrp->addInstance(modDoor1Idx, 0);

        int modTargetFirstInstanceIdx = modGrp->addInstance(modTargetsIdx, 2);

        //modGrp->addInstance(modPlateIdx, 1);

        for (int i = 0; i < modGrp->models[modStaticsIdx].parts.size(); i++){
            switch (str2int(modGrp->models[modStaticsIdx].parts[i].name.c_str())) {
            case str2int("spinner-hr_Cube.014"):
                worldObjs[worldObjSpinner].instanceIdx =
                        modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 0);
                break;
            case str2int("SpinnerBord_Cube.010"):
                instSpinnerFrameIdx =
                        modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 1);
                break;
            case str2int("bumper_Circle.115"):
                instBumperIdx =
                        modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 2);
                modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 2);
                modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 2);
                break;
            case str2int("left_damper_Plane.028"):
            case str2int("right_damper_Plane.029"):
                modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 2);
                break;
            case str2int("guides_NurbsPath.006"):
                modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 3);
                break;
            case str2int("BezierCurve_BezierCurve.005"):
            case str2int("BezierCurve.001_BezierCurve.007"):
                modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 4);
                break;
            default:
                modGrp->addInstance (modStaticsIdx, i, glm::mat4(), 1);
                break;
            }

        }

//        worldObjs[6].instanceIdx = modGrp->addInstance(modTestIdx, 0);
//        worldObjs[7].instanceIdx = worldObjs[6].instanceIdx + 1;

        modGrp->prepare(mapDic, texSize);

        for (int i = 0; i < 4; i++)
            leftTargetGroup.addTarget({&modGrp->instanceDatas[modTargetFirstInstanceIdx+i], false, leftTarget_points * i, nullptr});

        init_physics();
    }

    void preparePipelines() {
        VulkanExampleVk::preparePipelines();

        debugDrawer = new btVKDebugDrawer(vulkanDevice, renderPass, sampleCount, descriptorSetLayout);
        debugDrawer->setDebugMode(
                    btIDebugDraw::DBG_DrawConstraintLimits|
                    btIDebugDraw::DBG_DrawFrames|
                    btIDebugDraw::DBG_DrawContactPoints|
                    btIDebugDraw::DBG_DrawConstraints|
                    btIDebugDraw::DBG_DrawWireframe);

        dynamicsWorld->setDebugDrawer (debugDrawer);


    }
    glm::mat4 btTransformToGlmMat (const btTransform &trans){
        btVector3 o = trans.getOrigin();

        btQuaternion btQ = trans.getRotation();
        glm::quat q = glm::quat (btQ.getW(), btQ.getX(), btQ.getY(), btQ.getZ());

        return glm::translate(glm::mat4(),glm::vec3(-o.getX(), -o.getY(), -o.getZ())) * glm::mat4(q);
    }

    /*void addRigidBody (btCollisionShape* shape, btScalar mass = 0, btTransform transformation){
        btRigidBody* body = nullptr;
        btVector3 fallInertia(0, 0, 0);
        btHingeConstraint* hinge;
        btMotionState* motionState = nullptr;

        if (mass > 0.f){
            shape->calculateLocalInertia(mass, fallInertia);
            motionState = new btDefaultMotionState(transformation);
        }


        btRigidBody::btRigidBodyConstructionInfo rbci(mass, motionState, shape, btVector3(0, 0, 0));
        rbci.m_restitution = 0;
        rbci.m_friction = 1.3;

        body = new btRigidBody(rbci);
    }*/
    void initPhysicalBodies () {
        vks::ModelGroup* modBodies = new vks::ModelGroup(vulkanDevice, queue);

        btCollisionShape* shape = nullptr;
        btRigidBody* body = nullptr;
        int modBodiesIdx = -1;

        //low plane
        shape = new btStaticPlaneShape(upVector, 0);
        shape->setMargin(0.001);

        btRigidBody::btRigidBodyConstructionInfo rbci(0, nullptr, shape, btVector3(0, 0, 0));
        rbci.m_restitution = 0;
        rbci.m_friction = 1.3;
        btStaticObjs.push_back(new btRigidBody(rbci));

        //static bodies
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-static.obj");
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {
            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape , btVector3(0, 0, 0));
            rbci.m_restitution = 0.8;
            rbci.m_friction = 0.2;
            btStaticObjs.push_back (new btRigidBody(rbci));
        }
        //ramp
        /*modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-ramp.obj");
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {
            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape , btVector3(0, 0, 0));
            rbci.m_restitution = 0.;
            rbci.m_friction = 0.;
            btStaticObjs.push_back(new btRigidBody(rbci));
        }*/

        //damper
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-damp.obj");
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {
            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape, btVector3(0, 0, 0));
            rbci.m_restitution = 0.;
            rbci.m_friction = 0.1;

            body = new btRigidBody(rbci);

            dynamicsWorld->addRigidBody(body);
            body->setUserIndex(DAMP_BDY_ID);
        }

        //targets
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-target.obj");
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {
            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape, btVector3(0, 0, 0));
            rbci.m_restitution = 0.5;
            rbci.m_friction = 0.1;

            body = new btRigidBody(rbci);

            dynamicsWorld->addRigidBody(body);
            body->setUserIndex(TARG_BDY_ID);
            body->setUserIndex2(i);
            leftTargetGroup.targets[i].body = body;
        }

        for(std::vector<btRigidBody*>::iterator it = btStaticObjs.begin(); it != btStaticObjs.end(); ++it) {
            dynamicsWorld->addRigidBody ((btRigidBody*)*it,0x02,0x01);
        }

        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-flip.obj");
        btVector3 vUp = btVector3(0,1,0);
        btScalar mass = 0.6;
        btVector3 fallInertia(0, 0, 0);
        btHingeConstraint* hinge;
        btMotionState* motionState;

        //tests
        /*modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball.obj");
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {
            mass = 0.6;
            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            shape->calculateLocalInertia(mass, fallInertia);
            motionState = new btDefaultMotionState(
                        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
            rbci = btRigidBody::btRigidBodyConstructionInfo (mass, motionState, shape , btVector3(0, 0, 0));
            rbci.m_restitution = 0.8;
            rbci.m_friction = 0.2;
            body = new btRigidBody(rbci);
            body->setLinearFactor(btVector3(0,0,1));
            dynamicsWorld->addRigidBody(body,0x02,0x01);
            worldObjs[6+i].body = body;
        }*/

        //flippers
        mass = 0.08;
        shape = modBodies->getConvexHullShape(modBodiesIdx, 0);
        //shape->setMargin(0.005);
        shape->calculateLocalInertia(mass, fallInertia);

        //right
        btVector3 pos = btVector3(0.09194,0.01479,0.39887);
        btTransform trBody = btTransform(btQuaternion(0, 0, 0, 1), pos);
        trBody = btTransform(btQuaternion(btVector3(1,0,0), 7.f * M_PI / 180.0), btVector3(0,0,0)) * trBody;

        motionState = new btDefaultMotionState(trBody);

        rbci = btRigidBody::btRigidBodyConstructionInfo (mass, motionState, shape, fallInertia);
        rbci.m_restitution = 0.8;
        rbci.m_friction = 0.2;

        body = new btRigidBody(rbci);

        //body->setCollisionFlags(128);
        //body->setContactStiffnessAndDamping(9999,10);


        dynamicsWorld->addRigidBody(body,0x02,0x01);

        worldObjs[worldObjFlip].body = body;

        hinge = new btHingeConstraint(*body, btVector3(0,0,0), vUp, false);
        hinge->setLimit (M_PI - 0.6, M_PI + 0.5, 0.001f, 1.0f, -0.001f);
        dynamicsWorld->addConstraint(hinge);

        //left
        pos = btVector3(-0.09194,0.01479,0.39887);
        trBody = btTransform(btQuaternion(0, 0, 0, 1), pos);
        trBody = btTransform(btQuaternion(btVector3(1,0,0), 7.f * M_PI / 180.0), btVector3(0,0,0)) * trBody;

        motionState = new btDefaultMotionState(trBody);

        rbci = btRigidBody::btRigidBodyConstructionInfo (mass, motionState, shape, fallInertia);
        rbci.m_restitution = 0.;
        rbci.m_friction = 0.2;
        body = new btRigidBody(rbci);

        dynamicsWorld->addRigidBody(body,0x02,0x01);

        hinge = new btHingeConstraint (*body, btVector3(0,0,0), vUp, false);
        hinge->setLimit (-0.5, 0.6, .001f,1.0f,-0.001f);
        dynamicsWorld->addConstraint(hinge);

        worldObjs[worldObjFlip+1].body = body;

        //top left
        shape = modBodies->getConvexHullShape(modBodiesIdx, 0, 0.92f);
        //shape->setLocalScaling(btVector3(0.92,0.92,0.92));
        shape->setMargin(0.00);
        shape->calculateLocalInertia(mass, fallInertia);

        pos = btVector3(-0.22988,0.01479,-0.07538);

        trBody = btTransform(btQuaternion(0, 0, 0, 1), pos);
        trBody = btTransform(btQuaternion(btVector3(1,0,0), 7.f * M_PI / 180.0), btVector3(0,0,0)) * trBody;

        motionState = new btDefaultMotionState(trBody);

        rbci = btRigidBody::btRigidBodyConstructionInfo (mass, motionState, shape, fallInertia);
        rbci.m_restitution = 0.;
        rbci.m_friction = 0.2;
        body = new btRigidBody(rbci);

        dynamicsWorld->addRigidBody(body,0x02,0x01);

        hinge = new btHingeConstraint (*body, btVector3(0,0,0), vUp, false);
        hinge->setLimit (-1.20, 0., .001f,1.0f,-0.001f);
        dynamicsWorld->addConstraint(hinge);

        worldObjs[worldObjFlip+2].body = body;

        //door 1
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-door1.obj");

        shape = modBodies->getConvexHullShape(modBodiesIdx, 0);
        mass = 0.003;
        shape->calculateLocalInertia(mass, fallInertia);

        pos = btVector3(0.28105,0,-0.27905);
        trBody = btTransform(btQuaternion(0, 0, 0, 1), pos);
        trBody = btTransform(btQuaternion(btVector3(1,0,0), 7.f * M_PI / 180.0), btVector3(0,0,0)) * trBody;

        motionState = new btDefaultMotionState(trBody);

        rbci = btRigidBody::btRigidBodyConstructionInfo (mass, motionState, shape, fallInertia);
        rbci.m_restitution = 0.;
        rbci.m_friction = 0.01;
        body = new btRigidBody(rbci);
        body->setUserIndex(DOOR_BDI_ID);
        body->setUserIndex2(1);

        dynamicsWorld->addRigidBody(body,0x02,0x01);

        hinge = new btHingeConstraint (*body, btVector3(0,0,0), vUp, false);
        hinge->setLimit (0.148, 1., 0.01f,1.f,-0.001f);
        dynamicsWorld->addConstraint(hinge);

        worldObjs[worldObjDoor1].body = body;

        //spinner
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-spinner.obj");

        shape = modBodies->getConvexHullShape(modBodiesIdx, 0);
        mass = 0.01;
        shape->calculateLocalInertia(mass, fallInertia);

        pos = btVector3(0.24554,0.02731,-0.13162);
        trBody = btTransform(btQuaternion(btVector3(0,1,0), -21.f * M_PI / 180.0), pos);
        trBody = btTransform(btQuaternion(btVector3(1,0,0), 7.f * M_PI / 180.0), btVector3(0,0,0)) * trBody;

        motionState = new btDefaultMotionState(trBody);

        rbci = btRigidBody::btRigidBodyConstructionInfo (mass, motionState, shape, fallInertia);
        rbci.m_restitution = 0.;
        rbci.m_friction = 0.01;

        body = new btRigidBody(rbci);
        body->setUserIndex(SPIN_BDI_ID);
        body->setUserIndex2(1);

        dynamicsWorld->addRigidBody(body,0x02,0x01);

        hinge = new btHingeConstraint (*body, btVector3(0,0.006,0), btVector3(1,0,0), false);
        hinge->setLimit (0., 2 * M_PI, 0.1f, 0.f,-1.0f);
        dynamicsWorld->addConstraint(hinge);

        worldObjs[worldObjSpinner].body = body;
        //move spinner frame which is static, not move by physic update
        modGrp->instanceDatas [instSpinnerFrameIdx].modelMat = btTransformToGlmMat (trBody);

        //bumpers
        btVector3 bumperPos[] = {
            btVector3(-0.075, 0.0,-0.317),
            btVector3(0.086, 0.0,-0.354),
            btVector3(0.011, 0.0,-0.233),
        };
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-bump.obj");
        shape = modBodies->getConvexHullShape(modBodiesIdx, 0);
        for (int i = 0; i<3 ; i++) {
            pos = bumperPos[i];
            trBody = btTransform(btQuaternion(0, 0, 0, 1), pos);
            trBody = btTransform(btQuaternion(btVector3(1,0,0), 7.f * M_PI / 180.0), btVector3(0,0,0)) * trBody;

            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape, btVector3(0, 0, 0));
            rbci.m_restitution = 1.2;
            rbci.m_friction = 0.;

            body = new btRigidBody(rbci);
            body->setWorldTransform (trBody);

            dynamicsWorld->addRigidBody(body);
            body->setUserIndex(BUMP_BDY_ID);

            modGrp->instanceDatas [instBumperIdx+i].modelMat = btTransformToGlmMat (trBody);
        }

        //balls
        shape = new btSphereShape(ballSize * 0.5);
        //shape->setMargin(0.0002);
        mass = 0.08;
        shape->calculateLocalInertia(mass, fallInertia);

        for (int i=0; i < 1; i++) {
            motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0,0,0)));

            rbci = btRigidBody::btRigidBodyConstructionInfo (mass, motionState, shape, fallInertia);
            rbci .m_restitution = 0.5f;
            rbci .m_friction = 0.2;

            //ballRigidBodyCI.m_rollingFriction = 0.000001;
            body = new btRigidBody (rbci);
            dynamicsWorld->addRigidBody (body,0x01,0xff);

            body->setUserIndex             (BALL_BDY_ID);
            body->setFriction              (0.01f);
            body->setRollingFriction       (.00001);
            body->setSpinningFriction      (0.1);
            body->setAnisotropicFriction   (shape->getAnisotropicRollingFrictionDirection(),
                                                         btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
            body->setCcdMotionThreshold    (ballSize * 0.5f);
            body->setCcdSweptSphereRadius  (ballSize * 0.1f);

            worldObjs[worldObjBall].body = body;
        }


        modBodies->destroy();
        delete(modBodies);
    }


    void init_physics() {
        broadphase = new btDbvtBroadphase();

        collisionConfiguration = new btDefaultCollisionConfiguration();

        dispatcher = new btCollisionDispatcher(collisionConfiguration);
        //dispatcher->setNearCallback(customNearCallback);

        btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

        solver = new btSequentialImpulseConstraintSolver;

        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
        dynamicsWorld->setGravity(btVector3(0, -10, 0));

        dynamicsWorld->getSolverInfo().m_erp2 = 0.f;
        dynamicsWorld->getSolverInfo().m_globalCfm = 0.f;
        //dynamicsWorld->getDispatchInfo().m_dispatchFunc = btDispatcherInfo::DISPATCH_CONTINUOUS;

        dynamicsWorld->setInternalTickCallback(myTickCallback);

        btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
        info.m_numIterations = int(iterations);
        info.m_splitImpulse = int(splitImpulse);

        initPhysicalBodies();

    }

    void update_physics () {

        for (int i=0; i < movingObjectCount; i++) {
            btTransform trans;
            worldObjs[i].body->getMotionState()->getWorldTransform(trans);
            modGrp->instanceDatas[worldObjs[i].instanceIdx].modelMat = btTransformToGlmMat(trans);
        }
        //scale top left flipper
        modGrp->instanceDatas[worldObjFlip+2].modelMat = modGrp->instanceDatas[worldObjFlip+2].modelMat *
                glm::scale(glm::mat4(), glm::vec3(0.92,0.92,0.92));

        modGrp->updateInstancesBuffer();
    }

    clock_t lastTime;

    void step_physics () {
//        btTransform trA = worldObjs[2].body->getWorldTransform();
//        btVector3 hingeAxisInWorld = trA.getBasis()*hingeL->getFrameOffsetA().getBasis().getColumn(2);
        worldObjs[worldObjDoor1].body->applyTorqueImpulse(btVector3(0,1,0)* -0.0001);
        btVector3 avSpinner = worldObjs[worldObjSpinner].body->getAngularVelocity();
        if (avSpinner.getX() != 0) {

            avSpinner.setX(avSpinner.getX()*0.999);
            worldObjs[worldObjSpinner].body->setAngularVelocity (avSpinner);
        }

        if (leftFlip) {
            //hingeL->getRigidBodyA().applyTorque(hingeAxisInWorld * flipperStrength);

            //worldObjs[2].body->applyTorque(upVector * flipperStrength);
            worldObjs[worldObjFlip+1].body->applyTorqueImpulse(btVector3(0,1,0)* flipperStrength);
            worldObjs[worldObjFlip+2].body->applyTorqueImpulse(btVector3(0,1,0)* flipperStrength);
        }else {
            //hingeL->getRigidBodyA().applyTorque(hingeAxisInWorld * -flipperStrength);

            //worldObjs[2].body->applyTorque(btVector3(0,-1,0)*multi);
            worldObjs[worldObjFlip+1].body->applyTorqueImpulse(btVector3(0,-1,0)* flipperStrength);
            worldObjs[worldObjFlip+2].body->applyTorqueImpulse(btVector3(0,-1,0)* flipperStrength);
            //worldObjs[2].body->applyTorque(-upVector * flipperStrength);
        }

//        trA = worldObjs[1].body->getWorldTransform();
//        hingeAxisInWorld = trA.getBasis()*hingeR->getFrameOffsetA().getBasis().getColumn(2);

        if (rightFlip) {
            worldObjs[worldObjFlip].body->applyTorqueImpulse(btVector3(0,1,0)* -flipperStrength);
        }else {
            worldObjs[worldObjFlip].body->applyTorqueImpulse(btVector3(0,1,0)* flipperStrength);
        }
        clock_t time = clock();

        float diff = float(diffclock(time, lastTime));

        if (diff > timeStep) {
            lastTime = time;
            dynamicsWorld->updateAabbs();
            subSteps = dynamicsWorld->stepSimulation(diff,int(maxSubsteps),fixedTimeStepDiv);
            update_physics();
            check_collisions(dynamicsWorld, this);

            if (leftTargetGroup.reachedTarget == leftTargetGroup.targetCount)
                leftTargetGroup.tryReset(time);
        }

        //dynamicsWorld->stepSimulation(1.0/1600.0,10,1.0/1000.0);


    }

    virtual void keyDown(uint32_t key) {
        switch (key) {
        case 37://left ctrl
            leftFlip = true;
            worldObjs[worldObjFlip+1].body->activate();
            worldObjs[worldObjFlip+2].body->activate();
            break;
        case 105://right ctrl
            rightFlip = true;
            worldObjs[worldObjFlip].body->activate();
            break;
        }
    }
    virtual void keyUp(uint32_t key) {
        switch (key) {
        case 37://left ctrl
            leftFlip = false;
            worldObjs[worldObjFlip+1].body->activate();
            worldObjs[worldObjFlip+2].body->activate();
            break;
        case 105://right ctrl
            rightFlip = false;
            worldObjs[worldObjFlip].body->activate();
            break;
        }
    }
    virtual void keyPressed(uint32_t key) {
        switch (key) {
//        case 37://left ctrl
//            worldObjs[2].body->clearForces();
//            break;
//        case 105://right ctrl
//            worldObjs[1].body->clearForces();
//            break;
        case 65:
            worldObjs[0].body->setLinearVelocity(worldObjs[0].body->getLinearVelocity() + pushDir);
            worldObjs[0].body->activate();
            break;
        case 79://7
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.085,0.08,-0.431)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 80://8
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.085,0.02,0.331)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 81://9
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.085,0.01,0.365)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 83://4
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.215,0.01,0.17)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 84://5
            //pushDir = btVector3(1,0,0);
            //worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.4457,0.05,0.535)));
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.298,0.05,0.2)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 85://6
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.215,0.01,0.17)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 87://1
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.11,0.02,0.22)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 88:
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.,0.02,0.)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 89://3
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.195,0.025,-0.10)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 90://0
            rebuildCommandBuffers();
            break;
        }
    }
    void additionalDrawCommands(VkCommandBuffer cmd) {
        debugDrawer->buildCommandBuffer (cmd);
    }
    virtual void render()
    {
        if (!prepared)
            return;
        step_physics();
        dynamicsWorld->debugDrawWorld();

        rebuildCommandBuffers();

        draw();
    }

     virtual void OnUpdateUIOverlay(vks::UIOverlay *overlay)
    {
        if (overlay->header("Settings")) {
            if (dynamicsWorld != nullptr) {
                overlay->text("Sub Step: %d", subSteps);
                overlay->inputFloat ("MAX substeps", &maxSubsteps, 1., 0);
                overlay->inputFloat ("Fixed Time DIV", &fixedTimeStepDiv, 10., 0);
                btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
                if (overlay->inputFloat ("Iterations", &iterations, 1., 0)) {
                    info.m_numIterations = int(iterations);
                }
                if (overlay->checkBox("Split impulse", &splitImpulse)) {
                    info.m_splitImpulse = int(splitImpulse);
                }
                /*btVector3 orig = hingeL->getAFrame().getOrigin().getX();
                float f = orig.getX();
                if (overlay->inputFloat ("HingeX", &f, 0.001, 4)) {
                    hingeL->setFrames();*/
            }

            if (overlay->comboBox("Material", &materialIndex, materialNames)) {
                buildCommandBuffers();
            }
//            if (overlay->comboBox("Object type", &models.objectIndex, objectNames)) {
//                updateUniformBuffers();
//                buildCommandBuffers();
//            }
            if (overlay->inputFloat("Exposure", &uboParams.exposure, 0.1f, 2)) {
                updateParams();
            }
            if (modGrp != nullptr){
                if (overlay->inputFloat("plate roughness", &modGrp->materials[15].roughness, 0.01f, 3)) {
                    modGrp->updateMaterialBuffer();
                }
            }

            if (overlay->inputFloat("Gamma", &uboParams.gamma, 0.1f, 2)) {
                updateParams();
            }
            if (overlay->checkBox("Skybox", &displaySkybox)) {
                buildCommandBuffers();
            }
        }
    }

};

void check_collisions (btDynamicsWorld *dynamicsWorld, void *app) {
    VulkanExample* vkapp = (VulkanExample*) app;

    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; --j) {
        btCollisionObject *obj = dynamicsWorld->getCollisionObjectArray()[j];

        btRigidBody *body = btRigidBody::upcast(obj);
        btTransform trans;

        switch (body->getUserIndex()) {
        case BUMP_BDY_ID:
        {
            std::vector<btManifoldPoint*>& manifoldPoints = objectsCollisions[body];
            if (manifoldPoints.size()==0)
                continue;
            /*trans = obj->getWorldTransform();
            btVector3 center;
            btScalar radius;
            obj->getCollisionShape()->getBoundingSphere(center, radius);
            trans = vkapp->worldObjs[0].body->getWorldTransform();
            btVector3 vDir = trans.getOrigin() - center;
            vkapp->worldObjs[0].body->clearForces();
            vkapp->worldObjs[0].body->applyImpulse(-vDir.normalized() * bumperStrength,btVector3(0,0,0));*/
            vkapp->worldObjs[0].body->applyImpulse(-manifoldPoints[0]->m_normalWorldOnB * damperStrength,btVector3(0,0,0));
            vkapp->player_points += vkapp->bumper_points;
            break;
        }
        case DAMP_BDY_ID:
        {
            std::vector<btManifoldPoint*>& manifoldPoints = objectsCollisions[body];
            if (manifoldPoints.size()==0)
                continue;
            //btVector3 vDir = manifoldPoints[0]->getPositionWorldOnA() - manifoldPoints[0]->getPositionWorldOnB();
            vkapp->worldObjs[0].body->applyImpulse(-manifoldPoints[0]->m_normalWorldOnB * damperStrength,btVector3(0,0,0));
            vkapp->player_points += vkapp->damper_points;
            break;
        }
        case TARG_BDY_ID:
        {
            std::vector<btManifoldPoint*>& manifoldPoints = objectsCollisions[body];
            if (manifoldPoints.size()==0)
                continue;
            int tg = body->getUserIndex2();
            if (vkapp->leftTargetGroup.targets[tg].state)
                continue;
            obj->setActivationState(DISABLE_SIMULATION);
            vkapp->leftTargetGroup.targets[tg].state = true;
            vkapp->leftTargetGroup.targets[tg].pInstance->modelMat =
                    glm::translate (glm::mat4(), glm::vec3(0,+0.02,0));
            vkapp->leftTargetGroup.reachedTarget++;
            vkapp->player_points += vkapp->leftTargetGroup.targets[tg].points;
            if (vkapp->leftTargetGroup.reachedTarget == vkapp->leftTargetGroup.targetCount)
                vkapp->leftTargetGroup.reachedTime = clock();
            break;
        }
        /*case DOOR_BDI_ID:
        {
            std::vector<btManifoldPoint*>& manifoldPoints = objectsCollisions[body];
            if (manifoldPoints.size()==0)
                continue;
            body->applyTorqueImpulse(btVector3(0,1,0)* -0.001);

        }*/
        }




//        if (body && body->getMotionState()) {
//            body->getMotionState()->getWorldTransform(trans);
//        } else {
//            trans = obj->getWorldTransform();
//        }
//        btVector3 origin = trans.getOrigin();


    }
}

VULKAN_EXAMPLE_MAIN()
