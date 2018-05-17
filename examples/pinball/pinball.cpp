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
#include "vulkanexamplebase.h"
#include "VulkanBuffer.hpp"
#include "VulkanTexture.hpp"
#include "VulkanModel.hpp"
#include "ModelGroup.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

#define ENABLE_VALIDATION false
#define GRID_DIM 7

#define INSTANCE_COUNT 16

#define BUMP_BDY_ID 1
#define BALL_BDY_ID 2
#define DAMP_BDY_ID 3
#define TARG_BDY_ID 4

struct OldMaterial {
    // Parameter block used as push constant block
    struct PushBlock {
        float roughness = 0.0f;
        float metallic = 0.0f;
        float specular = 0.0f;
        float r, g, b;
    } params;
    std::string name;
    OldMaterial() {};
    OldMaterial(std::string n, glm::vec3 c) : name(n) {
        params.r = c.r;
        params.g = c.g;
        params.b = c.b;
    };
};

std::map<const btCollisionObject*,std::vector<btManifoldPoint*>> objectsCollisions;

double diffclock( clock_t clock1, clock_t clock2 ) {

    double diffticks = clock1 - clock2;
    double diffms    = diffticks / ( CLOCKS_PER_SEC  );

    return diffms;
}

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

class VulkanExample : public VulkanExampleBase
{
public:
    btBroadphaseInterface* broadphase = nullptr;
    btDefaultCollisionConfiguration* collisionConfiguration = nullptr;
    btCollisionDispatcher* dispatcher = nullptr;
    btSequentialImpulseConstraintSolver* solver = nullptr;
    btDiscreteDynamicsWorld* dynamicsWorld = nullptr;


    btCollisionShape*       plateShape = nullptr;
    btRigidBody*            plateRB = nullptr;
    btCollisionShape*       plateShape2 = nullptr;
    btRigidBody*            plateRB2 = nullptr;

    btCollisionShape*       borderShape = nullptr;
    btDefaultMotionState*   borderState = nullptr;
    btRigidBody*            borderRB = nullptr;

    btCollisionShape* ballShape = nullptr;

    int currentGameType = 0;

    struct GameType {
        double tableWidth;
        double tableLength;
        double tableHeight;
        double ballSize;
    }gameTypes [2];

    struct MovingObject {
        btCollisionShape* shape = nullptr;
        btDefaultMotionState* motionState = nullptr;
        btRigidBody* body = nullptr;
    } worldObjs [15];

    struct StaticObject {
        btCollisionShape* shape = nullptr;
        btRigidBody* body = nullptr;
    };

    std::vector<btRigidBody*> btStaticObjs;


    bool displaySkybox = true;

    struct Textures {
        vks::TextureCubeMap environmentCube;
        // Generated at runtime
        vks::Texture2D lutBrdf;
        vks::TextureCubeMap irradianceCube;
        vks::TextureCubeMap prefilteredCube;
    } textures;

    // Vertex layout for the models
    vks::VertexLayout vertexLayout = vks::VertexLayout({
        vks::VERTEX_COMPONENT_POSITION,
        vks::VERTEX_COMPONENT_NORMAL,
        vks::VERTEX_COMPONENT_UV,
    });

    vks::Model skybox;

    struct {
        vks::Buffer matrices;
        vks::Buffer skybox;
        vks::Buffer params;
    } uniformBuffers;

    struct UBOMatrices {
        glm::mat4 projection;
        glm::mat4 model;
        glm::mat4 view;
        glm::vec3 camPos;
    } uboMatrices;

    struct UBOParams {
        glm::vec4 lights[4];
        float exposure = 3.5f;
        float gamma = 1.0f;
    } uboParams;

    struct {
        VkPipeline skybox;
        VkPipeline pbr;
    } pipelines;

    struct {
        VkDescriptorSet matrices;
        VkDescriptorSet skybox;
    } descriptorSets;

    VkPipelineLayout pipelineLayout;
    VkDescriptorSetLayout descriptorSetLayout;

    // Default materials to select from
    std::vector<OldMaterial> materials;
    int32_t materialIndex = 0;

    std::vector<std::string> materialNames;
    std::vector<std::string> objectNames;

    VulkanExample() : VulkanExampleBase(ENABLE_VALIDATION)
    {
        title = "PBR with image based lighting";

        camera.type = Camera::CameraType::firstperson;
        camera.movementSpeed = 1.0f;
        camera.setPerspective(60.0f, (float)width / (float)height, 0.1f, 256.0f);
        camera.rotationSpeed = 0.05f;

        camera.setRotation({ -35.f, 180.0f, 0.9f });
        camera.setPosition({ 0.f, 0.3f, 0.64f });

        // Setup some default materials (source: https://seblagarde.wordpress.com/2011/08/17/feeding-a-physical-based-lighting-mode/)
        materials.push_back(OldMaterial("Gold", glm::vec3(1.0f, 0.765557f, 0.336057f)));
        materials.push_back(OldMaterial("Copper", glm::vec3(0.955008f, 0.637427f, 0.538163f)));
        materials.push_back(OldMaterial("Chromium", glm::vec3(0.549585f, 0.556114f, 0.554256f)));
        materials.push_back(OldMaterial("Nickel", glm::vec3(0.659777f, 0.608679f, 0.525649f)));
        materials.push_back(OldMaterial("Titanium", glm::vec3(0.541931f, 0.496791f, 0.449419f)));
        materials.push_back(OldMaterial("Cobalt", glm::vec3(0.662124f, 0.654864f, 0.633732f)));
        materials.push_back(OldMaterial("Platinum", glm::vec3(0.672411f, 0.637331f, 0.585456f)));
        // Testing materials
        materials.push_back(OldMaterial("White", glm::vec3(1.0f)));
        materials.push_back(OldMaterial("Dark", glm::vec3(0.1f)));
        materials.push_back(OldMaterial("Black", glm::vec3(0.0f)));
        materials.push_back(OldMaterial("Red", glm::vec3(1.0f, 0.0f, 0.0f)));
        materials.push_back(OldMaterial("Blue", glm::vec3(0.0f, 0.0f, 1.0f)));

        settings.overlay = true;

        for (auto material : materials) {
            materialNames.push_back(material.name);
        }
        objectNames = { "Sphere", "Teapot", "Torusknot", "Venus" };

        materialIndex = 9;

        gameTypes[0].tableWidth = 1.37;
        gameTypes[0].tableLength = 2.42;
        gameTypes[0].ballSize = 0.027;

        gameTypes[1].ballSize = 0.0525;
    }

    ~VulkanExample()
    {
        vkDestroyPipeline(device, pipelines.skybox, nullptr);
        vkDestroyPipeline(device, pipelines.pbr, nullptr);

        vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
        vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

        delete dynamicsWorld;
        delete solver;
        delete dispatcher;
        delete collisionConfiguration;
        delete broadphase;

        delete plateShape;
        delete plateRB;

        delete borderShape;
        delete borderState;
        delete borderRB;

        delete ballShape;
        for (int i=0; i < 15; i++) {
            delete worldObjs[i].motionState;
            delete worldObjs[i].body;
        }

        //models->instanceBuff.unmap();
        modGrp->destroy();
        delete modGrp;

        skybox.destroy();

        uniformBuffers.matrices.destroy();
        uniformBuffers.skybox.destroy();
        uniformBuffers.params.destroy();

        textures.environmentCube.destroy();
        textures.irradianceCube.destroy();
        textures.prefilteredCube.destroy();
        textures.lutBrdf.destroy();
    }

    virtual void getEnabledFeatures()
    {
        if (deviceFeatures.samplerAnisotropy) {
            enabledFeatures.samplerAnisotropy = VK_TRUE;
        }
    }

    void buildCommandBuffers()
    {
        VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

        VkClearValue clearValues[2];
        clearValues[0].color = { { 0.1f, 0.1f, 0.1f, 1.0f } };
        clearValues[1].depthStencil = { 1.0f, 0 };

        VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
        renderPassBeginInfo.renderPass = renderPass;
        renderPassBeginInfo.renderArea.offset.x = 0;
        renderPassBeginInfo.renderArea.offset.y = 0;
        renderPassBeginInfo.renderArea.extent.width = width;
        renderPassBeginInfo.renderArea.extent.height = height;
        renderPassBeginInfo.clearValueCount = 2;
        renderPassBeginInfo.pClearValues = clearValues;

        for (size_t i = 0; i < drawCmdBuffers.size(); ++i)
        {
            // Set target frame buffer
            renderPassBeginInfo.framebuffer = frameBuffers[i];

            VK_CHECK_RESULT(vkBeginCommandBuffer(drawCmdBuffers[i], &cmdBufInfo));

            vkCmdBeginRenderPass(drawCmdBuffers[i], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

            VkViewport viewport = vks::initializers::viewport((float)width,	(float)height, 0.0f, 1.0f);
            vkCmdSetViewport(drawCmdBuffers[i], 0, 1, &viewport);

            VkRect2D scissor = vks::initializers::rect2D(width,	height,	0, 0);
            vkCmdSetScissor(drawCmdBuffers[i], 0, 1, &scissor);

            VkDeviceSize offsets[1] = { 0 };

            // Skybox
            if (displaySkybox)
            {
                vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.skybox, 0, NULL);
                vkCmdBindVertexBuffers(drawCmdBuffers[i], 0, 1, &skybox.vertices.buffer, offsets);
                vkCmdBindIndexBuffer(drawCmdBuffers[i], skybox.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
                vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.skybox);
                vkCmdDrawIndexed(drawCmdBuffers[i], skybox.indexCount, 1, 0, 0, 0);
            }

            // Objects
            vkCmdBindDescriptorSets(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets.matrices, 0, NULL);
            vkCmdBindVertexBuffers(drawCmdBuffers[i], 0, 1, &modGrp->vertices.buffer, offsets);
            vkCmdBindIndexBuffer(drawCmdBuffers[i], modGrp->indices.buffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdBindVertexBuffers(drawCmdBuffers[i], 1, 1, &modGrp->instanceBuff.buffer, offsets);
            vkCmdBindPipeline(drawCmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelines.pbr);

            OldMaterial mat = materials[materialIndex];
            mat.params.roughness = 0.1f;// 1.0f-glm::clamp((float)x / (float)objcount, 0.005f, 1.0f);
            mat.params.metallic = 0.2f;// glm::clamp((float)x / (float)objcount, 0.005f, 1.0f);
            mat.params.specular = 0.9f;

            //vkCmdPushConstants(drawCmdBuffers[i], pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::vec3), &pos);
            vkCmdPushConstants(drawCmdBuffers[i], pipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, sizeof(glm::vec3), sizeof(OldMaterial::PushBlock), &mat);

            modGrp->buildCommandBuffer(drawCmdBuffers[i]);

            vkCmdEndRenderPass(drawCmdBuffers[i]);

            VK_CHECK_RESULT(vkEndCommandBuffer(drawCmdBuffers[i]));
        }
    }

    uint32_t texSize = 1024;

    vks::ModelGroup* modGrp;
    vks::ModelGroup* modBodies;

    int modBallIdx = -1, modPlateIdx = -1, modFlipIdx = -1, modStaticsIdx = -1, modTargetsIdx = -1;
    int worldObjBall = 0;
    int worldObjFlip = 1;

    struct Target {
        vks::ModelGroup::InstanceData* pInstance;
        bool state; //true when reached
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

    TargetGroup leftTargetGroup;


    void loadAssets()
    {
        modGrp = new vks::ModelGroup(vulkanDevice, queue);

        modBallIdx = modGrp->addModel(getAssetPath() + "models/geosphere1.obj", gameTypes[0].ballSize);
        modFlipIdx = modGrp->addModel(getAssetPath() + "models/pinball-flip-hr.obj");
        modTargetsIdx = modGrp->addModel(getAssetPath() + "models/pinball-target-hr.obj");
        modPlateIdx = modGrp->addModel(getAssetPath() + "models/pinball.obj");
        modStaticsIdx = modGrp->addModel(getAssetPath() + "models/pinball-static-hr.obj");

        // Skybox
        skybox.loadFromFile(getAssetPath() + "models/cube.obj", vertexLayout, 1.0f, vulkanDevice, queue);

        textures.environmentCube.loadFromFile(getAssetPath() + "textures/hdr/pisa_cube.ktx", VK_FORMAT_R16G16B16A16_SFLOAT, vulkanDevice, queue);

        std::vector<std::string> mapDic = {
            getAssetPath() + "pinball-back.png",
            getAssetPath() + "flipper.png",
        };

        modGrp->addMaterial (0.9f,0.9f,0.9f,1, 0.9f,0.16f,0.f);
        modGrp->addMaterial (0, 0.001f, 0.9f, 0);
        modGrp->addMaterial (1, 0.3f, 0.4f,0);
        modGrp->addMaterial (0.749585f, 0.756114f, 0.754256f, 1.0f, 0.7f, 0.02f, 0);
        //modGrp->addMaterial(1.0f,0.0f,1.0f,1.0f, 1.f, 1.f, 0);

        modGrp->addInstance(modBallIdx, 0);

        modGrp->addInstance(modFlipIdx, 2);

        int modTargetFirstInstanceIdx = modGrp->addInstance(modTargetsIdx, 2);

        modGrp->addInstance(modPlateIdx, 1);
        int inst = modGrp->addInstance(modStaticsIdx, 1);

        modGrp->instanceDatas[inst].materialIndex = 3;
        modGrp->instanceDatas[inst+1].materialIndex = 3;
        modGrp->instanceDatas[inst+6].materialIndex = 3;

        //modGrp->instanceDatas[inst+9].materialIndex = 3;
        //modGrp->instanceDatas[0].materialIndex = 0;
        int bump = 2 + inst;
        modGrp->instanceDatas[bump].materialIndex = 2;
        modGrp->instanceDatas[bump+1].materialIndex = 2;
        modGrp->instanceDatas[bump+2].materialIndex = 2;
        int damp = 7 + inst;
        modGrp->instanceDatas[damp].materialIndex = 2;
        modGrp->instanceDatas[damp+1].materialIndex = 2;

        modGrp->prepare(mapDic, texSize);

        for (int i = 0; i < 4; i++)
            leftTargetGroup.addTarget({&modGrp->instanceDatas[modTargetFirstInstanceIdx+i], false, nullptr});


    }

    void setupDescriptors()
    {
        // Descriptor Pool
        std::vector<VkDescriptorPoolSize> poolSizes = {
            vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 6),
            vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 8)
        };
        VkDescriptorPoolCreateInfo descriptorPoolInfo =	vks::initializers::descriptorPoolCreateInfo(poolSizes, 2);
        VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr, &descriptorPool));

        // Descriptor set layout
        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0),
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 1),
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 2),
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 3),
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 4),
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 5),
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_FRAGMENT_BIT, 6),
        };
        VkDescriptorSetLayoutCreateInfo descriptorLayout = 	vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings);
        VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorLayout, nullptr, &descriptorSetLayout));

        // Descriptor sets
        VkDescriptorSetAllocateInfo allocInfo = vks::initializers::descriptorSetAllocateInfo(descriptorPool, &descriptorSetLayout, 1);

        // Objects
        VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.matrices));
        std::vector<VkWriteDescriptorSet> writeDescriptorSets = {
            vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.matrices.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, &uniformBuffers.params.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2, &textures.irradianceCube.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3, &textures.lutBrdf.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4, &textures.prefilteredCube.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 5, &modGrp->texArray.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 6, &modGrp->materialsBuff.descriptor),
        };
        vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, NULL);

        // Sky box
        VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorSets.skybox));
        writeDescriptorSets = {
            vks::initializers::writeDescriptorSet(descriptorSets.skybox, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &uniformBuffers.skybox.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.skybox, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, &uniformBuffers.params.descriptor),
            vks::initializers::writeDescriptorSet(descriptorSets.skybox, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2, &textures.environmentCube.descriptor),
        };
        vkUpdateDescriptorSets(device, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, NULL);
    }

    void preparePipelines()
    {
        VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
            vks::initializers::pipelineInputAssemblyStateCreateInfo(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, VK_FALSE);

        VkPipelineRasterizationStateCreateInfo rasterizationState =
            vks::initializers::pipelineRasterizationStateCreateInfo(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE);

        VkPipelineColorBlendAttachmentState blendAttachmentState =
            vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE);

        VkPipelineColorBlendStateCreateInfo colorBlendState =
            vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);

        VkPipelineDepthStencilStateCreateInfo depthStencilState =
            vks::initializers::pipelineDepthStencilStateCreateInfo(VK_FALSE, VK_FALSE, VK_COMPARE_OP_LESS_OR_EQUAL);

        VkPipelineViewportStateCreateInfo viewportState =
            vks::initializers::pipelineViewportStateCreateInfo(1, 1);

        VkPipelineMultisampleStateCreateInfo multisampleState =
            vks::initializers::pipelineMultisampleStateCreateInfo(VK_SAMPLE_COUNT_1_BIT);

        std::vector<VkDynamicState> dynamicStateEnables = {
            VK_DYNAMIC_STATE_VIEWPORT,
            VK_DYNAMIC_STATE_SCISSOR
        };
        VkPipelineDynamicStateCreateInfo dynamicState =
            vks::initializers::pipelineDynamicStateCreateInfo(dynamicStateEnables);

        // Pipeline layout
        VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = vks::initializers::pipelineLayoutCreateInfo(&descriptorSetLayout, 1);
        // Push constant ranges
        std::vector<VkPushConstantRange> pushConstantRanges = {
            vks::initializers::pushConstantRange(VK_SHADER_STAGE_VERTEX_BIT, sizeof(glm::vec3), 0),
            vks::initializers::pushConstantRange(VK_SHADER_STAGE_FRAGMENT_BIT, sizeof(OldMaterial::PushBlock), sizeof(glm::vec3)),
        };
        pipelineLayoutCreateInfo.pushConstantRangeCount = 2;
        pipelineLayoutCreateInfo.pPushConstantRanges = pushConstantRanges.data();
        VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCreateInfo, nullptr, &pipelineLayout));

        // Pipelines
        VkGraphicsPipelineCreateInfo pipelineCreateInfo = vks::initializers::pipelineCreateInfo(pipelineLayout, renderPass);

        std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

        pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
        pipelineCreateInfo.pRasterizationState = &rasterizationState;
        pipelineCreateInfo.pColorBlendState = &colorBlendState;
        pipelineCreateInfo.pMultisampleState = &multisampleState;
        pipelineCreateInfo.pViewportState = &viewportState;
        pipelineCreateInfo.pDepthStencilState = &depthStencilState;
        pipelineCreateInfo.pDynamicState = &dynamicState;
        pipelineCreateInfo.stageCount = static_cast<uint32_t>(shaderStages.size());
        pipelineCreateInfo.pStages = shaderStages.data();

        // Vertex bindings an attributes
        // Binding description
        std::vector<VkVertexInputBindingDescription> vertexInputBindings = {
            vks::initializers::vertexInputBindingDescription(0, vertexLayout.stride(), VK_VERTEX_INPUT_RATE_VERTEX),
            vks::initializers::vertexInputBindingDescription(1, sizeof(vks::ModelGroup::InstanceData), VK_VERTEX_INPUT_RATE_INSTANCE)
        };

        // Attribute descriptions
        std::vector<VkVertexInputAttributeDescription> vertexInputAttributes = {
            vks::initializers::vertexInputAttributeDescription(0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0),					// Position
            vks::initializers::vertexInputAttributeDescription(0, 1, VK_FORMAT_R32G32B32_SFLOAT, sizeof(float) * 3),	// Normal
            vks::initializers::vertexInputAttributeDescription(0, 2, VK_FORMAT_R32G32_SFLOAT, sizeof(float) * 6),		// UV

            vks::initializers::vertexInputAttributeDescription(1, 3, VK_FORMAT_R32_UINT, 0),							// Material Index
            vks::initializers::vertexInputAttributeDescription(1, 4, VK_FORMAT_R32G32B32A32_SFLOAT, 4),					// Model matrix
            vks::initializers::vertexInputAttributeDescription(1, 5, VK_FORMAT_R32G32B32A32_SFLOAT, 20),
            vks::initializers::vertexInputAttributeDescription(1, 6, VK_FORMAT_R32G32B32A32_SFLOAT, 36),
            vks::initializers::vertexInputAttributeDescription(1, 7, VK_FORMAT_R32G32B32A32_SFLOAT, 52),
        };

        VkPipelineVertexInputStateCreateInfo vertexInputState = vks::initializers::pipelineVertexInputStateCreateInfo();
        vertexInputState.vertexBindingDescriptionCount = 1;
        vertexInputState.pVertexBindingDescriptions = vertexInputBindings.data();
        vertexInputState.vertexAttributeDescriptionCount = 3;
        vertexInputState.pVertexAttributeDescriptions = vertexInputAttributes.data();

        pipelineCreateInfo.pVertexInputState = &vertexInputState;

        // Skybox pipeline (background cube)
        shaderStages[0] = loadShader(getAssetPath() +  "shaders/pinball/skybox.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
        shaderStages[1] = loadShader(getAssetPath() +  "shaders/pinball/skybox.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
        VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.skybox));

        vertexInputState.vertexBindingDescriptionCount = static_cast<uint32_t>(vertexInputBindings.size());
        vertexInputState.vertexAttributeDescriptionCount = static_cast<uint32_t>(vertexInputAttributes.size());

        // PBR pipeline
        shaderStages[0] = loadShader(getAssetPath() +  "shaders/pinball/pbribl.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
        shaderStages[1] = loadShader(getAssetPath() +  "shaders/pinball/pbribl.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
        // Enable depth test and write
        depthStencilState.depthWriteEnable = VK_TRUE;
        depthStencilState.depthTestEnable = VK_TRUE;
        VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipelines.pbr));
    }

    // Generate a BRDF integration map used as a look-up-table (stores roughness / NdotV)
    void generateBRDFLUT()
    {
        auto tStart = std::chrono::high_resolution_clock::now();

        const VkFormat format = VK_FORMAT_R16G16_SFLOAT;	// R16G16 is supported pretty much everywhere
        const int32_t dim = 512;

        // Image
        VkImageCreateInfo imageCI = vks::initializers::imageCreateInfo();
        imageCI.imageType = VK_IMAGE_TYPE_2D;
        imageCI.format = format;
        imageCI.extent.width = dim;
        imageCI.extent.height = dim;
        imageCI.extent.depth = 1;
        imageCI.mipLevels = 1;
        imageCI.arrayLayers = 1;
        imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
        imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageCI.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &textures.lutBrdf.image));
        VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
        VkMemoryRequirements memReqs;
        vkGetImageMemoryRequirements(device, textures.lutBrdf.image, &memReqs);
        memAlloc.allocationSize = memReqs.size;
        memAlloc.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &textures.lutBrdf.deviceMemory));
        VK_CHECK_RESULT(vkBindImageMemory(device, textures.lutBrdf.image, textures.lutBrdf.deviceMemory, 0));
        // Image view
        VkImageViewCreateInfo viewCI = vks::initializers::imageViewCreateInfo();
        viewCI.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewCI.format = format;
        viewCI.subresourceRange = {};
        viewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewCI.subresourceRange.levelCount = 1;
        viewCI.subresourceRange.layerCount = 1;
        viewCI.image = textures.lutBrdf.image;
        VK_CHECK_RESULT(vkCreateImageView(device, &viewCI, nullptr, &textures.lutBrdf.view));
        // Sampler
        VkSamplerCreateInfo samplerCI = vks::initializers::samplerCreateInfo();
        samplerCI.magFilter = VK_FILTER_LINEAR;
        samplerCI.minFilter = VK_FILTER_LINEAR;
        samplerCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.minLod = 0.0f;
        samplerCI.maxLod = 1.0f;
        samplerCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        VK_CHECK_RESULT(vkCreateSampler(device, &samplerCI, nullptr, &textures.lutBrdf.sampler));

        textures.lutBrdf.descriptor.imageView = textures.lutBrdf.view;
        textures.lutBrdf.descriptor.sampler = textures.lutBrdf.sampler;
        textures.lutBrdf.descriptor.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        textures.lutBrdf.device = vulkanDevice;

        // FB, Att, RP, Pipe, etc.
        VkAttachmentDescription attDesc = {};
        // Color attachment
        attDesc.format = format;
        attDesc.samples = VK_SAMPLE_COUNT_1_BIT;
        attDesc.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attDesc.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attDesc.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attDesc.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attDesc.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attDesc.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference colorReference = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };

        VkSubpassDescription subpassDescription = {};
        subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpassDescription.colorAttachmentCount = 1;
        subpassDescription.pColorAttachments = &colorReference;

        // Use subpass dependencies for layout transitions
        std::array<VkSubpassDependency, 2> dependencies;
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

        // Create the actual renderpass
        VkRenderPassCreateInfo renderPassCI = vks::initializers::renderPassCreateInfo();
        renderPassCI.attachmentCount = 1;
        renderPassCI.pAttachments = &attDesc;
        renderPassCI.subpassCount = 1;
        renderPassCI.pSubpasses = &subpassDescription;
        renderPassCI.dependencyCount = 2;
        renderPassCI.pDependencies = dependencies.data();

        VkRenderPass renderpass;
        VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassCI, nullptr, &renderpass));

        VkFramebufferCreateInfo framebufferCI = vks::initializers::framebufferCreateInfo();
        framebufferCI.renderPass = renderpass;
        framebufferCI.attachmentCount = 1;
        framebufferCI.pAttachments = &textures.lutBrdf.view;
        framebufferCI.width = dim;
        framebufferCI.height = dim;
        framebufferCI.layers = 1;

        VkFramebuffer framebuffer;
        VK_CHECK_RESULT(vkCreateFramebuffer(device, &framebufferCI, nullptr, &framebuffer));

        // Desriptors
        VkDescriptorSetLayout descriptorsetlayout;
        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {};
        VkDescriptorSetLayoutCreateInfo descriptorsetlayoutCI = vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings);
        VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorsetlayoutCI, nullptr, &descriptorsetlayout));

        // Descriptor Pool
        std::vector<VkDescriptorPoolSize> poolSizes = { vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1) };
        VkDescriptorPoolCreateInfo descriptorPoolCI = vks::initializers::descriptorPoolCreateInfo(poolSizes, 2);
        VkDescriptorPool descriptorpool;
        VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCI, nullptr, &descriptorpool));

        // Descriptor sets
        VkDescriptorSet descriptorset;
        VkDescriptorSetAllocateInfo allocInfo = vks::initializers::descriptorSetAllocateInfo(descriptorpool, &descriptorsetlayout, 1);
        VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorset));

        // Pipeline layout
        VkPipelineLayout pipelinelayout;
        VkPipelineLayoutCreateInfo pipelineLayoutCI = vks::initializers::pipelineLayoutCreateInfo(&descriptorsetlayout, 1);
        VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCI, nullptr, &pipelinelayout));

        // Pipeline
        VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = vks::initializers::pipelineInputAssemblyStateCreateInfo(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, VK_FALSE);
        VkPipelineRasterizationStateCreateInfo rasterizationState = vks::initializers::pipelineRasterizationStateCreateInfo(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE);
        VkPipelineColorBlendAttachmentState blendAttachmentState = vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE);
        VkPipelineColorBlendStateCreateInfo colorBlendState = vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);
        VkPipelineDepthStencilStateCreateInfo depthStencilState = vks::initializers::pipelineDepthStencilStateCreateInfo(VK_FALSE, VK_FALSE, VK_COMPARE_OP_LESS_OR_EQUAL);
        VkPipelineViewportStateCreateInfo viewportState = vks::initializers::pipelineViewportStateCreateInfo(1, 1);
        VkPipelineMultisampleStateCreateInfo multisampleState = vks::initializers::pipelineMultisampleStateCreateInfo(VK_SAMPLE_COUNT_1_BIT);
        std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamicState = vks::initializers::pipelineDynamicStateCreateInfo(dynamicStateEnables);
        VkPipelineVertexInputStateCreateInfo emptyInputState = vks::initializers::pipelineVertexInputStateCreateInfo();
        std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

        VkGraphicsPipelineCreateInfo pipelineCI = vks::initializers::pipelineCreateInfo(pipelinelayout, renderpass);
        pipelineCI.pInputAssemblyState = &inputAssemblyState;
        pipelineCI.pRasterizationState = &rasterizationState;
        pipelineCI.pColorBlendState = &colorBlendState;
        pipelineCI.pMultisampleState = &multisampleState;
        pipelineCI.pViewportState = &viewportState;
        pipelineCI.pDepthStencilState = &depthStencilState;
        pipelineCI.pDynamicState = &dynamicState;
        pipelineCI.stageCount = 2;
        pipelineCI.pStages = shaderStages.data();
        pipelineCI.pVertexInputState = &emptyInputState;

        // Look-up-table (from BRDF) pipeline
        shaderStages[0] = loadShader(getAssetPath() +  "shaders/pbribl/genbrdflut.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
        shaderStages[1] = loadShader(getAssetPath() +  "shaders/pbribl/genbrdflut.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
        VkPipeline pipeline;
        VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipeline));

        // Render
        VkClearValue clearValues[1];
        clearValues[0].color = { { 0.0f, 0.0f, 0.0f, 1.0f } };

        VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
        renderPassBeginInfo.renderPass = renderpass;
        renderPassBeginInfo.renderArea.extent.width = dim;
        renderPassBeginInfo.renderArea.extent.height = dim;
        renderPassBeginInfo.clearValueCount = 1;
        renderPassBeginInfo.pClearValues = clearValues;
        renderPassBeginInfo.framebuffer = framebuffer;

        VkCommandBuffer cmdBuf = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
        vkCmdBeginRenderPass(cmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);
        VkViewport viewport = vks::initializers::viewport((float)dim, (float)dim, 0.0f, 1.0f);
        VkRect2D scissor = vks::initializers::rect2D(dim, dim, 0, 0);
        vkCmdSetViewport(cmdBuf, 0, 1, &viewport);
        vkCmdSetScissor(cmdBuf, 0, 1, &scissor);
        vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        vkCmdDraw(cmdBuf, 3, 1, 0, 0);
        vkCmdEndRenderPass(cmdBuf);
        vulkanDevice->flushCommandBuffer(cmdBuf, queue);

        vkQueueWaitIdle(queue);

        // todo: cleanup
        vkDestroyPipeline(device, pipeline, nullptr);
        vkDestroyPipelineLayout(device, pipelinelayout, nullptr);
        vkDestroyRenderPass(device, renderpass, nullptr);
        vkDestroyFramebuffer(device, framebuffer, nullptr);
        vkDestroyDescriptorSetLayout(device, descriptorsetlayout, nullptr);
        vkDestroyDescriptorPool(device, descriptorpool, nullptr);

        auto tEnd = std::chrono::high_resolution_clock::now();
        auto tDiff = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
        std::cout << "Generating BRDF LUT took " << tDiff << " ms" << std::endl;
    }

    // Generate an irradiance cube map from the environment cube map
    void generateIrradianceCube()
    {
        auto tStart = std::chrono::high_resolution_clock::now();

        const VkFormat format = VK_FORMAT_R32G32B32A32_SFLOAT;
        const int32_t dim = 64;
        const uint32_t numMips = static_cast<uint32_t>(floor(log2(dim))) + 1;

        // Pre-filtered cube map
        // Image
        VkImageCreateInfo imageCI = vks::initializers::imageCreateInfo();
        imageCI.imageType = VK_IMAGE_TYPE_2D;
        imageCI.format = format;
        imageCI.extent.width = dim;
        imageCI.extent.height = dim;
        imageCI.extent.depth = 1;
        imageCI.mipLevels = numMips;
        imageCI.arrayLayers = 6;
        imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
        imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageCI.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        imageCI.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
        VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &textures.irradianceCube.image));
        VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
        VkMemoryRequirements memReqs;
        vkGetImageMemoryRequirements(device, textures.irradianceCube.image, &memReqs);
        memAlloc.allocationSize = memReqs.size;
        memAlloc.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &textures.irradianceCube.deviceMemory));
        VK_CHECK_RESULT(vkBindImageMemory(device, textures.irradianceCube.image, textures.irradianceCube.deviceMemory, 0));
        // Image view
        VkImageViewCreateInfo viewCI = vks::initializers::imageViewCreateInfo();
        viewCI.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
        viewCI.format = format;
        viewCI.subresourceRange = {};
        viewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewCI.subresourceRange.levelCount = numMips;
        viewCI.subresourceRange.layerCount = 6;
        viewCI.image = textures.irradianceCube.image;
        VK_CHECK_RESULT(vkCreateImageView(device, &viewCI, nullptr, &textures.irradianceCube.view));
        // Sampler
        VkSamplerCreateInfo samplerCI = vks::initializers::samplerCreateInfo();
        samplerCI.magFilter = VK_FILTER_LINEAR;
        samplerCI.minFilter = VK_FILTER_LINEAR;
        samplerCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.minLod = 0.0f;
        samplerCI.maxLod = static_cast<float>(numMips);
        samplerCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        VK_CHECK_RESULT(vkCreateSampler(device, &samplerCI, nullptr, &textures.irradianceCube.sampler));

        textures.irradianceCube.descriptor.imageView = textures.irradianceCube.view;
        textures.irradianceCube.descriptor.sampler = textures.irradianceCube.sampler;
        textures.irradianceCube.descriptor.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        textures.irradianceCube.device = vulkanDevice;

        // FB, Att, RP, Pipe, etc.
        VkAttachmentDescription attDesc = {};
        // Color attachment
        attDesc.format = format;
        attDesc.samples = VK_SAMPLE_COUNT_1_BIT;
        attDesc.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attDesc.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attDesc.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attDesc.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attDesc.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attDesc.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        VkAttachmentReference colorReference = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };

        VkSubpassDescription subpassDescription = {};
        subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpassDescription.colorAttachmentCount = 1;
        subpassDescription.pColorAttachments = &colorReference;

        // Use subpass dependencies for layout transitions
        std::array<VkSubpassDependency, 2> dependencies;
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

        // Renderpass
        VkRenderPassCreateInfo renderPassCI = vks::initializers::renderPassCreateInfo();
        renderPassCI.attachmentCount = 1;
        renderPassCI.pAttachments = &attDesc;
        renderPassCI.subpassCount = 1;
        renderPassCI.pSubpasses = &subpassDescription;
        renderPassCI.dependencyCount = 2;
        renderPassCI.pDependencies = dependencies.data();
        VkRenderPass renderpass;
        VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassCI, nullptr, &renderpass));

        struct {
            VkImage image;
            VkImageView view;
            VkDeviceMemory memory;
            VkFramebuffer framebuffer;
        } offscreen;

        // Offfscreen framebuffer
        {
            // Color attachment
            VkImageCreateInfo imageCreateInfo = vks::initializers::imageCreateInfo();
            imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
            imageCreateInfo.format = format;
            imageCreateInfo.extent.width = dim;
            imageCreateInfo.extent.height = dim;
            imageCreateInfo.extent.depth = 1;
            imageCreateInfo.mipLevels = 1;
            imageCreateInfo.arrayLayers = 1;
            imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
            imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageCreateInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
            imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            VK_CHECK_RESULT(vkCreateImage(device, &imageCreateInfo, nullptr, &offscreen.image));

            VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
            VkMemoryRequirements memReqs;
            vkGetImageMemoryRequirements(device, offscreen.image, &memReqs);
            memAlloc.allocationSize = memReqs.size;
            memAlloc.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &offscreen.memory));
            VK_CHECK_RESULT(vkBindImageMemory(device, offscreen.image, offscreen.memory, 0));

            VkImageViewCreateInfo colorImageView = vks::initializers::imageViewCreateInfo();
            colorImageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
            colorImageView.format = format;
            colorImageView.flags = 0;
            colorImageView.subresourceRange = {};
            colorImageView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            colorImageView.subresourceRange.baseMipLevel = 0;
            colorImageView.subresourceRange.levelCount = 1;
            colorImageView.subresourceRange.baseArrayLayer = 0;
            colorImageView.subresourceRange.layerCount = 1;
            colorImageView.image = offscreen.image;
            VK_CHECK_RESULT(vkCreateImageView(device, &colorImageView, nullptr, &offscreen.view));

            VkFramebufferCreateInfo fbufCreateInfo = vks::initializers::framebufferCreateInfo();
            fbufCreateInfo.renderPass = renderpass;
            fbufCreateInfo.attachmentCount = 1;
            fbufCreateInfo.pAttachments = &offscreen.view;
            fbufCreateInfo.width = dim;
            fbufCreateInfo.height = dim;
            fbufCreateInfo.layers = 1;
            VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &offscreen.framebuffer));

            VkCommandBuffer layoutCmd = VulkanExampleBase::createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
            vks::tools::setImageLayout(
                layoutCmd,
                offscreen.image,
                VK_IMAGE_ASPECT_COLOR_BIT,
                VK_IMAGE_LAYOUT_UNDEFINED,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            VulkanExampleBase::flushCommandBuffer(layoutCmd, queue, true);
        }

        // Descriptors
        VkDescriptorSetLayout descriptorsetlayout;
        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0),
        };
        VkDescriptorSetLayoutCreateInfo descriptorsetlayoutCI = vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings);
        VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorsetlayoutCI, nullptr, &descriptorsetlayout));

        // Descriptor Pool
        std::vector<VkDescriptorPoolSize> poolSizes = { vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1) };
        VkDescriptorPoolCreateInfo descriptorPoolCI = vks::initializers::descriptorPoolCreateInfo(poolSizes, 2);
        VkDescriptorPool descriptorpool;
        VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCI, nullptr, &descriptorpool));

        // Descriptor sets
        VkDescriptorSet descriptorset;
        VkDescriptorSetAllocateInfo allocInfo = vks::initializers::descriptorSetAllocateInfo(descriptorpool, &descriptorsetlayout, 1);
        VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorset));
        VkWriteDescriptorSet writeDescriptorSet = vks::initializers::writeDescriptorSet(descriptorset, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 0, &textures.environmentCube.descriptor);
        vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);

        // Pipeline layout
        struct PushBlock {
            glm::mat4 mvp;
            // Sampling deltas
            float deltaPhi = (2.0f * float(M_PI)) / 180.0f;
            float deltaTheta = (0.5f * float(M_PI)) / 64.0f;
        } pushBlock;

        VkPipelineLayout pipelinelayout;
        std::vector<VkPushConstantRange> pushConstantRanges = {
            vks::initializers::pushConstantRange(VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, sizeof(PushBlock), 0),
        };
        VkPipelineLayoutCreateInfo pipelineLayoutCI = vks::initializers::pipelineLayoutCreateInfo(&descriptorsetlayout, 1);
        pipelineLayoutCI.pushConstantRangeCount = 1;
        pipelineLayoutCI.pPushConstantRanges = pushConstantRanges.data();
        VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCI, nullptr, &pipelinelayout));

        // Pipeline
        VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = vks::initializers::pipelineInputAssemblyStateCreateInfo(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, VK_FALSE);
        VkPipelineRasterizationStateCreateInfo rasterizationState = vks::initializers::pipelineRasterizationStateCreateInfo(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE);
        VkPipelineColorBlendAttachmentState blendAttachmentState = vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE);
        VkPipelineColorBlendStateCreateInfo colorBlendState = vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);
        VkPipelineDepthStencilStateCreateInfo depthStencilState = vks::initializers::pipelineDepthStencilStateCreateInfo(VK_FALSE, VK_FALSE, VK_COMPARE_OP_LESS_OR_EQUAL);
        VkPipelineViewportStateCreateInfo viewportState = vks::initializers::pipelineViewportStateCreateInfo(1, 1);
        VkPipelineMultisampleStateCreateInfo multisampleState = vks::initializers::pipelineMultisampleStateCreateInfo(VK_SAMPLE_COUNT_1_BIT);
        std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamicState = vks::initializers::pipelineDynamicStateCreateInfo(dynamicStateEnables);
        // Vertex input state
        VkVertexInputBindingDescription vertexInputBinding = vks::initializers::vertexInputBindingDescription(0, vertexLayout.stride(), VK_VERTEX_INPUT_RATE_VERTEX);
        VkVertexInputAttributeDescription vertexInputAttribute = vks::initializers::vertexInputAttributeDescription(0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0);

        VkPipelineVertexInputStateCreateInfo vertexInputState = vks::initializers::pipelineVertexInputStateCreateInfo();
        vertexInputState.vertexBindingDescriptionCount = 1;
        vertexInputState.pVertexBindingDescriptions = &vertexInputBinding;
        vertexInputState.vertexAttributeDescriptionCount = 1;
        vertexInputState.pVertexAttributeDescriptions = &vertexInputAttribute;

        std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

        VkGraphicsPipelineCreateInfo pipelineCI = vks::initializers::pipelineCreateInfo(pipelinelayout, renderpass);
        pipelineCI.pInputAssemblyState = &inputAssemblyState;
        pipelineCI.pRasterizationState = &rasterizationState;
        pipelineCI.pColorBlendState = &colorBlendState;
        pipelineCI.pMultisampleState = &multisampleState;
        pipelineCI.pViewportState = &viewportState;
        pipelineCI.pDepthStencilState = &depthStencilState;
        pipelineCI.pDynamicState = &dynamicState;
        pipelineCI.stageCount = 2;
        pipelineCI.pStages = shaderStages.data();
        pipelineCI.pVertexInputState = &vertexInputState;
        pipelineCI.renderPass = renderpass;

        shaderStages[0] = loadShader(getAssetPath() +  "shaders/pbribl/filtercube.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
        shaderStages[1] = loadShader(getAssetPath() +  "shaders/pbribl/irradiancecube.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
        VkPipeline pipeline;
        VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipeline));

        // Render

        VkClearValue clearValues[1];
        clearValues[0].color = { { 0.0f, 0.0f, 0.2f, 0.0f } };

        VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
        // Reuse render pass from example pass
        renderPassBeginInfo.renderPass = renderpass;
        renderPassBeginInfo.framebuffer = offscreen.framebuffer;
        renderPassBeginInfo.renderArea.extent.width = dim;
        renderPassBeginInfo.renderArea.extent.height = dim;
        renderPassBeginInfo.clearValueCount = 1;
        renderPassBeginInfo.pClearValues = clearValues;

        std::vector<glm::mat4> matrices = {
            // POSITIVE_X
            glm::rotate(glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // NEGATIVE_X
            glm::rotate(glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // POSITIVE_Y
            glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // NEGATIVE_Y
            glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // POSITIVE_Z
            glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // NEGATIVE_Z
            glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
        };

        VkCommandBuffer cmdBuf = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

        VkViewport viewport = vks::initializers::viewport((float)dim, (float)dim, 0.0f, 1.0f);
        VkRect2D scissor = vks::initializers::rect2D(dim, dim, 0, 0);

        vkCmdSetViewport(cmdBuf, 0, 1, &viewport);
        vkCmdSetScissor(cmdBuf, 0, 1, &scissor);

        VkImageSubresourceRange subresourceRange = {};
        subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        subresourceRange.baseMipLevel = 0;
        subresourceRange.levelCount = numMips;
        subresourceRange.layerCount = 6;

        // Change image layout for all cubemap faces to transfer destination
        vks::tools::setImageLayout(
            cmdBuf,
            textures.irradianceCube.image,
            VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            subresourceRange);

        for (uint32_t m = 0; m < numMips; m++) {
            for (uint32_t f = 0; f < 6; f++) {
                viewport.width = static_cast<float>(dim * std::pow(0.5f, m));
                viewport.height = static_cast<float>(dim * std::pow(0.5f, m));
                vkCmdSetViewport(cmdBuf, 0, 1, &viewport);

                // Render scene from cube face's point of view
                vkCmdBeginRenderPass(cmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

                // Update shader push constant block
                pushBlock.mvp = glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f, 512.0f) * matrices[f];

                vkCmdPushConstants(cmdBuf, pipelinelayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushBlock), &pushBlock);

                vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
                vkCmdBindDescriptorSets(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelinelayout, 0, 1, &descriptorset, 0, NULL);

                VkDeviceSize offsets[1] = { 0 };

                vkCmdBindVertexBuffers(cmdBuf, 0, 1, &skybox.vertices.buffer, offsets);
                vkCmdBindIndexBuffer(cmdBuf, skybox.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
                vkCmdDrawIndexed(cmdBuf, skybox.indexCount, 1, 0, 0, 0);

                vkCmdEndRenderPass(cmdBuf);

                vks::tools::setImageLayout(
                    cmdBuf,
                    offscreen.image,
                    VK_IMAGE_ASPECT_COLOR_BIT,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

                // Copy region for transfer from framebuffer to cube face
                VkImageCopy copyRegion = {};

                copyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copyRegion.srcSubresource.baseArrayLayer = 0;
                copyRegion.srcSubresource.mipLevel = 0;
                copyRegion.srcSubresource.layerCount = 1;
                copyRegion.srcOffset = { 0, 0, 0 };

                copyRegion.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copyRegion.dstSubresource.baseArrayLayer = f;
                copyRegion.dstSubresource.mipLevel = m;
                copyRegion.dstSubresource.layerCount = 1;
                copyRegion.dstOffset = { 0, 0, 0 };

                copyRegion.extent.width = static_cast<uint32_t>(viewport.width);
                copyRegion.extent.height = static_cast<uint32_t>(viewport.height);
                copyRegion.extent.depth = 1;

                vkCmdCopyImage(
                    cmdBuf,
                    offscreen.image,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    textures.irradianceCube.image,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    1,
                    &copyRegion);

                // Transform framebuffer color attachment back
                vks::tools::setImageLayout(
                    cmdBuf,
                    offscreen.image,
                    VK_IMAGE_ASPECT_COLOR_BIT,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            }
        }

        vks::tools::setImageLayout(
            cmdBuf,
            textures.irradianceCube.image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            subresourceRange);

        vulkanDevice->flushCommandBuffer(cmdBuf, queue);

        // todo: cleanup
        vkDestroyRenderPass(device, renderpass, nullptr);
        vkDestroyFramebuffer(device, offscreen.framebuffer, nullptr);
        vkFreeMemory(device, offscreen.memory, nullptr);
        vkDestroyImageView(device, offscreen.view, nullptr);
        vkDestroyImage(device, offscreen.image, nullptr);
        vkDestroyDescriptorPool(device, descriptorpool, nullptr);
        vkDestroyDescriptorSetLayout(device, descriptorsetlayout, nullptr);
        vkDestroyPipeline(device, pipeline, nullptr);
        vkDestroyPipelineLayout(device, pipelinelayout, nullptr);

        auto tEnd = std::chrono::high_resolution_clock::now();
        auto tDiff = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
        std::cout << "Generating irradiance cube with " << numMips << " mip levels took " << tDiff << " ms" << std::endl;
    }

    // Prefilter environment cubemap
    // See https://placeholderart.wordpress.com/2015/07/28/implementation-notes-runtime-environment-map-filtering-for-image-based-lighting/
    void generatePrefilteredCube()
    {
        auto tStart = std::chrono::high_resolution_clock::now();

        const VkFormat format = VK_FORMAT_R16G16B16A16_SFLOAT;
        const int32_t dim = 512;
        const uint32_t numMips = static_cast<uint32_t>(floor(log2(dim))) + 1;

        // Pre-filtered cube map
        // Image
        VkImageCreateInfo imageCI = vks::initializers::imageCreateInfo();
        imageCI.imageType = VK_IMAGE_TYPE_2D;
        imageCI.format = format;
        imageCI.extent.width = dim;
        imageCI.extent.height = dim;
        imageCI.extent.depth = 1;
        imageCI.mipLevels = numMips;
        imageCI.arrayLayers = 6;
        imageCI.samples = VK_SAMPLE_COUNT_1_BIT;
        imageCI.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageCI.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        imageCI.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
        VK_CHECK_RESULT(vkCreateImage(device, &imageCI, nullptr, &textures.prefilteredCube.image));
        VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
        VkMemoryRequirements memReqs;
        vkGetImageMemoryRequirements(device, textures.prefilteredCube.image, &memReqs);
        memAlloc.allocationSize = memReqs.size;
        memAlloc.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &textures.prefilteredCube.deviceMemory));
        VK_CHECK_RESULT(vkBindImageMemory(device, textures.prefilteredCube.image, textures.prefilteredCube.deviceMemory, 0));
        // Image view
        VkImageViewCreateInfo viewCI = vks::initializers::imageViewCreateInfo();
        viewCI.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
        viewCI.format = format;
        viewCI.subresourceRange = {};
        viewCI.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewCI.subresourceRange.levelCount = numMips;
        viewCI.subresourceRange.layerCount = 6;
        viewCI.image = textures.prefilteredCube.image;
        VK_CHECK_RESULT(vkCreateImageView(device, &viewCI, nullptr, &textures.prefilteredCube.view));
        // Sampler
        VkSamplerCreateInfo samplerCI = vks::initializers::samplerCreateInfo();
        samplerCI.magFilter = VK_FILTER_LINEAR;
        samplerCI.minFilter = VK_FILTER_LINEAR;
        samplerCI.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerCI.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerCI.minLod = 0.0f;
        samplerCI.maxLod = static_cast<float>(numMips);
        samplerCI.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        VK_CHECK_RESULT(vkCreateSampler(device, &samplerCI, nullptr, &textures.prefilteredCube.sampler));

        textures.prefilteredCube.descriptor.imageView = textures.prefilteredCube.view;
        textures.prefilteredCube.descriptor.sampler = textures.prefilteredCube.sampler;
        textures.prefilteredCube.descriptor.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        textures.prefilteredCube.device = vulkanDevice;

        // FB, Att, RP, Pipe, etc.
        VkAttachmentDescription attDesc = {};
        // Color attachment
        attDesc.format = format;
        attDesc.samples = VK_SAMPLE_COUNT_1_BIT;
        attDesc.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
        attDesc.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        attDesc.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attDesc.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attDesc.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        attDesc.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        VkAttachmentReference colorReference = { 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };

        VkSubpassDescription subpassDescription = {};
        subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpassDescription.colorAttachmentCount = 1;
        subpassDescription.pColorAttachments = &colorReference;

        // Use subpass dependencies for layout transitions
        std::array<VkSubpassDependency, 2> dependencies;
        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
        dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

        // Renderpass
        VkRenderPassCreateInfo renderPassCI = vks::initializers::renderPassCreateInfo();
        renderPassCI.attachmentCount = 1;
        renderPassCI.pAttachments = &attDesc;
        renderPassCI.subpassCount = 1;
        renderPassCI.pSubpasses = &subpassDescription;
        renderPassCI.dependencyCount = 2;
        renderPassCI.pDependencies = dependencies.data();
        VkRenderPass renderpass;
        VK_CHECK_RESULT(vkCreateRenderPass(device, &renderPassCI, nullptr, &renderpass));

        struct {
            VkImage image;
            VkImageView view;
            VkDeviceMemory memory;
            VkFramebuffer framebuffer;
        } offscreen;

        // Offfscreen framebuffer
        {
            // Color attachment
            VkImageCreateInfo imageCreateInfo = vks::initializers::imageCreateInfo();
            imageCreateInfo.imageType = VK_IMAGE_TYPE_2D;
            imageCreateInfo.format = format;
            imageCreateInfo.extent.width = dim;
            imageCreateInfo.extent.height = dim;
            imageCreateInfo.extent.depth = 1;
            imageCreateInfo.mipLevels = 1;
            imageCreateInfo.arrayLayers = 1;
            imageCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
            imageCreateInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
            imageCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            imageCreateInfo.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
            imageCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            VK_CHECK_RESULT(vkCreateImage(device, &imageCreateInfo, nullptr, &offscreen.image));

            VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
            VkMemoryRequirements memReqs;
            vkGetImageMemoryRequirements(device, offscreen.image, &memReqs);
            memAlloc.allocationSize = memReqs.size;
            memAlloc.memoryTypeIndex = vulkanDevice->getMemoryType(memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, &offscreen.memory));
            VK_CHECK_RESULT(vkBindImageMemory(device, offscreen.image, offscreen.memory, 0));

            VkImageViewCreateInfo colorImageView = vks::initializers::imageViewCreateInfo();
            colorImageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
            colorImageView.format = format;
            colorImageView.flags = 0;
            colorImageView.subresourceRange = {};
            colorImageView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            colorImageView.subresourceRange.baseMipLevel = 0;
            colorImageView.subresourceRange.levelCount = 1;
            colorImageView.subresourceRange.baseArrayLayer = 0;
            colorImageView.subresourceRange.layerCount = 1;
            colorImageView.image = offscreen.image;
            VK_CHECK_RESULT(vkCreateImageView(device, &colorImageView, nullptr, &offscreen.view));

            VkFramebufferCreateInfo fbufCreateInfo = vks::initializers::framebufferCreateInfo();
            fbufCreateInfo.renderPass = renderpass;
            fbufCreateInfo.attachmentCount = 1;
            fbufCreateInfo.pAttachments = &offscreen.view;
            fbufCreateInfo.width = dim;
            fbufCreateInfo.height = dim;
            fbufCreateInfo.layers = 1;
            VK_CHECK_RESULT(vkCreateFramebuffer(device, &fbufCreateInfo, nullptr, &offscreen.framebuffer));

            VkCommandBuffer layoutCmd = VulkanExampleBase::createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);
            vks::tools::setImageLayout(
                layoutCmd,
                offscreen.image,
                VK_IMAGE_ASPECT_COLOR_BIT,
                VK_IMAGE_LAYOUT_UNDEFINED,
                VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            VulkanExampleBase::flushCommandBuffer(layoutCmd, queue, true);
        }

        // Descriptors
        VkDescriptorSetLayout descriptorsetlayout;
        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
            vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0),
        };
        VkDescriptorSetLayoutCreateInfo descriptorsetlayoutCI = vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings);
        VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorsetlayoutCI, nullptr, &descriptorsetlayout));

        // Descriptor Pool
        std::vector<VkDescriptorPoolSize> poolSizes = { vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1) };
        VkDescriptorPoolCreateInfo descriptorPoolCI = vks::initializers::descriptorPoolCreateInfo(poolSizes, 2);
        VkDescriptorPool descriptorpool;
        VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolCI, nullptr, &descriptorpool));

        // Descriptor sets
        VkDescriptorSet descriptorset;
        VkDescriptorSetAllocateInfo allocInfo =	vks::initializers::descriptorSetAllocateInfo(descriptorpool, &descriptorsetlayout, 1);
        VK_CHECK_RESULT(vkAllocateDescriptorSets(device, &allocInfo, &descriptorset));
        VkWriteDescriptorSet writeDescriptorSet = vks::initializers::writeDescriptorSet(descriptorset, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 0, &textures.environmentCube.descriptor);
        vkUpdateDescriptorSets(device, 1, &writeDescriptorSet, 0, nullptr);

        // Pipeline layout
        struct PushBlock {
            glm::mat4 mvp;
            float roughness;
            uint32_t numSamples = 32u;
        } pushBlock;

        VkPipelineLayout pipelinelayout;
        std::vector<VkPushConstantRange> pushConstantRanges = {
            vks::initializers::pushConstantRange(VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, sizeof(PushBlock), 0),
        };
        VkPipelineLayoutCreateInfo pipelineLayoutCI = vks::initializers::pipelineLayoutCreateInfo(&descriptorsetlayout, 1);
        pipelineLayoutCI.pushConstantRangeCount = 1;
        pipelineLayoutCI.pPushConstantRanges = pushConstantRanges.data();
        VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCI, nullptr, &pipelinelayout));

        // Pipeline
        VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = vks::initializers::pipelineInputAssemblyStateCreateInfo(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, VK_FALSE);
        VkPipelineRasterizationStateCreateInfo rasterizationState = vks::initializers::pipelineRasterizationStateCreateInfo(VK_POLYGON_MODE_FILL, VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE);
        VkPipelineColorBlendAttachmentState blendAttachmentState = vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE);
        VkPipelineColorBlendStateCreateInfo colorBlendState = vks::initializers::pipelineColorBlendStateCreateInfo(1, &blendAttachmentState);
        VkPipelineDepthStencilStateCreateInfo depthStencilState = vks::initializers::pipelineDepthStencilStateCreateInfo(VK_FALSE, VK_FALSE, VK_COMPARE_OP_LESS_OR_EQUAL);
        VkPipelineViewportStateCreateInfo viewportState = vks::initializers::pipelineViewportStateCreateInfo(1, 1);
        VkPipelineMultisampleStateCreateInfo multisampleState = vks::initializers::pipelineMultisampleStateCreateInfo(VK_SAMPLE_COUNT_1_BIT);
        std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamicState = vks::initializers::pipelineDynamicStateCreateInfo(dynamicStateEnables);
        // Vertex input state
        VkVertexInputBindingDescription vertexInputBinding = vks::initializers::vertexInputBindingDescription(0, vertexLayout.stride(), VK_VERTEX_INPUT_RATE_VERTEX);
        VkVertexInputAttributeDescription vertexInputAttribute = vks::initializers::vertexInputAttributeDescription(0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0);

        VkPipelineVertexInputStateCreateInfo vertexInputState = vks::initializers::pipelineVertexInputStateCreateInfo();
        vertexInputState.vertexBindingDescriptionCount = 1;
        vertexInputState.pVertexBindingDescriptions = &vertexInputBinding;
        vertexInputState.vertexAttributeDescriptionCount = 1;
        vertexInputState.pVertexAttributeDescriptions = &vertexInputAttribute;

        std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;

        VkGraphicsPipelineCreateInfo pipelineCI = vks::initializers::pipelineCreateInfo(pipelinelayout, renderpass);
        pipelineCI.pInputAssemblyState = &inputAssemblyState;
        pipelineCI.pRasterizationState = &rasterizationState;
        pipelineCI.pColorBlendState = &colorBlendState;
        pipelineCI.pMultisampleState = &multisampleState;
        pipelineCI.pViewportState = &viewportState;
        pipelineCI.pDepthStencilState = &depthStencilState;
        pipelineCI.pDynamicState = &dynamicState;
        pipelineCI.stageCount = 2;
        pipelineCI.pStages = shaderStages.data();
        pipelineCI.pVertexInputState = &vertexInputState;
        pipelineCI.renderPass = renderpass;

        shaderStages[0] = loadShader(getAssetPath() +  "shaders/pbribl/filtercube.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
        shaderStages[1] = loadShader(getAssetPath() +  "shaders/pbribl/prefilterenvmap.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);
        VkPipeline pipeline;
        VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, pipelineCache, 1, &pipelineCI, nullptr, &pipeline));

        // Render

        VkClearValue clearValues[1];
        clearValues[0].color = { { 0.0f, 0.0f, 0.2f, 0.0f } };

        VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
        // Reuse render pass from example pass
        renderPassBeginInfo.renderPass = renderpass;
        renderPassBeginInfo.framebuffer = offscreen.framebuffer;
        renderPassBeginInfo.renderArea.extent.width = dim;
        renderPassBeginInfo.renderArea.extent.height = dim;
        renderPassBeginInfo.clearValueCount = 1;
        renderPassBeginInfo.pClearValues = clearValues;

        std::vector<glm::mat4> matrices = {
            // POSITIVE_X
            glm::rotate(glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f)), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // NEGATIVE_X
            glm::rotate(glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f)), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // POSITIVE_Y
            glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // NEGATIVE_Y
            glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // POSITIVE_Z
            glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f)),
            // NEGATIVE_Z
            glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1.0f)),
        };

        VkCommandBuffer cmdBuf = vulkanDevice->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

        VkViewport viewport = vks::initializers::viewport((float)dim, (float)dim, 0.0f, 1.0f);
        VkRect2D scissor = vks::initializers::rect2D(dim, dim, 0, 0);

        vkCmdSetViewport(cmdBuf, 0, 1, &viewport);
        vkCmdSetScissor(cmdBuf, 0, 1, &scissor);

        VkImageSubresourceRange subresourceRange = {};
        subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        subresourceRange.baseMipLevel = 0;
        subresourceRange.levelCount = numMips;
        subresourceRange.layerCount = 6;

        // Change image layout for all cubemap faces to transfer destination
        vks::tools::setImageLayout(
            cmdBuf,
            textures.prefilteredCube.image,
            VK_IMAGE_LAYOUT_UNDEFINED,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            subresourceRange);

        for (uint32_t m = 0; m < numMips; m++) {
            pushBlock.roughness = (float)m / (float)(numMips - 1);
            for (uint32_t f = 0; f < 6; f++) {
                viewport.width = static_cast<float>(dim * std::pow(0.5f, m));
                viewport.height = static_cast<float>(dim * std::pow(0.5f, m));
                vkCmdSetViewport(cmdBuf, 0, 1, &viewport);

                // Render scene from cube face's point of view
                vkCmdBeginRenderPass(cmdBuf, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

                // Update shader push constant block
                pushBlock.mvp = glm::perspective((float)(M_PI / 2.0), 1.0f, 0.1f, 512.0f) * matrices[f];

                vkCmdPushConstants(cmdBuf, pipelinelayout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushBlock), &pushBlock);

                vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
                vkCmdBindDescriptorSets(cmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelinelayout, 0, 1, &descriptorset, 0, NULL);

                VkDeviceSize offsets[1] = { 0 };

                vkCmdBindVertexBuffers(cmdBuf, 0, 1, &skybox.vertices.buffer, offsets);
                vkCmdBindIndexBuffer(cmdBuf, skybox.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
                vkCmdDrawIndexed(cmdBuf, skybox.indexCount, 1, 0, 0, 0);

                vkCmdEndRenderPass(cmdBuf);

                vks::tools::setImageLayout(
                    cmdBuf,
                    offscreen.image,
                    VK_IMAGE_ASPECT_COLOR_BIT,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);

                // Copy region for transfer from framebuffer to cube face
                VkImageCopy copyRegion = {};

                copyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copyRegion.srcSubresource.baseArrayLayer = 0;
                copyRegion.srcSubresource.mipLevel = 0;
                copyRegion.srcSubresource.layerCount = 1;
                copyRegion.srcOffset = { 0, 0, 0 };

                copyRegion.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copyRegion.dstSubresource.baseArrayLayer = f;
                copyRegion.dstSubresource.mipLevel = m;
                copyRegion.dstSubresource.layerCount = 1;
                copyRegion.dstOffset = { 0, 0, 0 };

                copyRegion.extent.width = static_cast<uint32_t>(viewport.width);
                copyRegion.extent.height = static_cast<uint32_t>(viewport.height);
                copyRegion.extent.depth = 1;

                vkCmdCopyImage(
                    cmdBuf,
                    offscreen.image,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    textures.prefilteredCube.image,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    1,
                    &copyRegion);

                // Transform framebuffer color attachment back
                vks::tools::setImageLayout(
                    cmdBuf,
                    offscreen.image,
                    VK_IMAGE_ASPECT_COLOR_BIT,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
            }
        }

        vks::tools::setImageLayout(
            cmdBuf,
            textures.prefilteredCube.image,
            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            subresourceRange);

        vulkanDevice->flushCommandBuffer(cmdBuf, queue);

        // todo: cleanup
        vkDestroyRenderPass(device, renderpass, nullptr);
        vkDestroyFramebuffer(device, offscreen.framebuffer, nullptr);
        vkFreeMemory(device, offscreen.memory, nullptr);
        vkDestroyImageView(device, offscreen.view, nullptr);
        vkDestroyImage(device, offscreen.image, nullptr);
        vkDestroyDescriptorPool(device, descriptorpool, nullptr);
        vkDestroyDescriptorSetLayout(device, descriptorsetlayout, nullptr);
        vkDestroyPipeline(device, pipeline, nullptr);
        vkDestroyPipelineLayout(device, pipelinelayout, nullptr);

        auto tEnd = std::chrono::high_resolution_clock::now();
        auto tDiff = std::chrono::duration<double, std::milli>(tEnd - tStart).count();
        std::cout << "Generating pre-filtered enivornment cube with " << numMips << " mip levels took " << tDiff << " ms" << std::endl;
    }

    // Prepare and initialize uniform buffer containing shader uniforms
    void prepareUniformBuffers()
    {
        // Objact vertex shader uniform buffer
        VK_CHECK_RESULT(vulkanDevice->createBuffer(
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            &uniformBuffers.matrices,
            sizeof(uboMatrices)));

        // Skybox vertex shader uniform buffer
        VK_CHECK_RESULT(vulkanDevice->createBuffer(
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            &uniformBuffers.skybox,
            sizeof(uboMatrices)));

        // Shared parameter uniform buffer
        VK_CHECK_RESULT(vulkanDevice->createBuffer(
            VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            &uniformBuffers.params,
            sizeof(uboParams)));

        // Map persistent
        VK_CHECK_RESULT(uniformBuffers.matrices.map());
        VK_CHECK_RESULT(uniformBuffers.skybox.map());
        VK_CHECK_RESULT(uniformBuffers.params.map());

        updateUniformBuffers();
        updateParams();
    }

    void updateUniformBuffers()
    {
        // 3D object
        uboMatrices.projection = camera.matrices.perspective;
        uboMatrices.view = camera.matrices.view;
        uboMatrices.model = glm::mat4();
        uboMatrices.camPos = camera.position * -1.0f;
        memcpy(uniformBuffers.matrices.mapped, &uboMatrices, sizeof(uboMatrices));

        // Skybox
        uboMatrices.model = glm::mat4(glm::mat3(camera.matrices.view));
        memcpy(uniformBuffers.skybox.mapped, &uboMatrices, sizeof(uboMatrices));
    }

    void updateParams()
    {
        const float p = 15.0f;
        uboParams.lights[0] = glm::vec4(-p, -p*0.5f, -p, 1.0f);
        uboParams.lights[1] = glm::vec4(-p, -p*0.5f,  p, 1.0f);
        uboParams.lights[2] = glm::vec4( p, -p*0.5f,  p, 1.0f);
        uboParams.lights[3] = glm::vec4( p, -p*0.5f, -p, 1.0f);

        memcpy(uniformBuffers.params.mapped, &uboParams, sizeof(uboParams));
    }

    float sloap = 7.f * M_PI/180.0f;
    btVector3 upVector = btVector3(0, cos(sloap), sin(sloap));


    void initTableBorderHull () {
        if (plateShape != nullptr) {
            dynamicsWorld->removeRigidBody(plateRB);
            dynamicsWorld->removeRigidBody(borderRB);
            delete plateRB;
            delete plateShape;

            delete borderShape;
            delete borderState;
            delete borderRB;
        }

        modBodies = new vks::ModelGroup(vulkanDevice, queue);
        btCollisionShape* shape = nullptr;
        btRigidBody* body = nullptr;
        int modBodiesIdx = -1;

        shape = new btStaticPlaneShape(upVector, 0);
        shape->setMargin(0.001);

        btRigidBody::btRigidBodyConstructionInfo rbci(0, nullptr, shape, btVector3(0, 0, 0));
        rbci.m_restitution = 0;
        rbci.m_friction = 1.3;
        btStaticObjs.push_back(new btRigidBody(rbci));

        //table
        for (int i = 0; i<modGrp->models[modPlateIdx].parts.size() ; i++) {
            shape = modGrp->getConvexHullShape(modPlateIdx, i);
            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape , btVector3(0, 0, 0));
            rbci.m_restitution = 0.8;
            rbci.m_friction = 0.2;
            btStaticObjs.push_back (new btRigidBody(rbci));
        }

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
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-ramp.obj");
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {
            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape , btVector3(0, 0, 0));
            rbci.m_restitution = 0.;
            rbci.m_friction = 0.;
            btStaticObjs.push_back(new btRigidBody(rbci));
        }

        //bumpers
        modBodiesIdx = modBodies->addModel(getAssetPath() + "models/pinball-bump.obj");
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {
            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            rbci = btRigidBody::btRigidBodyConstructionInfo (0, nullptr, shape, btVector3(0, 0, 0));
            rbci.m_restitution = 1.2;
            rbci.m_friction = 0.;

            body = new btRigidBody(rbci);

            dynamicsWorld->addRigidBody(body);
            body->setUserIndex(BUMP_BDY_ID);
        }
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
        //flippers
        for (int i = 0; i<modBodies->models[modBodiesIdx].parts.size() ; i++) {

            shape = modBodies->getConvexHullShape(modBodiesIdx, i);
            shape->setMargin(0.0034);
            worldObjs[i+worldObjFlip].motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), upVector*0.003));
            btScalar mass = 0.02;
            btVector3 fallInertia(0, 0, 0);
            shape->calculateLocalInertia(mass, fallInertia);

            btRigidBody::btRigidBodyConstructionInfo borderRigidBodyCI(mass, worldObjs[i+worldObjFlip].motionState, shape, fallInertia);
            borderRigidBodyCI.m_restitution = 0.;
            borderRigidBodyCI.m_friction = 0.2;


            worldObjs[i+worldObjFlip].body = new btRigidBody(borderRigidBodyCI);

            dynamicsWorld->addRigidBody(worldObjs[i+worldObjFlip].body,0x02,0x01);

            //worldObjs[i+worldObjFlip].body->setContactStiffnessAndDamping(300,10);
            //worldObjs[i+worldObjFlip].body->setAngularFactor(btVector3(0,0.2,0));
            //worldObjs[i+worldObjFlip].body->setLinearFactor(btVector3(1,0,1));
        }
        btHingeConstraint* hinge = new btHingeConstraint(*worldObjs[worldObjFlip+1].body, btVector3(-0.09220,-0.01478,0.39887),upVector,true);
        hinge->setLimit(-0.3, 0.5, 1000.f,0.9f,1);
        //hinge->setBreakingImpulseThreshold(200000000);
        dynamicsWorld->addConstraint(hinge);


        hinge = new btHingeConstraint(*worldObjs[worldObjFlip].body, btVector3(0.09220,-0.01478,0.39887),upVector,true);
        hinge->setLimit(-0.5, 0.3, 1000.f, 0.9f, 1);
        dynamicsWorld->addConstraint(hinge);

        modBodies->destroy();
    }

    float iterations = 20;
    bool splitImpulse = true;

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

        initTableBorderHull();

        ballShape = new btSphereShape(gameTypes[currentGameType].ballSize * 0.5);
        //ballShape->setMargin(0.002);
        btScalar mass = 0.08;
        btVector3 fallInertia(0, 0, 0);
        ballShape->calculateLocalInertia(mass, fallInertia);

        for (int i=0; i < 1; i++) {
            //balls[i].motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(i * 0.5, 5 + i *2, i*0.1f)));
            worldObjs[i].motionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.05,1,3)));

            btRigidBody::btRigidBodyConstructionInfo ballRigidBodyCI (mass, worldObjs[i].motionState, ballShape, fallInertia);
            ballRigidBodyCI.m_restitution = 0.5f;
            ballRigidBodyCI.m_friction = 0.2;

            //ballRigidBodyCI.m_rollingFriction = 0.000001;
            worldObjs[i].body = new btRigidBody(ballRigidBodyCI);
            dynamicsWorld->addRigidBody (worldObjs[i].body,0x01,0xff);

            worldObjs[i].body->setUserIndex(BALL_BDY_ID);
            worldObjs[i].body->setFriction(0.01f);
            worldObjs[i].body->setRollingFriction(.00005);
            worldObjs[i].body->setSpinningFriction(0.1);
            worldObjs[i].body->setAnisotropicFriction(ballShape->getAnisotropicRollingFrictionDirection(),btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
            worldObjs[i].body->setCcdMotionThreshold(gameTypes[currentGameType].ballSize * 0.5f);
            worldObjs[i].body->setCcdSweptSphereRadius(gameTypes[currentGameType].ballSize * 0.1f);

        }
    }

    btVector3 pushDir = btVector3(0,0,-1);

    void update_physics () {

        for (int i=0; i < 3; i++) {
            btTransform trans;
            worldObjs[i].body->getMotionState()->getWorldTransform(trans);
            //trans.getOpenGLMatrix(glm::value_ptr(data[i].mat));
            btVector3 o = trans.getOrigin();

            btQuaternion btQ = trans.getRotation();
            glm::quat q = glm::quat (btQ.getW(), btQ.getX(), btQ.getY(), btQ.getZ());

            modGrp->instanceDatas[i].modelMat =
                    glm::translate(glm::mat4(),glm::vec3(-o.getX(), -o.getY(), -o.getZ())) * glm::mat4(q);
        }
        modGrp->updateInstancesBuffer();
    }

    clock_t lastTime;

    float maxSubsteps = 10.f;
    float fixedTimeStepDiv = 1000.0;
    int subSteps = 0;

    void step_physics () {
        float multi = 3.;
        if (leftFlip) {
            worldObjs[2].body->applyTorque(upVector*multi);
        }else {
            worldObjs[2].body->applyTorque(-upVector*multi);
        }

        if (rightFlip) {
            worldObjs[1].body->applyTorque(-upVector*multi);
        }else {
            worldObjs[1].body->applyTorque(upVector*multi);
        }
        clock_t time = clock();

        float diff = float(diffclock(time, lastTime));

        if (diff > 1.0/1000.0) {
            lastTime = time;
            subSteps = dynamicsWorld->stepSimulation(diff,int(maxSubsteps),1.0/fixedTimeStepDiv);
            update_physics();
            check_collisions(dynamicsWorld, this);

            if (leftTargetGroup.reachedTarget == leftTargetGroup.targetCount)
                leftTargetGroup.tryReset(time);
        }

        //dynamicsWorld->stepSimulation(1.0/1600.0,10,1.0/1000.0);


    }

    void draw()
    {
        VulkanExampleBase::prepareFrame();

        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &drawCmdBuffers[currentBuffer];
        VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE));

        VulkanExampleBase::submitFrame();
    }

    void prepare()
    {
        VulkanExampleBase::prepare();
        loadAssets();

        init_physics();

        generateBRDFLUT();
        generateIrradianceCube();
        generatePrefilteredCube();
        prepareUniformBuffers();
        setupDescriptors();
        preparePipelines();
        buildCommandBuffers();
        prepared = true;
    }
    const float inpulse = 2.f;
    const int target = 2;
    bool leftFlip = false;
    bool rightFlip = false;
    virtual void keyDown(uint32_t key) {
        //btVector3 torc = upVector.rotate(btVector3(1,0,0), M_PI_2);
        btVector3 torc = btVector3(1,1,1);
        switch (key) {
        case 37://left ctrl
            leftFlip = true;
            //worldObjs[2].body->activate();
            break;
        case 105://right ctrl
            rightFlip = true;
            worldObjs[1].body->activate();
            break;
        }
    }
    virtual void keyUp(uint32_t key) {
        switch (key) {
        case 37://left ctrl
            leftFlip = false;
            //worldObjs[2].body->activate();
            break;
        case 105://right ctrl
            rightFlip = false;
            worldObjs[1].body->activate();
            break;
        }
    }
    virtual void keyPressed(uint32_t key) {
        switch (key) {
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
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.,0.02,0.)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 84:
            //pushDir = btVector3(1,0,0);
            //worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.4457,0.05,0.535)));
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.266,0.05,0.2)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 85://6
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.,0.02,-0.2)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 87://1
            worldObjs[0].body->setWorldTransform(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-0.11,0.02,0.22)));
            worldObjs[0].body->setLinearVelocity(btVector3(0,0,0));
            break;
        case 88:
            pushDir = btVector3(0,0,inpulse);
            break;
        case 89:
            pushDir = btVector3(inpulse,0,inpulse);
            break;
        }
    }
    virtual void render()
    {
        if (!prepared)
            return;
        step_physics();
        draw();
    }

    virtual void viewChanged()
    {
        updateUniformBuffers();
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
            trans = obj->getWorldTransform();
            btVector3 center;
            btScalar radius;
            obj->getCollisionShape()->getBoundingSphere(center, radius);
            trans = vkapp->worldObjs[0].body->getWorldTransform();
            btVector3 vDir = trans.getOrigin() - center;
            vkapp->worldObjs[0].body->clearForces();
            vkapp->worldObjs[0].body->applyImpulse(-vDir.normalized()*0.1,btVector3(0,0,0));
            break;
        }
        case DAMP_BDY_ID:
        {
            std::vector<btManifoldPoint*>& manifoldPoints = objectsCollisions[body];
            if (manifoldPoints.size()==0)
                continue;
            //btVector3 vDir = manifoldPoints[0]->getPositionWorldOnA() - manifoldPoints[0]->getPositionWorldOnB();
            vkapp->worldObjs[0].body->applyImpulse(-manifoldPoints[0]->m_normalWorldOnB*0.08,btVector3(0,0,0));
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
            if (vkapp->leftTargetGroup.reachedTarget == vkapp->leftTargetGroup.targetCount)
                vkapp->leftTargetGroup.reachedTime = clock();
            break;
        }
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
