/*
* Physical based rendering with image based lighting
*
* Copyright (C) 2016-2017 by Sascha Willems - www.saschawillems.de
* Copyright (C) 2018 by JP Bruyère
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <vector>
#include <map>

#include <gli/gli.hpp>

//#include <vulkan/vulkan.h>
#include "vulkanexamplebase.h"

#include <glm/gtc/matrix_transform.hpp>

#include "VulkanBuffer.hpp"
#include "VulkanTexture.hpp"
#include "VulkanModel.hpp"
#include "ModelGroup.hpp"

struct OldMaterial {
    // Parameter block used as push constant block
    struct PushBlock {
        float roughness = 0.0f;
        float metallic = 0.0f;
        float specular = 0.0f;
        float r, g, b;
    } params;
    std::string name;
    OldMaterial() {}
    OldMaterial(std::string n, glm::vec3 c) : name(n) {
        params.r = c.r;
        params.g = c.g;
        params.b = c.b;
    }
};

double diffclock( clock_t clock1, clock_t clock2 );

class VulkanExampleVk : public VulkanExampleBase
{
public:
    bool displaySkybox = true;
    vks::ModelGroup* modGrp;

    struct Textures {
        vks::TextureCubeMap environmentCube;
        // Generated at runtime
        vks::Texture2D lutBrdf;
        vks::TextureCubeMap irradianceCube;
        vks::TextureCubeMap prefilteredCube;
    } textures;

    struct {
        struct {
            VkImage image;
            VkImageView view;
            VkDeviceMemory memory;
        } color;
        struct {
            VkImage image;
            VkImageView view;
            VkDeviceMemory memory;
        } depth;
    } multisampleTarget;

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

    VulkanExampleVk(bool enableValidation);
    virtual ~VulkanExampleVk();

    virtual void getEnabledFeatures();

    void buildCommandBuffers();

    void parsebmFont();

    virtual void loadAssets();

    void setupDescriptors();

    virtual void preparePipelines();
    void generateBRDFLUT();
    void generateIrradianceCube();
    void generatePrefilteredCube();
    void prepareUniformBuffers();
    void updateUniformBuffers();
    void updateParams();
    void draw();
    void setupMultisampleTarget();
    void setupFrameBuffer();
    void prepare();
    virtual void render();
    virtual void viewChanged();
};
