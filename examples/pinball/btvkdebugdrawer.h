#ifndef BTVKDEBUGDRAWER_H
#define BTVKDEBUGDRAWER_H

#include "vulkanexamplebase.h"
#include "VulkanModel.hpp"
#include "VulkanSwapChain.hpp"

#include "LinearMath/btIDebugDraw.h"

class btVKDebugDrawer : public btIDebugDraw
{
    int m_debugMode;

    VkSampleCountFlagBits       sampleCount;

    vks::VulkanDevice*          device;
    VulkanSwapChain             swapChain;
    VkRenderPass                renderPass;
    std::vector<VkFramebuffer>  frameBuffers;
    VkDescriptorSet             dsUboMatrices;

    VkCommandPool               commandPool;
    std::vector<VkCommandBuffer>cmdBuffers;

    VkDescriptorPool        descriptorPool;
    VkDescriptorSetLayout   descriptorSetLayout;
    VkDescriptorSet         descriptorSet;

    VkFence                 fence;
    VkSubmitInfo            submitInfo;

    VkPipeline              pipeline;
    VkPipelineLayout        pipelineLayout;
    VkPipelineCache         pipelineCache;


    vks::Image*         imgDepth;

    std::vector<float>	vertices;
    vks::Buffer			vertexBuff;
    uint32_t			vBufferSize = 10000 * sizeof(float) * 6;


    vks::VertexLayout vertexLayout = vks::VertexLayout({
        vks::VERTEX_COMPONENT_POSITION,
        vks::VERTEX_COMPONENT_COLOR,
    });

    void prepareRenderPass();
    void prepareDescriptors();
    void preparePipeline();

    const std::string getAssetPath();
    VkPipelineShaderStageCreateInfo loadShader(std::string fileName, VkShaderStageFlagBits stage);

public:
    VkSemaphore             drawComplete;
    uint32_t			vertexCount = 0;

    void buildCommandBuffer ();
    void submit (VkQueue queue, uint32_t bufferindex, VkSemaphore waitSemaphore);

    btVKDebugDrawer(vks::VulkanDevice* _device, VulkanSwapChain _swapChain, vks::Image* _depthImg, VkSampleCountFlagBits _sampleCount, std::vector<VkFramebuffer> &_frameBuffers, VkDescriptorSet _dsUboMatrices);
    virtual ~btVKDebugDrawer();

    virtual void drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor);

    virtual void drawLine(const btVector3& from,const btVector3& to,const btVector3& color);

    virtual void drawSphere (const btVector3& p, btScalar radius, const btVector3& color);

    virtual void drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);

    virtual void drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);

    virtual void reportErrorWarning(const char* warningString);

    virtual void draw3dText(const btVector3& location,const char* textString);

    virtual void setDebugMode(int debugMode);

    virtual int	getDebugMode() const { return m_debugMode;}

    virtual void clearLines();

    virtual void flushLines();
};

#endif // BTVKDEBUGDRAWER_H
