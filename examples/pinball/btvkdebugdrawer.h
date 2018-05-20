#ifndef BTVKDEBUGDRAWER_H
#define BTVKDEBUGDRAWER_H

#include "vulkanexamplebase.h"
#include "VulkanModel.hpp"

#include "LinearMath/btIDebugDraw.h"

class btVKDebugDrawer : public btIDebugDraw
{
	int m_debugMode;

	VkSampleCountFlagBits sampleCount;
	VkDevice			device;
	VkRenderPass		renderPass;
	VkPipeline			pipeline;
	VkPipelineLayout	pipelineLayout;
	VkDescriptorSetLayout descriptorSetLayout;

	std::vector<float>	vertices;
	vks::Buffer			vertexBuff;
	uint32_t			vBufferSize = 10000 * sizeof(float) * 6;


	vks::VertexLayout vertexLayout = vks::VertexLayout({
		vks::VERTEX_COMPONENT_POSITION,
		vks::VERTEX_COMPONENT_COLOR,
	});

	void preparePipeline();

	const std::string getAssetPath();
	VkPipelineShaderStageCreateInfo loadShader(std::string fileName, VkShaderStageFlagBits stage);

public:
	uint32_t			vertexCount = 0;

	void buildCommandBuffer (VkCommandBuffer cmd);

	btVKDebugDrawer(vks::VulkanDevice *_device, VkRenderPass _renderPass, VkSampleCountFlagBits _sampleCount, VkDescriptorSetLayout _descriptorSetLayout);
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
