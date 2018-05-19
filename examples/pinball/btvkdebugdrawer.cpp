#include "btvkdebugdrawer.h"


btVKDebugDrawer::btVKDebugDrawer(vks::VulkanDevice *_device, VkRenderPass _renderPass, VkSampleCountFlagBits _sampleCount, VkDescriptorSetLayout _descriptorSetLayout)
:m_debugMode(0)
{
    device = _device->logicalDevice;
    renderPass = _renderPass;
    sampleCount = _sampleCount;
    descriptorSetLayout = _descriptorSetLayout;

    preparePipeline ();

    VK_CHECK_RESULT(_device->createBuffer(
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
        &vertexBuff ,
        vBufferSize));

    vertexBuff.map();
}

btVKDebugDrawer::~btVKDebugDrawer()
{
    vertexBuff.unmap();
    vertexBuff.destroy();

    vkDestroyPipeline(device, pipeline, VK_NULL_HANDLE);
    vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
}

void btVKDebugDrawer::preparePipeline () {
    VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
        vks::initializers::pipelineInputAssemblyStateCreateInfo(
            VK_PRIMITIVE_TOPOLOGY_LINE_LIST,
            0,
            VK_FALSE);

    VkPipelineRasterizationStateCreateInfo rasterizationState =
        vks::initializers::pipelineRasterizationStateCreateInfo(
            VK_POLYGON_MODE_FILL,
            VK_CULL_MODE_NONE,
            VK_FRONT_FACE_CLOCKWISE,
            0);

    VkPipelineColorBlendAttachmentState blendAttachmentState =
        vks::initializers::pipelineColorBlendAttachmentState(
            0xf,
            VK_FALSE);

    VkPipelineColorBlendStateCreateInfo colorBlendState =
        vks::initializers::pipelineColorBlendStateCreateInfo(
            1,
            &blendAttachmentState);

    VkPipelineDepthStencilStateCreateInfo depthStencilState =
        vks::initializers::pipelineDepthStencilStateCreateInfo(
            VK_FALSE,
            VK_FALSE,
            VK_COMPARE_OP_LESS_OR_EQUAL);

    VkPipelineViewportStateCreateInfo viewportState =
        vks::initializers::pipelineViewportStateCreateInfo(1, 1, 0);

    std::vector<VkDynamicState> dynamicStateEnables = {
        VK_DYNAMIC_STATE_VIEWPORT,
        VK_DYNAMIC_STATE_SCISSOR
    };
    VkPipelineDynamicStateCreateInfo dynamicState =
        vks::initializers::pipelineDynamicStateCreateInfo(
            dynamicStateEnables.data(),
            dynamicStateEnables.size(),
            0);

    // Pipeline layout
    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo = vks::initializers::pipelineLayoutCreateInfo(&descriptorSetLayout, 1);
    VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCreateInfo, nullptr, &pipelineLayout));

    VkGraphicsPipelineCreateInfo pipelineCreateInfo =
        vks::initializers::pipelineCreateInfo(
            pipelineLayout,
            renderPass,
            0);

    VkPipelineMultisampleStateCreateInfo multisampleState =
            vks::initializers::pipelineMultisampleStateCreateInfo(sampleCount,0);

    // MSAA with sample shading pipeline
    // Sample shading enables per-sample shading to avoid shader aliasing and smooth out e.g. high frequency texture maps
    // Note: This will trade performance for are more stable image
    //multisampleState.rasterizationSamples = sampleCount;
    if (sampleCount > VK_SAMPLE_COUNT_1_BIT) {
        multisampleState.sampleShadingEnable = VK_TRUE;				// Enable per-sample shading (instead of per-fragment)
        multisampleState.minSampleShading = 0.25f;
    }

    // Vertex bindings an attributes
    // Binding description
    std::vector<VkVertexInputBindingDescription> vertexInputBindings = {
        vks::initializers::vertexInputBindingDescription(0, vertexLayout.stride(), VK_VERTEX_INPUT_RATE_VERTEX),
    };

    // Attribute descriptions
    std::vector<VkVertexInputAttributeDescription> vertexInputAttributes = {
        vks::initializers::vertexInputAttributeDescription(0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0),					// Position
        vks::initializers::vertexInputAttributeDescription(0, 1, VK_FORMAT_R32G32B32_SFLOAT, sizeof(float) * 3),	// color
    };

    VkPipelineVertexInputStateCreateInfo vertexInputState = vks::initializers::pipelineVertexInputStateCreateInfo();
    vertexInputState.vertexBindingDescriptionCount = static_cast<uint32_t>(vertexInputBindings.size());
    vertexInputState.pVertexBindingDescriptions = vertexInputBindings.data();
    vertexInputState.vertexAttributeDescriptionCount = static_cast<uint32_t>(vertexInputAttributes.size());
    vertexInputState.pVertexAttributeDescriptions = vertexInputAttributes.data();

    std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages;
    shaderStages[0] = loadShader(getAssetPath() +  "shaders/pinball/debugDraw.vert.spv", VK_SHADER_STAGE_VERTEX_BIT);
    shaderStages[1] = loadShader(getAssetPath() +  "shaders/pinball/debugDraw.frag.spv", VK_SHADER_STAGE_FRAGMENT_BIT);

    pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
    pipelineCreateInfo.pRasterizationState = &rasterizationState;
    pipelineCreateInfo.pColorBlendState = &colorBlendState;
    pipelineCreateInfo.pMultisampleState = &multisampleState;
    pipelineCreateInfo.pViewportState = &viewportState;
    pipelineCreateInfo.pDepthStencilState = &depthStencilState;
    pipelineCreateInfo.pDynamicState = &dynamicState;
    pipelineCreateInfo.stageCount = static_cast<uint32_t>(shaderStages.size());
    pipelineCreateInfo.pStages = shaderStages.data();
    pipelineCreateInfo.pVertexInputState = &vertexInputState;

    VK_CHECK_RESULT(vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineCreateInfo, nullptr, &pipeline));

    vkDestroyShaderModule(device, shaderStages[0].module, nullptr);
    vkDestroyShaderModule(device, shaderStages[1].module, nullptr);
}

void btVKDebugDrawer::buildCommandBuffer (VkCommandBuffer cmd){
    if (vertexCount > 0) {
        VkDeviceSize offsets[1] = { 0 };
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuff.buffer, offsets);
        vkCmdDraw (cmd,  vertexCount, 1, 0, 0);
    }
}
void btVKDebugDrawer::clearLines(){
    vertices.clear();
    vertexCount = 0;
}

void btVKDebugDrawer::flushLines(){
    memcpy(vertexBuff.mapped, vertices.data(), vertices.size() * sizeof(float));
    vertexCount = vertices.size() / 6;
}

void btVKDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor)
{
    vertices.push_back(-from.getX());
    vertices.push_back(-from.getY());
    vertices.push_back(-from.getZ());

    vertices.push_back(fromColor.getX());
    vertices.push_back(fromColor.getY());
    vertices.push_back(fromColor.getZ());

    vertices.push_back(-to.getX());
    vertices.push_back(-to.getY());
    vertices.push_back(-to.getZ());

    vertices.push_back(toColor.getX());
    vertices.push_back(toColor.getY());
    vertices.push_back(toColor.getZ());
}

void btVKDebugDrawer::drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
{
    drawLine(from,to,color,color);
}

void btVKDebugDrawer::drawSphere (const btVector3& p, btScalar radius, const btVector3& color)
{
//	glColor4f (color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
//	glPushMatrix ();
//	glTranslatef (p.getX(), p.getY(), p.getZ());

//	int lats = 5;
//	int longs = 5;

//	int i, j;
//	for(i = 0; i <= lats; i++) {
//		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lats);
//		btScalar z0  = radius*sin(lat0);
//		btScalar zr0 =  radius*cos(lat0);

//		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lats);
//		btScalar z1 = radius*sin(lat1);
//		btScalar zr1 = radius*cos(lat1);

//		glBegin(GL_QUAD_STRIP);
//		for(j = 0; j <= longs; j++) {
//			btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longs;
//			btScalar x = cos(lng);
//			btScalar y = sin(lng);

//			glNormal3f(x * zr0, y * zr0, z0);
//			glVertex3f(x * zr0, y * zr0, z0);
//			glNormal3f(x * zr1, y * zr1, z1);
//			glVertex3f(x * zr1, y * zr1, z1);
//		}
//		glEnd();
//	}

//	glPopMatrix();
}



void btVKDebugDrawer::drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha)
{
    if (m_debugMode > 1)
    {
        drawLine(a,b,color);
        drawLine(b,c,color);
        drawLine(c,a,color);
    }
}

void btVKDebugDrawer::setDebugMode(int debugMode)
{
    m_debugMode = debugMode;

}

void btVKDebugDrawer::draw3dText(const btVector3& location,const char* textString)
{
    //glRasterPos3f(location.x(),  location.y(),  location.z());
    //BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),textString);
}

void btVKDebugDrawer::reportErrorWarning(const char* warningString)
{
    printf("%s\n",warningString);
}

void btVKDebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
    drawLine(pointOnB,pointOnB+normalOnB*0.1,color);
}

#if !(defined(VK_USE_PLATFORM_IOS_MVK) || defined(VK_USE_PLATFORM_MACOS_MVK))
const std::string btVKDebugDrawer::getAssetPath()
{
#if defined(VK_USE_PLATFORM_ANDROID_KHR)
    return "";
#elif defined(VK_EXAMPLE_DATA_DIR)
    return VK_EXAMPLE_DATA_DIR;
#else
    return "./../data/";
#endif
}
#endif

VkPipelineShaderStageCreateInfo btVKDebugDrawer::loadShader(std::string fileName, VkShaderStageFlagBits stage)
{
    VkPipelineShaderStageCreateInfo shaderStage = {};
    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = stage;
#if defined(VK_USE_PLATFORM_ANDROID_KHR)
    shaderStage.module = vks::tools::loadShader(androidApp->activity->assetManager, fileName.c_str(), device);
#else
    shaderStage.module = vks::tools::loadShader(fileName.c_str(), device);
#endif
    shaderStage.pName = "main"; // todo : make param
    assert(shaderStage.module != VK_NULL_HANDLE);
    return shaderStage;
}

