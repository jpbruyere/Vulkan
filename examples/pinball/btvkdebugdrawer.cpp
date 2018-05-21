#include "btvkdebugdrawer.h"

const VkPipelineStageFlags stageFlags = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
btVKDebugDrawer::btVKDebugDrawer(vks::VulkanDevice* _device, VulkanSwapChain _swapChain, vks::Image *_depthImg, VkSampleCountFlagBits _sampleCount,
                                 std::vector<VkFramebuffer>&_frameBuffers, VkDescriptorSet _dsUboMatrices)
:m_debugMode(0)
{
    swapChain   = _swapChain;
    device      = _device;
    imgDepth    = _depthImg;
    sampleCount = _sampleCount;
    frameBuffers= _frameBuffers;
    dsUboMatrices= _dsUboMatrices;

    prepareRenderPass();

    fence       = device->createFence();
    drawComplete= device->createSemaphore();


    submitInfo = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submitInfo.pWaitDstStageMask = &stageFlags;
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = &drawComplete;

    // Command buffer
    VkCommandPoolCreateInfo cmdPoolInfo = {};
    cmdPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    cmdPoolInfo.queueFamilyIndex = device->queueFamilyIndices.graphics;
    cmdPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    VK_CHECK_RESULT(vkCreateCommandPool(device->logicalDevice, &cmdPoolInfo, nullptr, &commandPool));

    cmdBuffers.resize(swapChain.imageCount);

    VkCommandBufferAllocateInfo cmdBufAllocateInfo =
        vks::initializers::commandBufferAllocateInfo(commandPool, VK_COMMAND_BUFFER_LEVEL_PRIMARY, static_cast<uint32_t>(cmdBuffers.size()));
    VK_CHECK_RESULT(vkAllocateCommandBuffers(device->logicalDevice, &cmdBufAllocateInfo, cmdBuffers.data()));


    // Command buffer execution fence
    VkFenceCreateInfo fenceCreateInfo = vks::initializers::fenceCreateInfo();
    VK_CHECK_RESULT(vkCreateFence(device->logicalDevice, &fenceCreateInfo, nullptr, &fence));


    prepareDescriptors();
    preparePipeline ();

    VK_CHECK_RESULT(device->createBuffer(
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

    vkDestroyRenderPass     (device->logicalDevice, renderPass, VK_NULL_HANDLE);
    vkDestroyPipeline       (device->logicalDevice, pipeline, VK_NULL_HANDLE);
    vkDestroyPipelineLayout (device->logicalDevice, pipelineLayout, VK_NULL_HANDLE);

    device->destroyFence        (fence);
    device->destroySemaphore    (drawComplete);
}
void btVKDebugDrawer::prepareRenderPass()
{
    VkAttachmentDescription attachments[2] = {};

    // Color attachment
    attachments[0].format = swapChain.colorFormat;
    attachments[0].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
    attachments[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    attachments[0].finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    // Depth attachment
    attachments[1].format = imgDepth->infos.format;
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorReference = {};
    colorReference.attachment = 0;
    colorReference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference depthReference = {};
    depthReference.attachment = 1;
    depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkSubpassDependency subpassDependencies[2] = {};

    // Transition from final to initial (VK_SUBPASS_EXTERNAL refers to all commmands executed outside of the actual renderpass)
    subpassDependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    subpassDependencies[0].dstSubpass = 0;
    subpassDependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    subpassDependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    subpassDependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
    subpassDependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    subpassDependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

    // Transition from initial to final
    subpassDependencies[1].srcSubpass = 0;
    subpassDependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    subpassDependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    subpassDependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    subpassDependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    subpassDependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
    subpassDependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

    VkSubpassDescription subpassDescription = {};
    subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpassDescription.flags = 0;
    subpassDescription.inputAttachmentCount = 0;
    subpassDescription.pInputAttachments = NULL;
    subpassDescription.colorAttachmentCount = 1;
    subpassDescription.pColorAttachments = &colorReference;
    subpassDescription.pResolveAttachments = NULL;
    subpassDescription.pDepthStencilAttachment = &depthReference;
    subpassDescription.preserveAttachmentCount = 0;
    subpassDescription.pPreserveAttachments = NULL;

    VkRenderPassCreateInfo renderPassInfo = {};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.pNext = NULL;
    renderPassInfo.attachmentCount = 2;
    renderPassInfo.pAttachments = attachments;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpassDescription;
    renderPassInfo.dependencyCount = 2;
    renderPassInfo.pDependencies = subpassDependencies;

    VK_CHECK_RESULT(vkCreateRenderPass(device->logicalDevice, &renderPassInfo, nullptr, &renderPass));
}
void btVKDebugDrawer::prepareDescriptors()
{
    // Descriptor pool
    std::vector<VkDescriptorPoolSize> poolSizes = {
        vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1),
        //vks::initializers::descriptorPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1)
    };

    VkDescriptorPoolCreateInfo descriptorPoolInfo = vks::initializers::descriptorPoolCreateInfo(poolSizes, 2);
    VK_CHECK_RESULT(vkCreateDescriptorPool(device->logicalDevice, &descriptorPoolInfo, nullptr, &descriptorPool));

    // Descriptor set layout
    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
        vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT, 0),
        //vks::initializers::descriptorSetLayoutBinding(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 1),
    };

    VkDescriptorSetLayoutCreateInfo descriptorLayout = vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings);
    VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device->logicalDevice, &descriptorLayout, nullptr, &descriptorSetLayout));

    // Descriptor set
    /*VkDescriptorSetAllocateInfo allocInfo = vks::initializers::descriptorSetAllocateInfo(descriptorPool, &descriptorSetLayout, 1);
    VK_CHECK_RESULT(vkAllocateDescriptorSets(device->logicalDevice, &allocInfo, &descriptorSet));

    VK_CHECK_RESULT(vkAllocateDescriptorSets(device->logicalDevice, &allocInfo, &descriptorSets.matrices));
    std::vector<VkWriteDescriptorSet> writeDescriptorSets = {
        vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 0, &ubo.descriptor),
        //vks::initializers::writeDescriptorSet(descriptorSets.matrices, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2, &textures.irradianceCube.descriptor),
    };
    vkUpdateDescriptorSets(device->logicalDevice, static_cast<uint32_t>(writeDescriptorSets.size()), writeDescriptorSets.data(), 0, nullptr);*/
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
    VK_CHECK_RESULT(vkCreatePipelineLayout(device->logicalDevice, &pipelineLayoutCreateInfo, nullptr, &pipelineLayout));

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

    VK_CHECK_RESULT(vkCreateGraphicsPipelines(device->logicalDevice, VK_NULL_HANDLE, 1, &pipelineCreateInfo, nullptr, &pipeline));

    vkDestroyShaderModule(device->logicalDevice, shaderStages[0].module, nullptr);
    vkDestroyShaderModule(device->logicalDevice, shaderStages[1].module, nullptr);
}

void btVKDebugDrawer::buildCommandBuffer (){
    if (vertexCount == 0) {
        return;
    }
    VkCommandBufferBeginInfo cmdBufInfo = vks::initializers::commandBufferBeginInfo();

    VkRenderPassBeginInfo renderPassBeginInfo = vks::initializers::renderPassBeginInfo();
    renderPassBeginInfo.renderPass = renderPass;
    renderPassBeginInfo.renderArea.extent = swapChain.swapchainExtent;

    for (size_t i = 0; i < cmdBuffers.size(); ++i)
    {
        renderPassBeginInfo.framebuffer = frameBuffers[i];
        VK_CHECK_RESULT(vkBeginCommandBuffer(cmdBuffers[i], &cmdBufInfo));

        vkCmdBeginRenderPass(cmdBuffers[i], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport = vks::initializers::viewport((float)swapChain.swapchainExtent.width,	(float)swapChain.swapchainExtent.height, 0.0f, 1.0f);
        vkCmdSetViewport(cmdBuffers[i], 0, 1, &viewport);

        VkRect2D scissor = vks::initializers::rect2D(swapChain.swapchainExtent.width, swapChain.swapchainExtent.height, 0, 0);
        vkCmdSetScissor(cmdBuffers[i], 0, 1, &scissor);

        VkDeviceSize offsets[1] = { 0 };

        vkCmdBindPipeline (cmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
        vkCmdBindDescriptorSets(cmdBuffers[i], VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &dsUboMatrices, 0, NULL);
        vkCmdBindVertexBuffers (cmdBuffers[i], 0, 1, &vertexBuff.buffer, offsets);
        vkCmdDraw (cmdBuffers[i],  vertexCount, 1, 0, 0);

        vkCmdEndRenderPass(cmdBuffers[i]);

        VK_CHECK_RESULT(vkEndCommandBuffer(cmdBuffers[i]));
    }
}

void btVKDebugDrawer::submit (VkQueue queue, uint32_t bufferindex, VkSemaphore waitSemaphore) {
    submitInfo.pCommandBuffers = &cmdBuffers[bufferindex];
    submitInfo.commandBufferCount = 1;
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = &waitSemaphore;

    VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, fence));

    VK_CHECK_RESULT(vkWaitForFences(device->logicalDevice, 1, &fence, VK_TRUE, UINT64_MAX));
    VK_CHECK_RESULT(vkResetFences(device->logicalDevice, 1, &fence));
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
    shaderStage.module = vks::tools::loadShader(androidApp->activity->assetManager, fileName.c_str(), device->logicalDevice);
#else
    shaderStage.module = vks::tools::loadShader(fileName.c_str(), device->logicalDevice);
#endif
    shaderStage.pName = "main"; // todo : make param
    assert(shaderStage.module != VK_NULL_HANDLE);
    return shaderStage;
}

