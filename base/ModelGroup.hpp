/*
* Vulkan Model loader using ASSIMP
*
* Copyright(C) 2016-2017 by Sascha Willems - www.saschawillems.de
*			   2017 JP Bruyère - jp_bruyère@hotmail.com
*
* This code is licensed under the MIT license(MIT) (http://opensource.org/licenses/MIT)
*/

#pragma once

#include <stdlib.h>
#include <string>
#include <fstream>
#include <vector>

#include "vulkan/vulkan.h"

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "VulkanDevice.hpp"
#include "VulkanBuffer.hpp"
#include "VulkanTexture.hpp"

#include <btBulletDynamicsCommon.h>

#if defined(__ANDROID__)
#include <android/asset_manager.h>
#endif

namespace vks
{
    struct Material {
        float color[4];         //if alpha > 0=> use color
        float metalness;
        float roughness;
        float specular;
        uint32_t texIdx;    //0 => no textue
    };

    class ModelGroup {
    public:
        struct ModelPart {
            uint32_t vertexBase;
            uint32_t vertexCount;
            uint32_t indexBase;
            uint32_t indexCount;
            uint32_t materialIdx;
        };
        struct DrawCommand {
            uint32_t modelIndex;
            uint32_t partIndex;
        };
        struct InstanceData {
            uint32_t materialIndex = 0;
            glm::mat4 modelMat = glm::mat4();
        };
        struct Model {
            std::vector<ModelPart> parts;
            struct Dimension
            {
                glm::vec3 min = glm::vec3(FLT_MAX);
                glm::vec3 max = glm::vec3(-FLT_MAX);
                glm::vec3 size;
            } dim;
        };

        std::vector<DrawCommand> instances;
        std::vector<InstanceData> instanceDatas;

        uint32_t addInstance(uint32_t modelIdx, uint32_t partIdx, std::vector<InstanceData> datas){
            uint32_t idx = instances.size();
            for (int i = 0; i < datas.size(); i++){
                instances.push_back({modelIdx,partIdx});
                instanceDatas.push_back(datas[i]);
            }
            return idx;
        }
        uint32_t addInstance(uint32_t modelIdx, uint32_t partIdx, InstanceData data){
            uint32_t idx = instances.size();
            instances.push_back({modelIdx,partIdx});
            instanceDatas.push_back(data);
            return idx;
        }
        uint32_t addInstance(uint32_t modelIdx, uint32_t partIdx,const glm::mat4& modelMat, uint32_t matIndex = 0){
            uint32_t idx = instances.size();
            instances.push_back({modelIdx,partIdx});
            InstanceData id;
            id.materialIndex = matIndex;
            id.modelMat = modelMat;
            instanceDatas.push_back(id);
            return idx;
        }
        uint32_t addInstance(uint32_t modelIdx, uint32_t matIndex = 0){
            uint32_t idx = instances.size();
            for (int i = 0; i < models[modelIdx].parts.size(); i++)
                addInstance(modelIdx, i, glm::mat4(), matIndex);
            return idx;

        }
        uint32_t addMaterial (uint32_t mapIndex, float metalness, float roughness, float specular) {

            Material mat = {{0.f,0.f,0.f,0.f}, metalness, roughness, specular, mapIndex};
            materials.push_back(mat);
            return materials.size()-1;
        }
        uint32_t addMaterial (float r, float g, float b, float a, float metalness, float roughness, float specular) {
            Material mat = {{r,g,b,a}, metalness, roughness, specular, 0};
            materials.push_back(mat);
            return materials.size()-1;
        }
        btConvexHullShape* getConvexHullShape (uint32_t modelIdx, uint32_t partIdx) {
            ModelPart* mod = &models[modelIdx].parts[partIdx];
            btConvexHullShape* shape  = new btConvexHullShape();

            for (int i = 0 ; i < mod->indexCount ; i++) {
                float* pV = &vertexBuffer [(mod->vertexBase + indexBuffer[mod->indexBase + i])*8];
                btVector3 v = btVector3(-pV[0], -pV[1], -pV[2]);
                shape->addPoint(v);
            }
            shape->optimizeConvexHull();
            shape->setMargin(0.001);
            return shape;
        }

        static const int defaultFlags = aiProcess_FlipWindingOrder |
                aiProcess_Triangulate | aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices;

        /*orig : static const int defaultFlags = aiProcess_FlipWindingOrder | aiProcess_PreTransformVertices |
                aiProcess_Triangulate | aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices;*/
        /*model2 : static const int defaultFlags =
                aiProcess_MakeLeftHanded | aiProcess_OptimizeMeshes | aiProcess_Triangulate |
                aiProcess_JoinIdenticalVertices| aiProcess_CalcTangentSpace | aiProcess_GenSmoothNormals;*/

        vks::VulkanDevice* device = nullptr;
        VkQueue copyQueue;

        vks::VertexLayout layout = vks::VertexLayout({
            vks::VERTEX_COMPONENT_POSITION,
            vks::VERTEX_COMPONENT_NORMAL,
            vks::VERTEX_COMPONENT_UV,
        });

        uint32_t indexCount = 0;
        uint32_t vertexCount = 0;

        std::vector<float> vertexBuffer;
        std::vector<uint32_t> indexBuffer;
        std::vector<aiMaterial*> aiMaterials;

        std::vector<Model> models;
        std::vector<Material> materials;

        vks::Buffer vertices;
        vks::Buffer indices;
        vks::Buffer instanceBuff;
        vks::Buffer materialsBuff;
        vks::Texture2DArray texArray;

        ModelGroup (vks::VulkanDevice* dev, VkQueue queue){
            device = dev;
            //layout = vertexLayout;
            copyQueue = queue;
        }
        ~ModelGroup () {
        }

        void destroy()
        {
            assert(device);
            if (materialsBuff.size > 0)
                materialsBuff.destroy();
            if (instanceBuff.size > 0)
                instanceBuff.destroy();
            //texArray.destroy();
            vertices.destroy();
            indices.destroy();
        }

        void prepare(std::vector<std::string> mapDic, uint32_t texSize = 1024)
        {
            //build vertices and indices buffers
            uint32_t vBufferSize = static_cast<uint32_t>(vertexBuffer.size()) * sizeof(float);
            uint32_t iBufferSize = static_cast<uint32_t>(indexBuffer.size()) * sizeof(uint32_t);

            // Use staging buffer to move vertex and index buffer to device local memory
            // Create staging buffers
            vks::Buffer vertexStaging, indexStaging;

            // Vertex buffer
            VK_CHECK_RESULT(device->createBuffer(
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                &vertexStaging,
                vBufferSize,
                vertexBuffer.data()));

            // Index buffer
            VK_CHECK_RESULT(device->createBuffer(
                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                &indexStaging,
                iBufferSize,
                indexBuffer.data()));

            // Create device local target buffers
            // Vertex buffer
            VK_CHECK_RESULT(device->createBuffer(
                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                &vertices,
                vBufferSize));

            // Index buffer
            VK_CHECK_RESULT(device->createBuffer(
                VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                &indices,
                iBufferSize));

            // Copy from staging buffers
            VkCommandBuffer copyCmd = device->createCommandBuffer(VK_COMMAND_BUFFER_LEVEL_PRIMARY, true);

            VkBufferCopy copyRegion{};

            copyRegion.size = vertices.size;
            vkCmdCopyBuffer(copyCmd, vertexStaging.buffer, vertices.buffer, 1, &copyRegion);

            copyRegion.size = indices.size;
            vkCmdCopyBuffer(copyCmd, indexStaging.buffer, indices.buffer, 1, &copyRegion);

            device->flushCommandBuffer(copyCmd, copyQueue);

            // Destroy staging resources
            vkDestroyBuffer(device->logicalDevice, vertexStaging.buffer, nullptr);
            vkFreeMemory(device->logicalDevice, vertexStaging.memory, nullptr);
            vkDestroyBuffer(device->logicalDevice, indexStaging.buffer, nullptr);
            vkFreeMemory(device->logicalDevice, indexStaging.memory, nullptr);


            if (mapDic.size()==0)
                return;

            texArray.buildFromImages(mapDic, texSize, VK_FORMAT_R8G8B8A8_UNORM, device, copyQueue);

            buildInstanceBuffer();
            buildMaterialBuffer();
        }

        void buildCommandBuffer(VkCommandBuffer cmdBuff){
            uint32_t modIdx = instances[0].modelIndex;
            uint32_t partIdx = instances[0].partIndex;
            uint32_t instCount = 0;
            uint32_t instOffset = 0;

            for (int i = 0; i < instances.size(); i++){
                if (modIdx != instances[i].modelIndex || partIdx != instances[i].partIndex) {
                    vkCmdDrawIndexed(cmdBuff,	models[modIdx].parts[partIdx].indexCount, instCount,
                                                models[modIdx].parts[partIdx].indexBase,
                                                models[modIdx].parts[partIdx].vertexBase, instOffset);

                    modIdx = instances[i].modelIndex;
                    partIdx = instances[i].partIndex;
                    instCount = 0;
                    instOffset = i;
                }
                instCount++;
            }
            if (instCount==0)
                return;
            vkCmdDrawIndexed(cmdBuff,	models[modIdx].parts[partIdx].indexCount, instCount,
                                        models[modIdx].parts[partIdx].indexBase,
                                        models[modIdx].parts[partIdx].vertexBase, instOffset);
        }

        void buildMaterialBuffer () {
            VK_CHECK_RESULT(device->createBuffer(
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                &materialsBuff,
                sizeof(vks::Material)*256));

            VK_CHECK_RESULT(materialsBuff.map());
            updateMaterialBuffer();
        }
        void buildInstanceBuffer (){
            if (instanceBuff.size > 0){
                vkDeviceWaitIdle(device->logicalDevice);
                instanceBuff.destroy();
            }

            instanceBuff.size = instanceDatas.size() * sizeof(InstanceData);

            VK_CHECK_RESULT(device->createBuffer(
                VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                &instanceBuff,
                instanceBuff.size));

            VK_CHECK_RESULT(instanceBuff.map());
            updateInstancesBuffer();
        }

        void updateInstancesBuffer(){
            memcpy(instanceBuff.mapped, instanceDatas.data(), instanceDatas.size() * sizeof(InstanceData));
        }
        void updateMaterialBuffer(){
            memcpy(materialsBuff.mapped, materials.data(), sizeof(vks::Material)*materials.size());
        }

        int addModel(const std::string& filename, float scaleFactor = 1.f, const int flags = defaultFlags)
        {
            Assimp::Importer Importer;
            const aiScene* pScene;

            // Load file
#if defined(__ANDROID__)
            // Meshes are stored inside the apk on Android (compressed)
            // So they need to be loaded via the asset manager

            AAsset* asset = AAssetManager_open(androidApp->activity->assetManager, filename.c_str(), AASSET_MODE_STREAMING);
            if (!asset) {
                LOGE("Could not load mesh from \"%s\"!", filename.c_str());
                return false;
            }
            assert(asset);
            size_t size = AAsset_getLength(asset);

            assert(size > 0);

            void *meshData = malloc(size);
            AAsset_read(asset, meshData, size);
            AAsset_close(asset);

            pScene = Importer.ReadFileFromMemory(meshData, size, flags);

            free(meshData);
#else
            pScene = Importer.ReadFile(filename.c_str(), flags);
#endif

            if (pScene)
            {
                glm::vec3 scale(scaleFactor);
                glm::vec2 uvscale(1.0f);
                glm::vec3 center(0.0f);

                uint32_t modelIndex = models.size();

                Model model;
                model.parts.resize(pScene->mNumMeshes);

                // Load meshes
                for (unsigned int i = 0; i < pScene->mNumMeshes; i++)
                {
                    const aiMesh* paiMesh = pScene->mMeshes[i];

                    model.parts[i] = {};
                    model.parts[i].vertexBase = vertexCount;
                    model.parts[i].indexBase = indexCount;
                    model.parts[i].materialIdx = 0;

                    vertexCount += paiMesh->mNumVertices;

                    const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);

                    for (unsigned int j = 0; j < paiMesh->mNumVertices; j++)
                    {
                        const aiVector3D* pPos = &(paiMesh->mVertices[j]);
                        const aiVector3D* pNormal = &(paiMesh->mNormals[j]);
                        const aiVector3D* pTexCoord = (paiMesh->HasTextureCoords(0)) ? &(paiMesh->mTextureCoords[0][j]) : &Zero3D;
                        const aiVector3D* pTangent = (paiMesh->HasTangentsAndBitangents()) ? &(paiMesh->mTangents[j]) : &Zero3D;
                        const aiVector3D* pBiTangent = (paiMesh->HasTangentsAndBitangents()) ? &(paiMesh->mBitangents[j]) : &Zero3D;
                        aiColor3D pColor(0.f, 0.f, 0.f);
                        for (auto& component : layout.components)
                        {
                            switch (component) {
                            case VERTEX_COMPONENT_POSITION:
                                vertexBuffer.push_back(pPos->x * scale.x + center.x);
                                vertexBuffer.push_back(-pPos->y * scale.y + center.y);
                                vertexBuffer.push_back(pPos->z * scale.z + center.z);
                                break;
                            case VERTEX_COMPONENT_NORMAL:
                                vertexBuffer.push_back(pNormal->x);
                                vertexBuffer.push_back(-pNormal->y);
                                vertexBuffer.push_back(pNormal->z);
                                break;
                            case VERTEX_COMPONENT_UV:
                                vertexBuffer.push_back(pTexCoord->x * uvscale.s);
                                vertexBuffer.push_back(pTexCoord->y * uvscale.t);
                                break;
                            case VERTEX_COMPONENT_COLOR:
                                vertexBuffer.push_back(pColor.r);
                                vertexBuffer.push_back(pColor.g);
                                vertexBuffer.push_back(pColor.b);
                                break;
                            case VERTEX_COMPONENT_TANGENT:
                                vertexBuffer.push_back(pTangent->x);
                                vertexBuffer.push_back(pTangent->y);
                                vertexBuffer.push_back(pTangent->z);
                                break;
                            case VERTEX_COMPONENT_BITANGENT:
                                vertexBuffer.push_back(pBiTangent->x);
                                vertexBuffer.push_back(pBiTangent->y);
                                vertexBuffer.push_back(pBiTangent->z);
                                break;
                            // Dummy components for padding
                            case VERTEX_COMPONENT_DUMMY_FLOAT:
                                vertexBuffer.push_back(0.0f);
                                break;
                            case VERTEX_COMPONENT_DUMMY_VEC4:
                                vertexBuffer.push_back(0.0f);
                                vertexBuffer.push_back(0.0f);
                                vertexBuffer.push_back(0.0f);
                                vertexBuffer.push_back(0.0f);
                                break;
                            };
                        }

                        model.dim.max.x = fmax(pPos->x, model.dim.max.x);
                        model.dim.max.y = fmax(pPos->y, model.dim.max.y);
                        model.dim.max.z = fmax(pPos->z, model.dim.max.z);

                        model.dim.min.x = fmin(pPos->x, model.dim.min.x);
                        model.dim.min.y = fmin(pPos->y, model.dim.min.y);
                        model.dim.min.z = fmin(pPos->z, model.dim.min.z);
                    }

                    model.dim.size = model.dim.max - model.dim.min;

                    model.parts[i].vertexCount = paiMesh->mNumVertices;

                    for (unsigned int j = 0; j < paiMesh->mNumFaces; j++)
                    {
                        const aiFace& Face = paiMesh->mFaces[j];
                        if (Face.mNumIndices != 3)
                            continue;
                        indexBuffer.push_back(Face.mIndices[0]);
                        indexBuffer.push_back(Face.mIndices[1]);
                        indexBuffer.push_back(Face.mIndices[2]);
                        model.parts[i].indexCount += 3;
                        indexCount += 3;
                    }
                }
                models.push_back(model);
                return modelIndex;
            }else{
                printf("Error parsing '%s': '%s'\n", filename.c_str(), Importer.GetErrorString());
#if defined(__ANDROID__)
                LOGE("Error parsing '%s': '%s'", filename.c_str(), Importer.GetErrorString());
#endif
                return -1;
            }
        }
    };
}
