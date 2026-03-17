// Core/MaterialMapper.cpp

#include "MaterialMapper.h"
#include "GeometryUtils.h"
#include <algorithm>
#include <iostream>
#include <omp.h>

MaterialMapper::MaterialMapper() {}
MaterialMapper::~MaterialMapper() {}

void MaterialMapper::addRegion(const std::string& name, const std::string& stlPath, int priority) {
    AnatomicalRegion region;
    region.name = name;
    region.stlPath = stlPath;
    region.priority = priority;
    m_regions.push_back(region);
}

void MaterialMapper::initialize() {
    for (auto& region : m_regions) {
        auto mesh = GeometryUtils::loadSTL(region.stlPath);
        region.distFunc = vtkSmartPointer<vtkImplicitPolyDataDistance>::New();
        region.distFunc->SetInput(mesh);
        std::cout << "[MaterialMapper] Initialized region: " << region.name << " from " << region.stlPath << std::endl;
    }
    // 优先级降序排列
    std::sort(m_regions.begin(), m_regions.end(), [](const auto& a, const auto& b) {
        return a.priority > b.priority;
        });
}

void MaterialMapper::applyMaterials(Simulation::Model* model, const std::map<std::string, double>& params, double searchRadius) {

    const auto& vertices = model->get_Vertice_0(); // 初始顶点
    const std::vector<std::vector<int>>& indices = model->get_Indices();    // 单元索引
    int numTets = (int)indices.size(); // 获取单元总数

    Simulation::TetModel* tetModel = static_cast<Simulation::TetModel*>(model);

    // ... (中间参数解析部分保持不变) ...

    std::vector<int> regionIndicesIndex;
    const std::map<std::string, std::vector<int>>& partIndices = tetModel->get_Part_Indices();

    // 将包含AORTA和LBB的indices复制到regionIndices
    for (const auto& part : partIndices) {
        if (part.first.find("AORTA") != std::string::npos || part.first.find("LBB") != std::string::npos) {
            regionIndicesIndex.insert(regionIndicesIndex.end(), part.second.begin(), part.second.end());
        }
    }

    int updatedCount = 0;

    // [调试建议] 暂时注释掉 OpenMP，防止多线程掩盖具体的越界错误
    // #pragma omp parallel for reduction(+:updatedCount)
    for (int i = 0; i < (int)regionIndicesIndex.size(); ++i) {
        int tetIdx = regionIndicesIndex[i];

        // [关键修复] 边界检查与修正
        if (tetIdx >= numTets) {
            // 尝试判断是否为 1-based 索引 (通常 INP 文件是 1-based)
            if (tetIdx - 1 < numTets && tetIdx - 1 >= 0) {
                tetIdx -= 1; // 自动修正为 0-based
            }
            else {
                // 依然越界，打印错误并跳过，防止崩溃
                static bool warned = false;
                if (!warned) {
                    std::cerr << "[Error] Tet Index Out of Bounds! Index: " << tetIdx
                        << " Max: " << numTets << std::endl;
                    warned = true;
                }
                continue;
            }
        }

        // 双重保险：如果修正后依然负数（异常数据）
        if (tetIdx < 0) continue;

        // 计算四面体质心
        Eigen::Vector3d centroid(0, 0, 0);

        // 这里的 indices[tetIdx] 也需要安全访问
        const auto& tetNodes = indices[tetIdx];
        for (int idx : tetNodes) {
            // 检查节点索引是否越界
            if (idx >= 0 && idx < vertices.size()) {
                centroid += vertices[idx];
            }
        }
        centroid /= 4.0;

        for (const auto& region : m_regions) {
            double dist = GeometryUtils::getDistanceToMesh(centroid, region.distFunc);

            // 如果在内部 (<=0) 或非常接近 (<= searchRadius)
            if (dist <= searchRadius) {
                std::string keyE = region.name + "_E";
                std::string keyNu = region.name + "_Nu";

                if (params.find(keyE) != params.end()) {
                    double E = params.at(keyE);
                    double Nu = params.count(keyNu) ? params.at(keyNu) : 0.4;

                    double lam, mu;
                    model->convert_Elastic_To_Lame(E, Nu, mu, lam);

                    // 使用修正后的 tetIdx 安全写入
                    model->set_Lambda(tetIdx, lam);
                    model->set_Mu(tetIdx, mu);

                    updatedCount++;
                    break;
                }
            }
        }
    }

    std::cout << "[MaterialMapper] Applied materials. Special regions affected " << updatedCount << " elements." << std::endl;
}