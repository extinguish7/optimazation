// Core/MaterialMapper.cpp

#include "MaterialMapper.h"
#include "../Utils/GeometryUtils.h"
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

    Simulation::TetModel* tetModel = static_cast<Simulation::TetModel*>(model);

    for (const auto& param : params)
    {
        std::string keyE = "Aorta_E";
        std::string keyNu = "Aorta_Nu";

        if (params.find(keyE) != params.end())
        {
            double E = params.at(keyE);
            double Nu = params.count(keyNu) ? params.at(keyNu) : 0.4;

            double lam, mu;
            model->convert_Elastic_To_Lame(E, Nu, mu, lam);

            tetModel->set_Lambda("ES_AORTA", lam);
            tetModel->set_Mu("ES_AORTA", mu);
        }

        keyE = "Valve_E";
        keyNu = "Valve_Nu";
        if (params.find(keyE) != params.end())
        {
            double E = params.at(keyE);
            double Nu = params.count(keyNu) ? params.at(keyNu) : 0.4;

            double lam, mu;
            model->convert_Elastic_To_Lame(E, Nu, mu, lam);

            tetModel->set_Lambda("ES_AORTICVALVE", lam);
            tetModel->set_Mu("ES_AORTICVALVE", mu);
        }
    }


    std::vector<int> regionIndicesIndex;
    const std::map<std::string, std::vector<int>>& partIndices = tetModel->get_Part_Indices();
    // 将包含AORTA和LBB的indices复制到regionIndices
    for (const auto& part : partIndices) {
        if (part.first.find("AORTA") != std::string::npos || part.first.find("LBB") != std::string::npos) {
			regionIndicesIndex.insert(regionIndicesIndex.end(), part.second.begin(), part.second.end());
        }
    }

    int updatedCount = 0;

#pragma omp parallel for reduction(+:updatedCount)
    for (int i = 0; i < (int)regionIndicesIndex.size(); ++i) {
        // 计算四面体质心
        Eigen::Vector3d centroid(0, 0, 0);
        for (int idx : indices[regionIndicesIndex[i]]) {
            // 假设 vector<Vector3r>，Eigen 类型
            centroid += vertices[idx];
        }
        centroid /= 4.0;

        for (const auto& region : m_regions) {
            double dist = GeometryUtils::getDistanceToMesh(centroid, region.distFunc);

            // 如果在内部 (<=0) 或非常接近 (<= searchRadius)
            if (dist <= searchRadius) {
                std::string keyE = region.name + "_E";
                std::string keyNu = region.name + "_Nu";

                // 检查参数表中是否有该区域的参数
                if (params.find(keyE) != params.end()) {
                    double E = params.at(keyE);
                    double Nu = params.count(keyNu) ? params.at(keyNu) : 0.4;

                    double lam, mu;
                    model->convert_Elastic_To_Lame(E, Nu, mu, lam);

                    model->set_Lambda(regionIndicesIndex[i], lam);
                    model->set_Mu(regionIndicesIndex[i], mu);

                    updatedCount++;
                    break; // 优先级高的已匹配，跳过后续区域
                }
            }
        }
    }

    


    std::cout << "[MaterialMapper] Applied materials. Special regions affected " << updatedCount << " elements." << std::endl;
}