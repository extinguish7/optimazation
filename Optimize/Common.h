#pragma once
#include <string>
#include <vector>
#include <map>
#include "Common/Common.h"

// 单个优化参数的定义（解决需求 2：灵活参数）
struct ParameterSpec {
    std::string name;       // 例如 "Vessel_E", "Plaque_Nu"
    std::string regionName; // 对应 MaterialMapper 中的区域名
    std::string paramType;  // "E" (杨氏模量) 或 "Nu" (泊松比)
    double minVal;          // 真实物理量的下限
    double maxVal;          // 真实物理量的上限
};

// 仿真环境配置（解决需求 1：暴露设置）
struct SimulationConfig {
    std::string meshRoot;
    std::string outputRoot;

    // [新增] 支架库的根目录 (用于动态定位 stent 文件)
    std::string stentRoot;

    std::string vesselInpPath;
    std::string vesselExpandedPath;
    Simulation::StentType stentType;

    bool useHausdorff = false;
    std::string targetMeshPath;
};