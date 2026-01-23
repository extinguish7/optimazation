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
    std::string vesselInpPath;        // 原始血管 INP
    std::string vesselExpandedPath;   // 扩张后血管 INP (如果需要)
    Simulation::StentType stentType;

    // 优化目标设置
    bool useHausdorff = false;        // 是否使用 Hausdorff 距离作为误差
    std::string targetMeshPath;       // 如果使用 Hausdorff，需要真实血管的 STL/OBJ 路径
};