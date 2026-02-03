#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <windows.h> 
#include "Core/SimulationRunner.h"
#include "Core/MaterialMapper.h"
#include "Common.h"
#include "../Optimize/Utils/PathUtils.h"

// 辅助：字符串转枚举
Simulation::StentType parseStentType(const std::string& typeStr) {
    if (typeStr == "VenusA_L32") return Simulation::StentType::VenusA_L32;
    if (typeStr == "VenusA_L26") return Simulation::StentType::VenusA_L26;
    if (typeStr == "VenusA_L29") return Simulation::StentType::VenusA_L29;
    if (typeStr == "VenusA_L23") return Simulation::StentType::VenusA_L23;
    // 默认值，防止报错
    std::cerr << "[Warning] Unknown stent type: " << typeStr << ", using default L26." << std::endl;
    return Simulation::StentType::VenusA_L26;
}

int main(int argc, char* argv[]) {
    SetConsoleTitleA("SimWorker - Initializing...");
    system("chcp 65001>nul");

    if (argc < 3) return -1;
    std::string inFile = argv[1];
    std::string outFile = argv[2];

    try {
        std::ifstream in(inFile);
        if (!in.is_open()) return -2;

        // [修改] 1. 读取环境配置
        std::string meshRoot, outputRoot, stentTypeStr;
        std::getline(in, meshRoot);
        std::getline(in, outputRoot);
        std::getline(in, stentTypeStr); // 读取支架型号

        // 2. 读取优化参数
        int size;
        std::vector<double> params;
        if (in >> size) {
            double val;
            while (in >> val) params.push_back(val);
        }

        // 3. 配置 Config
        SimulationConfig config;
        config.meshRoot = meshRoot;
        config.outputRoot = outputRoot;

        // [修改] 文件名统一化
        config.vesselInpPath = config.meshRoot + "aorta.inp";
        config.vesselExpandedPath = config.meshRoot + "aorta_expanded.inp";
        // 假设目标STL文件名也是统一的，或者根据实际情况修改
        config.targetMeshPath = config.meshRoot + "target_stent.stl";

        // [新增] 动态计算支架目录：Exe目录 + data/stent/
        std::string exeDir = PathUtils::getExeDir();
        config.stentRoot = exeDir + "data/stent/";

        // [修改] 动态设置支架类型
        config.stentType = parseStentType(stentTypeStr);
        config.useHausdorff = true;

        std::string title = "SimWorker - " + stentTypeStr + " - " + meshRoot;
        SetConsoleTitleA(title.c_str());

        // 4. 初始化 Runner (保持不变)
        std::vector<ParameterSpec> specs;
        specs.push_back({ "Aorta_E", "Aorta", "E", 0.1e6, 10e6 });
        specs.push_back({ "Valve_E", "Valve", "E", 0.1e6, 5e6 });
        specs.push_back({ "AorticAnnulus_E", "AorticAnnulus", "E", 0.1e6, 20e6 });
        specs.push_back({ "AortomitralCurtain_E", "AortomitralCurtain", "E", 0.1e6, 5e6 });
        specs.push_back({ "LeftVentricular_E", "LeftVentricular", "E", 0.5e6, 30e6 });

        auto mapper = std::make_shared<MaterialMapper>();
        // 确保这些STL文件在 meshRoot 下存在，如果命名统一则无需修改
        mapper->addRegion("AorticAnnulus", config.meshRoot + "AorticAnnulus.stl", 10);
        mapper->addRegion("AortomitralCurtain", config.meshRoot + "AortomitralCurtain.stl", 5);
        mapper->addRegion("LeftVentricular", config.meshRoot + "LeftVentricular.stl", 1);
        mapper->initialize();

        SimulationRunner runner(config);
        runner.setMaterialMapper(mapper);
        runner.setOptimizationSpecs(specs);

        double error = runner.run(params);

        std::ofstream out(outFile);
        out << error;

    }
    catch (...) {
        return 1;
    }
    return 0;
}