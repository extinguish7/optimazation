#include <iostream>
#include <fstream>
#include <vector>
#include "Core/SimulationRunner.h"
#include "Core/MaterialMapper.h"
#include "Common.h"

int main(int argc, char* argv[]) {
    // 1. 设置控制台标题，方便区分
    SetConsoleTitleA("SimWorker - Processing Simulation...");

    // 防止中文乱码
    system("chcp 65001>nul");

    if (argc < 3) return -1;
    std::string inFile = argv[1];
    std::string outFile = argv[2];

    try {
        // 1. 读取参数
        std::vector<double> params;
        std::ifstream in(inFile);
        int size;
        if (in >> size) {
            double val;
            while (in >> val) params.push_back(val);
        }

        // 2. 配置环境 (直接复制你原项目的配置)
        SimulationConfig config;
        config.meshRoot = "D:/BayesOptDataSet/patient/cao feng ming/meshes/";
        config.outputRoot = "D:/BayesOptDataSet/patient/cao feng ming/output/";
        config.vesselInpPath = config.meshRoot + "CFM.inp";
        config.vesselExpandedPath = config.meshRoot + "CFM_expanded.inp";
        config.stentType = Simulation::StentType::VenusA_L26;
        config.useHausdorff = true;
        config.targetMeshPath = config.meshRoot + "CFM_stent_transformed.stl";

        // 3. 定义参数 Specs (用于 Runner 内部反归一化)
        std::vector<ParameterSpec> specs;
        specs.push_back({ "Aorta_E", "Aorta", "E", 0.1e6, 10e6 });
        specs.push_back({ "Valve_E", "Valve", "E", 0.1e6, 10e6 });
        specs.push_back({ "AorticAnnulus_E", "AorticAnnulus", "E", 0.1e6, 40e6 });
        specs.push_back({ "AortomitralCurtain_E", "AortomitralCurtain", "E", 0.1e6, 10e6 });
        specs.push_back({ "LeftVentricular_E", "LeftVentricular", "E", 0.1e6, 50e6 });

        // 4. 初始化 Runner
        auto mapper = std::make_shared<MaterialMapper>();
        mapper->addRegion("AorticAnnulus", config.meshRoot + "AorticAnnulus.stl", 10);
        mapper->addRegion("AortomitralCurtain", config.meshRoot + "AortomitralCurtain.stl", 5);
        mapper->addRegion("LeftVentricular", config.meshRoot + "LeftVentricular.stl", 1);
        mapper->initialize();

        SimulationRunner runner(config);
        runner.setMaterialMapper(mapper);
        runner.setOptimizationSpecs(specs);

        // 5. 运行 (这里是同步运行，如果卡死，父进程会杀掉我)
        double error = runner.run(params);

        // 6. 写结果
        std::ofstream out(outFile);
        out << error;

    }
    catch (...) {
        return 1; // 崩溃退出
    }
    return 0;
}