#include <libcmaes/cmaes.h>
#include <iostream>
#include <vector>
#include <memory>
#include "Utils/ProcessUtils.h"
#include "Utils/OptimizationLogger.h"
#include "Common.h" // 包含 ParameterSpec 定义

int main() {
    // 设置主窗口标题
    SetConsoleTitleA("Optimizer - Main Control (DO NOT CLOSE)");

    system("chcp 65001>nul");

    // ================= 配置区域 =================
    const std::string WORKER_EXE = "SimWorker.exe";
    const int TIMEOUT_MS = 900000; // 15*60 = 900 15min超时
    std::string logPath = "D:/BayesOptDataSet/patient/cao feng ming/output/cmaes_log.csv";

    // 参数定义 (必须与 Worker 一致)
    std::vector<ParameterSpec> specs;
    specs.push_back({ "Aorta_E", "Aorta", "E", 0.1e6, 10e6 });
    specs.push_back({ "Valve_E", "Valve", "E", 0.02e6, 10e6 });
    specs.push_back({ "AorticAnnulus_E", "AorticAnnulus", "E", 0.1e6, 40e6 });
    specs.push_back({ "AortomitralCurtain_E", "AortomitralCurtain", "E", 0.02e6, 10e6 });
    specs.push_back({ "LeftVentricular_E", "LeftVentricular", "E", 0.1e6, 50e6 });
    int dim = (int)specs.size();
    // ===========================================

    // 日志
    auto logger = std::make_unique<OptimizationLogger>(logPath);
    std::vector<std::string> names;
    for (auto& s : specs) names.push_back(s.name);
    logger->writeHeader(names);

    int iterCount = 0;

    // 定义目标函数 (直接调子进程)
    libcmaes::FitFunc fitnessFunc = [&](const double* x, const int N) {
        iterCount++;
        std::vector<double> params(x, x + N);

        // 1. 边界钳制 [0, 1]
        for (auto& v : params) v = std::max(0.0, std::min(1.0, v));

        // 2. 【核心】调用子进程，超时杀进程
        double error = ProcessUtils::runWorker(WORKER_EXE, params, TIMEOUT_MS);

        // 3. 记录日志 (转换回物理参数用于记录)
        std::vector<double> realParams;
        for (int i = 0; i < dim; ++i) {
            realParams.push_back(specs[i].minVal + params[i] * (specs[i].maxVal - specs[i].minVal));
        }
        logger->logIteration(iterCount, realParams, error);

        std::cout << "Iter " << iterCount << " | Error: " << error
            << (error > 1e6 ? " [FAIL]" : "") << std::endl;
        return error;
    };

    // CMA-ES 启动参数
    std::vector<double> x0(dim, 0.5);
    double sigma = 0.2;
    std::vector<double> lb(dim, 0.0), ub(dim, 1.0);

    libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lb.data(), ub.data(), dim);
    libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>
        cmaparams(x0, sigma, -1, 0, gp);

    libcmaes::ESOptimizer<libcmaes::CMAStrategy<libcmaes::CovarianceUpdate,
        libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>,
        libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>>
        optim(fitnessFunc, cmaparams);

    // 开始跑
    std::cout << ">>> Optimizer Started. Spawning " << WORKER_EXE << "..." << std::endl;
	int generation = 0;
    while (!optim.stop()) {
        // A. 生成候选点
        dMat candidates = optim.ask();

        // B. 评估候选点（内部调用仿真并记录日志）
        optim.eval(candidates);

        // C. 更新分布并进入下一代
        optim.tell();
        optim.inc_iter(); // 必须调用，信号进入下一代

        generation++;
        std::cout << ">>> Generation " << generation << " Done. Best: "
            << optim.get_solutions().best_candidate().get_fvalue() << std::endl;
    }

    return 0;
}

