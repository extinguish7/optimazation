// =========================================================================
// 必须放在第一行，用于解决 Boost 与 C++17 的兼容性报错 (Error C4996)
// =========================================================================
#define _SILENCE_CXX17_OLD_ALLOCATOR_MEMBERS_DEPRECATION_WARNING

#include <libcmaes/cmaes.h>
#include <bayesopt/bayesopt.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <filesystem> 
#include <fstream> 
#include "Utils/ProcessUtils.h"
#include "Utils/OptimizationLogger.h"
#include "Common.h"

// 使用命名空间
using namespace libcmaes;
namespace fs = std::filesystem;

// =========================================================
// 全局运行模式配置
// =========================================================
enum class RunMode {
    ManualSingleRun,    // 模式1: 手动设置参数跑一次 (用于测试/验证)
    CmaesOptimization,   // 模式2: 自动 CMA-ES 优化 (用于寻找最优解)
	BayesOptOptimization  // 模式3: 自动贝叶斯优化
};

// 【在此处切换功能】
//const RunMode CURRENT_MODE = RunMode::BayesOptOptimization;
const RunMode CURRENT_MODE = RunMode::CmaesOptimization;
//const RunMode CURRENT_MODE = RunMode::ManualSingleRun;

// =========================================================
// 辅助函数
// =========================================================

// 读取 config.txt 中的支架型号
std::string readStentType(const std::string& configPath) {
    std::ifstream file(configPath);
    std::string type;
    if (file.is_open()) {
        std::getline(file, type);
        type.erase(0, type.find_first_not_of(" \t\n\r"));
        type.erase(type.find_last_not_of(" \t\n\r") + 1);
    }
    return type.empty() ? "VenusA_L26" : type;
}

// 路径修复 (用于 Windows 命令)
std::string fixPath(std::string p) {
    for (char& c : p) if (c == '/') c = '\\';
    return p;
}

// 保存最佳结果到 best_output 文件夹
void saveBestOutput(const std::string& outputDir) {
    std::cout << "  >>> [Saving] Copying current output to 'best_output'..." << std::endl;

    std::string srcDir = outputDir + "output";
    std::string dstDir = outputDir + "best_output";
    std::string regStlFile = outputDir + "registered_target.stl";

    std::string cmdDel = "rmdir /S /Q \"" + fixPath(dstDir) + "\" >nul 2>&1";
    system(cmdDel.c_str());

    std::string cmdCopyDir = "xcopy \"" + fixPath(srcDir) + "\" \"" + fixPath(dstDir) + "\" /E /I /Y /Q >nul 2>&1";
    system(cmdCopyDir.c_str());

    std::string cmdCopyFile = "copy \"" + fixPath(regStlFile) + "\" \"" + fixPath(dstDir) + "\" /Y >nul 2>&1";
    system(cmdCopyFile.c_str());
}

// =========================================================
// 贝叶斯优化适配器 (继承自 ContinuousModel)
// =========================================================
class BayesOptExecutor : public bayesopt::ContinuousModel {
public:
    BayesOptExecutor(
        size_t dim,
        const bayesopt::Parameters& params,
        const std::string& workerExe,
        const std::string& meshDir,
        const std::string& outputDir,
        const std::string& stentTypeStr,
        const std::vector<ParameterSpec>& specs,
        int timeoutMs
    )
        : bayesopt::ContinuousModel(dim, params),
        m_workerExe(workerExe), m_meshDir(meshDir), m_outputDir(outputDir),
        m_stentTypeStr(stentTypeStr), m_specs(specs), m_timeoutMs(timeoutMs),
        m_globalBestError(1e9), m_iterCount(0)
    {
        // 初始化日志
        std::string logPath = outputDir + "bayesopt_log.csv";
        m_logger = std::make_unique<OptimizationLogger>(logPath);
        std::vector<std::string> names;
        for (const auto& s : m_specs) names.push_back(s.name);
        m_logger->writeHeader(names);
    }

    // 核心函数：贝叶斯优化器调用此函数来评估样本
    double evaluateSample(const vectord& x) override {
        m_iterCount++;

        // 1. 转换参数格式 (vectord -> vector<double>)
        // BayesOpt 应该配置为在 [0,1] 范围内搜索
        std::vector<double> params(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            params[i] = std::max(0.0, std::min(1.0, x[i])); // 强制钳制
        }

        // 2. 调用子进程 SimWorker
        double error = ProcessUtils::runWorker(m_workerExe, m_meshDir, m_outputDir, m_stentTypeStr, params, m_timeoutMs);

        // 3. 记录日志 (计算物理值用于显示)
        std::vector<double> realParams;
        for (size_t i = 0; i < m_specs.size(); ++i) {
            realParams.push_back(m_specs[i].minVal + params[i] * (m_specs[i].maxVal - m_specs[i].minVal));
        }
        m_logger->logIteration(m_iterCount, realParams, error);

        std::cout << "[BayesOpt] Iter " << m_iterCount << " | Error: " << error << std::endl;

        // 4. 保存最佳结果 (与 CMA-ES 逻辑一致)
        if (error < m_globalBestError && error < 1e5) {
            m_globalBestError = error;
            std::cout << "  >>> [New Best] Found error: " << error << ". Saving best_output..." << std::endl;
            saveBestOutput(m_outputDir);
        }

        return error;
    }

private:
    std::string m_workerExe;
    std::string m_meshDir;
    std::string m_outputDir;
    std::string m_stentTypeStr;
    std::vector<ParameterSpec> m_specs;
    int m_timeoutMs;

    std::unique_ptr<OptimizationLogger> m_logger;
    double m_globalBestError;
    int m_iterCount;
};

// =========================================================
// 功能模块 1: 手动单次仿真
// =========================================================
void runManualSimulation(
    const std::string& patientName,
    const std::string& WORKER_EXE,
    const std::string& meshDir,
    const std::string& outputDir,
    const std::string& stentTypeStr,
    const std::vector<ParameterSpec>& specs,
    int TIMEOUT_MS
) {
    std::cout << "\n--------------------------------------------------" << std::endl;
    std::cout << "[Mode: Manual] Running Single Simulation for " << patientName << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    // 1. 设置手动物理参数 (在此处修改你要测试的值)
    std::vector<double> manualPhysicalParams = {
        //2.0e6,   // Aorta_E
        //0.5e5,   // Valve_E
        //2.0e6,   // AorticAnnulus_E
        //0.5e5,   // AortomitralCurtain_E
        //8.0e6    // LeftVentricular_E

        2.98221e+06,
        2.42607e+06, 
        100000, 
        20000, 
        197715
    };

    if (manualPhysicalParams.size() != specs.size()) {
        std::cerr << "Error: Manual parameters count mismatch!" << std::endl;
        return;
    }

    // 2. 打印参数确认
    std::cout << "Parameters:" << std::endl;
    std::vector<double> normParams;
    for (size_t i = 0; i < specs.size(); ++i) {
        std::cout << "  " << specs[i].name << ": " << manualPhysicalParams[i] << std::endl;
        // 归一化
        double norm = (manualPhysicalParams[i] - specs[i].minVal) / (specs[i].maxVal - specs[i].minVal);
        normParams.push_back(std::max(0.0, std::min(1.0, norm)));
    }

    // 3. 调用 Worker
    double error = ProcessUtils::runWorker(WORKER_EXE, meshDir, outputDir, stentTypeStr, normParams, TIMEOUT_MS);

    std::cout << ">>> [Manual Result] Error: " << error << std::endl;

    // (可选) 如果手动跑的结果你觉得很好，也可以强制保存
    // saveBestOutput(outputDir);
}

// =========================================================
// 功能模块 2: CMA-ES 优化
// =========================================================
void runCMAESOptimization(
    const std::string& patientName,
    const std::string& WORKER_EXE,
    const std::string& meshDir,
    const std::string& outputDir,
    const std::string& stentTypeStr,
    const std::vector<ParameterSpec>& specs,
    int TIMEOUT_MS,
    int MAX_GENERATIONS
) {
    std::cout << "\n--------------------------------------------------" << std::endl;
    std::cout << "[Mode: Optimization] Starting CMA-ES for " << patientName << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    int dim = (int)specs.size();

    // 1. 初始化日志
    std::string logPath = outputDir + "cmaes_log.csv";
    auto logger = std::make_unique<OptimizationLogger>(logPath);
    std::vector<std::string> names;
    for (auto& s : specs) names.push_back(s.name);
    logger->writeHeader(names);

    double globalBestError = 1e9;
    int iterCount = 0;

    // 2. 定义目标函数
    FitFunc fitnessFunc = [&](const double* x, const int N) {
        iterCount++;
        std::vector<double> params(x, x + N);
        // 边界钳制
        for (auto& v : params) v = std::max(0.0, std::min(1.0, v));

        // 调用子进程
        double error = ProcessUtils::runWorker(WORKER_EXE, meshDir, outputDir, stentTypeStr, params, TIMEOUT_MS);

        // 记录日志
        std::vector<double> realParams;
        for (int i = 0; i < dim; ++i) {
            realParams.push_back(specs[i].minVal + params[i] * (specs[i].maxVal - specs[i].minVal));
        }
        logger->logIteration(iterCount, realParams, error);

        std::cout << "[" << patientName << "] Iter " << iterCount << " | Error: " << error << std::endl;

        // 保存最佳结果
        if (error < globalBestError && error < 1e5) {
            globalBestError = error;
            std::cout << "  >>> [New Best] Found error: " << error << ". Saving best_output..." << std::endl;
            saveBestOutput(outputDir);
        }
        return error;
    };

    // 3. 配置 CMA-ES 初始点 (x0) - 使用你指定的物理参数作为起点
    std::vector<double> initialParamsPhysical = {
        2.0e6,   // Aorta_E
        0.5e5,   // Valve_E
        2.0e6,   // AorticAnnulus_E
        0.5e5,   // AortomitralCurtain_E
        8.0e6    // LeftVentricular_E
    };

    std::vector<double> x0(dim);
    for (int i = 0; i < dim; ++i) {
        double norm = (initialParamsPhysical[i] - specs[i].minVal) / (specs[i].maxVal - specs[i].minVal);
        x0[i] = std::max(0.0, std::min(1.0, norm));
    }

    double sigma = 0.2;
    std::vector<double> lb(dim, 0.0), ub(dim, 1.0);

    // 4. 启动优化器
    GenoPheno<pwqBoundStrategy> gp(lb.data(), ub.data(), dim);
    CMAParameters<GenoPheno<pwqBoundStrategy>> cmaparams(x0, sigma, -1, 0, gp);
    cmaparams.set_max_iter(MAX_GENERATIONS);

    ESOptimizer<CMAStrategy<CovarianceUpdate, GenoPheno<pwqBoundStrategy>>,
        CMAParameters<GenoPheno<pwqBoundStrategy>>>
        optim(fitnessFunc, cmaparams);

    int currentGen = 0;
    while (!optim.stop()) {
        dMat candidates = optim.ask();
        optim.eval(candidates);
        optim.tell();
        optim.inc_iter();

        currentGen++;
        std::cout << ">>> [" << patientName << "] Generation " << currentGen << "/" << MAX_GENERATIONS << " Done. Best: "
            << optim.get_solutions().best_candidate().get_fvalue() << std::endl;

        if (currentGen >= MAX_GENERATIONS) break;
    }
}

// =========================================================
// 功能模块 3: 贝叶斯优化 (Bayesian Optimization)
// =========================================================
void runBayesOptOptimization(
    const std::string& patientName,
    const std::string& WORKER_EXE,
    const std::string& meshDir,
    const std::string& outputDir,
    const std::string& stentTypeStr,
    const std::vector<ParameterSpec>& specs,
    int TIMEOUT_MS,
    int MAX_GENERATIONS
) {
    std::cout << "\n--------------------------------------------------" << std::endl;
    std::cout << "[Mode: BayesOpt] Starting Bayesian Optimization for " << patientName << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    int dim = (int)specs.size();

    // 1. 配置贝叶斯优化参数
    bayesopt::Parameters boptParams = initialize_parameters_to_default();

    // 关键设置
    boptParams.n_iterations = MAX_GENERATIONS; // 总迭代次数
    boptParams.surr_name = "sGaussianProcess"; // 代理模型：高斯过程

    // 2. 实例化执行器
    BayesOptExecutor opt(dim, boptParams, WORKER_EXE, meshDir, outputDir, stentTypeStr, specs, TIMEOUT_MS);

    // 3. 设置边界 [0, 1]
    // 因为 SimWorker 接收的是归一化参数，所以我们让 BayesOpt 也在 [0, 1] 空间内搜索
    vectord lowerBound(dim), upperBound(dim);
    for (int i = 0; i < dim; ++i) {
        lowerBound[i] = 0.0;
        upperBound[i] = 1.0;
    }
    opt.setBoundingBox(lowerBound, upperBound);

    // 4. 运行优化
    std::cout << ">>> BayesOpt Started. Max Iterations: " << MAX_GENERATIONS << std::endl;

    vectord bestParamsNormalized(dim);
    try {
        opt.optimize(bestParamsNormalized);
    }
    catch (std::exception& e) {
        std::cerr << "[BayesOpt Error] " << e.what() << std::endl;
        return;
    }

    // 5. 输出最终结果
    std::cout << "\n>>> Optimization Finished for " << patientName << std::endl;
    std::cout << "Best Parameters found (Physical):" << std::endl;
    for (int i = 0; i < dim; ++i) {
        double p = specs[i].minVal + bestParamsNormalized[i] * (specs[i].maxVal - specs[i].minVal);
        std::cout << "  " << specs[i].name << ": " << p << std::endl;
    }
}

// =========================================================
// 主函数
// =========================================================
int main() {
    SetConsoleTitleA("Optimizer - Batch Manager");
    system("chcp 65001>nul");

    const std::string DATASET_ROOT = "D:/BayesOptDataSet/patient/";
    const std::string WORKER_EXE = "SimWorker.exe";
    const int TIMEOUT_MS = 900000; // 15分钟超时
    const int MAX_GENERATIONS = 1000;

    if (!fs::exists(DATASET_ROOT)) {
        std::cerr << "Error: Dataset root not found: " << DATASET_ROOT << std::endl;
        return -1;
    }

    // 参数规格定义 (通用)
    std::vector<ParameterSpec> specs;
    specs.push_back({ "Aorta_E", "Aorta", "E", 0.1e6, 10e6 });
    specs.push_back({ "Valve_E", "Valve", "E", 0.1e6, 5e6 });
    specs.push_back({ "AorticAnnulus_E", "AorticAnnulus", "E", 0.1e6, 20e6 });
    specs.push_back({ "AortomitralCurtain_E", "AortomitralCurtain", "E", 0.1e6, 5e6 });
    specs.push_back({ "LeftVentricular_E", "LeftVentricular", "E", 0.5e6, 30e6 });

    // 遍历病人文件夹
    for (const auto& entry : fs::directory_iterator(DATASET_ROOT)) {
        if (!entry.is_directory()) continue;

        std::string patientName = entry.path().filename().string();
        std::string patientRoot = entry.path().string() + "/";

        std::string meshDir = patientRoot + "mesh/";
        if (!fs::exists(meshDir)) meshDir = patientRoot + "meshes/";

        std::string outputDir = patientRoot + "output/";

        if (!fs::exists(meshDir)) {
            std::cerr << "[Skip] No mesh folder found for: " << patientName << std::endl;
            continue;
        }

        // 读取支架配置
        std::string configPath = patientRoot + "config.txt";
        std::string stentTypeStr = fs::exists(configPath) ? readStentType(configPath) : "VenusA_L26";

        // 确保输出目录存在
        if (!fs::exists(outputDir)) fs::create_directory(outputDir);

        // ==========================================
        // 根据模式调用不同功能
        // ==========================================
        if (CURRENT_MODE == RunMode::ManualSingleRun) {
            runManualSimulation(patientName, WORKER_EXE, meshDir, outputDir, stentTypeStr, specs, TIMEOUT_MS);
        }
        else if (CURRENT_MODE == RunMode::CmaesOptimization) {
            runCMAESOptimization(patientName, WORKER_EXE, meshDir, outputDir, stentTypeStr, specs, TIMEOUT_MS, MAX_GENERATIONS);
        }
        else if (CURRENT_MODE == RunMode::BayesOptOptimization) {
            runBayesOptOptimization(patientName, WORKER_EXE, meshDir, outputDir, stentTypeStr, specs, TIMEOUT_MS, MAX_GENERATIONS);
        }
    }

    std::cout << "\nAll patients processed!" << std::endl;
    getchar();
    return 0;
}