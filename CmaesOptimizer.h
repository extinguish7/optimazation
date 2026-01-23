#pragma once
#include <libcmaes/cmaes.h>
#include "../Core/SimulationRunner.h"
#include "../Utils/OptimizationLogger.h"
#include <vector>
#include <algorithm>
#include <future>   // 新增：用于异步超时控制
#include <chrono>   // 新增：用于时间定义
#include <cmath>    // 新增：用于 isinf, isnan

// 适配器类，用于连接 libcmaes 和仿真运行器
class VascularEvaluator {
public:
    VascularEvaluator(std::shared_ptr<SimulationRunner> runner,
        const std::vector<ParameterSpec>& specs,
        const std::string& logPath)
        : m_runner(runner), m_specs(specs), m_iterCount(0) {

        m_logger = std::make_unique<OptimizationLogger>(logPath);
        std::vector<std::string> names;
        for (const auto& spec : m_specs) names.push_back(spec.name);
        m_logger->writeHeader(names);
    }

    // 严格符合 FitFunc 签名: double(const double*, const int)
    double evaluate(const double* x, const int N) {
        m_iterCount++;

        // 1. 将数据转为 vector 并进行边界保护
        std::vector<double> normParams(x, x + N);
        for (auto& val : normParams) {
            val = std::max(0.0, std::min(1.0, val));
        }

        // 2. 运行仿真获取误差
        double error = m_runner->run(normParams);

        // 3. 计算物理参数用于日志记录
        std::vector<double> realParams;
        for (size_t i = 0; i < m_specs.size(); ++i) {
            double val = m_specs[i].minVal + normParams[i] * (m_specs[i].maxVal - m_specs[i].minVal);
            realParams.push_back(val);
        }
        m_logger->logIteration(m_iterCount, realParams, error);

        return error;
    }

private:
    std::shared_ptr<SimulationRunner> m_runner;
    std::vector<ParameterSpec> m_specs;
    std::unique_ptr<OptimizationLogger> m_logger;
    int m_iterCount;
};