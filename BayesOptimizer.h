// optimization/BayesOptimizer.h
#pragma once
#include <bayesopt/bayesopt.hpp>
#include "../Utils/OptimizationLogger.h"
#include "../Core/SimulationRunner.h"
#include <memory>
#include <vector>

class StentOptimizer : public bayesopt::ContinuousModel {
public:
    StentOptimizer(size_t dim, const bayesopt::Parameters& params,
        std::shared_ptr<SimulationRunner> runner,
        const std::string& logPath,
        const std::vector<ParameterSpec>& specs);

    // BayesOpt 接口
    double evaluateSample(const vectord& x) override;

private:
    std::shared_ptr<SimulationRunner> m_runner;
    std::unique_ptr<OptimizationLogger> m_logger;
    int m_currentIter;
};