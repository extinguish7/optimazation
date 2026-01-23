// Optimization/BayesOptimizer.cpp

#include "BayesOptimizer.h"
#include <iostream>

StentOptimizer::StentOptimizer(size_t dim, const bayesopt::Parameters& params,
    std::shared_ptr<SimulationRunner> runner,
    const std::string& logPath,
    const std::vector<ParameterSpec>& specs) // 传入参数定义
    : ContinuousModel(dim, params), m_runner(runner), m_currentIter(0)
{
    m_logger = std::make_unique<OptimizationLogger>(logPath);

    // 动态生成表头
    std::vector<std::string> names;
    for (const auto& spec : specs) {
        names.push_back(spec.name); // e.g. "Vessel_E", "Plaque_Nu"
    }
    m_logger->writeHeader(names);
}

double StentOptimizer::evaluateSample(const vectord& x) {
    m_currentIter++;

    // 修复 Error 3: 手动转换 bayesopt::vectord 到 std::vector<double>
    // vectord 通常支持 size() 和 operator[]
    std::vector<double> inputs;
    inputs.resize(x.size());
    for (size_t i = 0; i < x.size(); ++i) {
        inputs[i] = x[i];
    }

    // 运行仿真
    double error = m_runner->run(inputs);

	// renormalize inputs
	for (size_t i = 0; i < inputs.size(); ++i) {
		double minVal = m_runner->getParameterSpecs()[i].minVal;
		double maxVal = m_runner->getParameterSpecs()[i].maxVal;
		inputs[i] = minVal + inputs[i] * (maxVal - minVal);
	}

    // 记录日志
    m_logger->logIteration(m_currentIter, inputs, error);

    return error;
}