#include "OptimizationLogger.h"
#include <iostream>
#include <iomanip>
// Utils/OptimizationLogger.cpp

OptimizationLogger::OptimizationLogger(const std::string& filepath) : m_filepath(filepath) {}

OptimizationLogger::~OptimizationLogger() {}

void OptimizationLogger::writeHeader(const std::vector<std::string>& paramNames) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::ifstream check(m_filepath);
    // 如果文件不存在或为空，写入表头
    if (check.peek() == std::ifstream::traits_type::eof() || !check.good()) {
        std::ofstream file(m_filepath, std::ios::app);
        file << "Iteration,";
        for (const auto& name : paramNames) file << name << ",";
        file << "Cost\n";
    }
}

void OptimizationLogger::logIteration(int iter, const std::vector<double>& params, double cost) {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::ofstream file(m_filepath, std::ios::app);
    file << iter << ",";
    for (double p : params) file << std::fixed << std::setprecision(6) << p << ",";
    file << cost << "\n";

    // 控制台输出
    std::cout << "[Logger] Iter: " << iter << " | Cost: " << cost << " | Params: [";
    for (size_t i = 0; i < params.size(); ++i) std::cout << params[i] << (i == params.size() - 1 ? "" : ", ");
    std::cout << "]" << std::endl;
}