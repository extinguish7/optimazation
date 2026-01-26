// Utils/Logger.h
#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <mutex>

class OptimizationLogger {
public:
    OptimizationLogger(const std::string& filepath);
    ~OptimizationLogger();

    // 写入表头
    void writeHeader(const std::vector<std::string>& paramNames);

    // 写入一次迭代结果
    void logIteration(int iter, const std::vector<double>& params, double cost);

private:
    std::string m_filepath;
    std::mutex m_mutex;
};