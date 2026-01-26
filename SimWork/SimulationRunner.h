// Core/SimulationRunner.h

#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include "MaterialMapper.h"
#include "solver/cuda_Simulation_Engine.h"
#include "Common.h" 

// 存储目标切面的测量数据（真实值）
struct TargetSliceData {
    double yHeight;
    double targetLongAxis;
    double targetShortAxis;
    double targetArea;
    double targetCircumference;
};

class SimulationRunner {
public:
    // 构造函数传入配置
    SimulationRunner(const SimulationConfig& config);
    ~SimulationRunner();

    void setMaterialMapper(std::shared_ptr<MaterialMapper> mapper);

    // 注册要优化的参数列表
    void setOptimizationSpecs(const std::vector<ParameterSpec>& specs);

    // 设置基于切片的目标数据
    void setSliceTargets(const std::vector<TargetSliceData>& targets);

    // 核心运行接口
    double run(const std::vector<double>& normalizedParams);

	std::vector<ParameterSpec> getParameterSpecs() const {
		return m_paramSpecs;
	}

private:
    SimulationConfig m_config;
    std::shared_ptr<MaterialMapper> m_mapper;
    std::vector<TargetSliceData> m_sliceTargets;
    std::vector<ParameterSpec> m_paramSpecs;

    // 内部辅助：加载支架模型
    void loadStentModel(std::vector<Simulation::Model*>& models);

    // 内部辅助：归一化还原
    double reNormalize(double val, double min, double max);


    /**
     * @brief 导出弹性模量分布到 Tecplot DAT 文件
     *
     * @param model 模型指针
     * @param filename 输出文件名 (e.g., "modulus.dat")
     * @param zoneName Zone的名称 (e.g., "Aorta_Modulus")
     */
    bool exportElasticModulusToTecplot(
        Simulation::TetModel* model,
        const std::string& filename,
        const std::string& zoneName = "ElasticModulus_Zone"
    );

};