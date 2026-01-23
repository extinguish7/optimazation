#include <iostream>
#include <memory>
#include <vector>
#include "Core/MaterialMapper.h"
#include "Core/SimulationRunner.h"
#include "Optimization/BayesOptimizer.h"
#include "Optimization/CmaesOptimizer.h"

int main(int argc, char** argv) {
    system("chcp 65001>nul");

    // ---------------------------------------------------------
    // 1. 配置仿真环境 (Configuration)
    // ---------------------------------------------------------
    SimulationConfig simConfig;
    simConfig.meshRoot = "D:/BayesOptDataSet/patient/cao feng ming/meshes/";
    simConfig.outputRoot = "D:/BayesOptDataSet/patient/cao feng ming/output/";
    simConfig.vesselInpPath = simConfig.meshRoot + "CFM.inp";
    simConfig.vesselExpandedPath = simConfig.meshRoot + "CFM_expanded.inp";

    // [设置] 支架型号暴露
    simConfig.stentType = Simulation::StentType::VenusA_L26;

    // [设置] 误差计算策略 (Hausdorff 示例)
     simConfig.useHausdorff = true;
     simConfig.targetMeshPath = simConfig.meshRoot + "CFM_stent_transformed.stl";

    // 或者使用切片策略
    //simConfig.useHausdorff = false;

     bool useBayes = false;
     bool useCmaes = true;

    // ---------------------------------------------------------
    // 2. 定义优化参数 (Flexible Parameters)
    // ---------------------------------------------------------
    std::vector<ParameterSpec> specs;

    // 灵活添加任意数量的参数：
    // { "显示名称", "区域Key", "类型", Min, Max }
	specs.push_back({ "Aorta_E",   "Aorta",  "E", 0.1e6, 10e6 });
	specs.push_back({ "Valve_E",  "Valve", "E", 0.1e6, 10e6 });
	specs.push_back({ "AorticAnnulus_E",   "AorticAnnulus",  "E", 0.1e6, 40e6 });
	specs.push_back({ "AortomitralCurtain_E",   "AortomitralCurtain",  "E", 0.1e6, 10e6 });
	specs.push_back({ "LeftVentricular_E",   "LeftVentricular",  "E", 0.1e6, 50e6 });
	// 如果想优化 Plaque 的泊松比，直接加一行即可：
    // specs.push_back({ "Plaque_Nu",  "Plaque",  "Nu", 0.3,   0.49 });

    size_t n_dims = specs.size();

    // ---------------------------------------------------------
    // 3. 初始化模块
    // ---------------------------------------------------------

    // (A) Material Mapper
    auto mapper = std::make_shared<MaterialMapper>();
    mapper->addRegion("AorticAnnulus", simConfig.meshRoot + "AorticAnnulus.stl", 10);
    mapper->addRegion("AortomitralCurtain", simConfig.meshRoot + "AortomitralCurtain.stl", 5);
    mapper->addRegion("LeftVentricular", simConfig.meshRoot + "LeftVentricular.stl", 1);
    mapper->initialize();

    // (B) Runner
    auto runner = std::make_shared<SimulationRunner>(simConfig);
    runner->setMaterialMapper(mapper);
    runner->setOptimizationSpecs(specs); // 注入参数定义

    if (!simConfig.useHausdorff) {
        std::vector<TargetSliceData> targets = {
            { -5.0,  16.97, 14.52, 775.1, 99.1 },
            { 20.5,  13.52, 11.55, 491.0, 78.9 },
            { 35.6,  19.23, 18.68, 1129.5, 119.1 }
        };
        runner->setSliceTargets(targets);
    }

    if (useBayes)
    {
        // (C) Optimizer
        bayesopt::Parameters boptParams = initialize_parameters_to_default();
        boptParams.n_iterations = 200;

        // 注意：构造函数现在接收 specs 用于打印 Log 表头
        StentOptimizer opt(n_dims, boptParams, runner, simConfig.outputRoot + "optimization_log.csv", specs);

        // 自动设置边界 [0, 1] (因为我们在 Runner 内部做 reNormalize)
        vectord lowerBound(n_dims), upperBound(n_dims);
        for (size_t i = 0; i < n_dims; ++i) {
            lowerBound[i] = 0.0;
            upperBound[i] = 1.0;
        }
        opt.setBoundingBox(lowerBound, upperBound);

        // ---------------------------------------------------------
        // 4. 运行
        // ---------------------------------------------------------
        std::cout << "Starting Optimization with " << n_dims << " parameters..." << std::endl;
        if (simConfig.useHausdorff) std::cout << "Objective: Hausdorff Distance" << std::endl;
        else std::cout << "Objective: Slice Fitting Error" << std::endl;

        vectord result(n_dims);
        try {
            opt.optimize(result);

            std::cout << "\nOptimal Parameters Found:\n";
            for (size_t i = 0; i < n_dims; ++i) {
                // 打印还原后的物理值
                double realVal = specs[i].minVal + result[i] * (specs[i].maxVal - specs[i].minVal);
                std::cout << "  " << specs[i].name << ": " << realVal << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Optimization Failed: " << e.what() << std::endl;
        }

    }

	if (useCmaes)
	{
        const int dim = static_cast<int>(specs.size());
        std::vector<double> x0(dim, 0.5); // 初始猜测点（归一化空间中心）
        double sigma = 0.2;               // 初始步长

        // 1. 解决报错 1: 使用 std::vector 避免 VLA 编译问题
        std::vector<double> lb(dim, 0.0);
        std::vector<double> ub(dim, 1.0);

        // 2. 解决报错 2: 修正变量命名，实例化评估器
        VascularEvaluator vascularOpt(runner, specs, simConfig.outputRoot + "cmaes_log.csv");

        // 3. 定义 FitFunc 类型的 Lambda 表达式
        libcmaes::FitFunc fitnessFunc = [&](const double* x, const int N) {
            return vascularOpt.evaluate(x, N);
        };

        // 4. 配置边界策略 GenoPheno
        // 使用 .data() 获取底层指针以适配库接口
        libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lb.data(), ub.data(), dim);

        // 5. 配置参数
        // -1: 自动决定 lambda, 0: 随机种子
        libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>
            cmaparams(x0, sigma, -1, 0, gp);

        // 6. 初始化优化器（采用 ask-tell 模式）
        // 明确定义 Strategy 类型，确保它包含边界策略
        typedef libcmaes::CMAStrategy<libcmaes::CovarianceUpdate,
            libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>> Strategy;

        libcmaes::ESOptimizer<Strategy,
            libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>>
            optim(fitnessFunc, cmaparams);

        // 7. 严谨的优化循环
        std::cout << "Starting CMA-ES Optimization..." << std::endl;
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
            std::cout << "Generation " << generation << " finished." << std::endl;
            std::cout << "solution: " << optim.get_solutions() << std::endl;
        }

        // 8. 获取并打印最优结果
        auto bestCandidate = optim.get_solutions().best_candidate();
        std::cout << "Best Error: " << bestCandidate.get_fvalue() << std::endl;
	}

    getchar();
    return 0;
}