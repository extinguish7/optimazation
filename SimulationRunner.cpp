// Core/SimulationRunner.cpp

#include "SimulationRunner.h"
#include "../Utils/GeometryUtils.h"
#include "solver/TetModel.h"
#include "Utils/IglUtils.h" // 假设你有这个用于导出的工具
#include <sstream>
#include <iomanip>
#include <iostream>
#include <future>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory> // for std::shared_ptr


using namespace Simulation;

SimulationRunner::SimulationRunner(const SimulationConfig& config)
	: m_config(config) {}

SimulationRunner::~SimulationRunner() {}

void SimulationRunner::setOptimizationSpecs(const std::vector<ParameterSpec>& specs) {
	m_paramSpecs = specs;
}

void SimulationRunner::setMaterialMapper(std::shared_ptr<MaterialMapper> mapper) {
    m_mapper = mapper;
}

void SimulationRunner::setSliceTargets(const std::vector<TargetSliceData>& targets) {
	m_sliceTargets = targets;
}

double SimulationRunner::reNormalize(double val, double min, double max) {
    return val * (max - min) + min;
}

void SimulationRunner::loadStentModel(std::vector<Simulation::Model*>& models) {
    // 使用 m_config.stentType 来判断
    // 这里的参数可以做成 Config 的一部分，或者保持硬编码如果它们是不变量
    Real density = 6450e-6;
    Real youngs_module_A = 78780e6; Real poisson_ratio_A = 0.33;
    Real youngs_module_M = 27140e6; Real poisson_ratio_M = 0.33;
    Real varepsilon_L = 0.045;
    Real T = 37;
    Real C_AS = 6.5;
    Real C_SA = 6.5;
    Real T_AS = 37;
    Real T_SA = 37;
    Real sigma_AS_start = 520e6;
    Real sigma_AS_finish = 635e6;
    Real sigma_SA_start = 180e6;
    Real sigma_SA_finish = 26e6;
    Real sigma_AS_compress = 520e6;

	std::string nodePath, compressedNodePath, elePath, historyPath;
    
    // 根据 Config 拼接路径
    std::string stentPath = "D:/BayesOptDataSet/stent/";

    switch (m_config.stentType) {
    case StentType::VenusA_L32:
        nodePath = stentPath + "VenusA32_Y.node";
		compressedNodePath = stentPath + "VenusA32_compressed_D8mm_Y.node";
		elePath = stentPath + "VenusA32_Y.ele";
		historyPath = stentPath + "2.0000_stent_NULL_L32.dat";
        break;
	case StentType::VenusA_L26:
		nodePath = stentPath + "VenusA26_Y.node";
		compressedNodePath = stentPath + "VenusA26_compressed_D8mm_Y.node";
		elePath = stentPath + "VenusA26_Y.ele";
		historyPath = stentPath + "2.0000_stent_NULL_L26.dat";
		break;
	case StentType::VenusA_L29:
		nodePath = stentPath + "VenusA29_Y.node";
		compressedNodePath = stentPath + "VenusA29_compressed_D8mm_Y.node";
		elePath = stentPath + "VenusA29_Y.ele";
		historyPath = stentPath + "2.0000_stent_NULL_L29.dat";
		break;
	case StentType::VenusA_L23:
		nodePath = stentPath + "VenusA23_Y.node";
		compressedNodePath = stentPath + "VenusA23_compressed_D8mm_Y.node";
		elePath = stentPath + "VenusA23_Y.ele";
		historyPath = stentPath + "2.0000_stent_NULL_L23.dat";
		break;
    default:
		std::cerr << "Unsupported Stent Type!" << std::endl;
		return;
    }

	models.push_back(new TetModel(nodePath,
		compressedNodePath,
		elePath, "stent",
		density, youngs_module_A, poisson_ratio_A, youngs_module_M, poisson_ratio_M, varepsilon_L, T, C_AS, C_SA,
		sigma_AS_start, sigma_SA_start, sigma_AS_finish, sigma_SA_finish, sigma_AS_compress, T_AS, T_SA,
		MaterialType::Superelastic, historyPath));

    models.back()->set_Friction(0.2);
    models.back()->set_Damping(0.97);

}


double SimulationRunner::run(const std::vector<double>& normalizedParams) {
    // 1. 动态解析参数 (不再硬编码索引)
    std::map<std::string, double> paramMap;

    // 确保输入维度匹配
    if (normalizedParams.size() != m_paramSpecs.size()) {
        std::cerr << "Error: Parameter dimension mismatch!" << std::endl;
        return 1e9;
    }

    for (size_t i = 0; i < m_paramSpecs.size(); ++i) {
        const auto& spec = m_paramSpecs[i];
        // 还原归一化数值
        double realVal = reNormalize(normalizedParams[i], spec.minVal, spec.maxVal);

        // 构建 MaterialMapper 需要的键名，例如 "Plaque_E"
        std::string key = spec.regionName + "_" + spec.paramType;
        paramMap[key] = realVal;
    }

    // 处理默认参数 (如果没有显式优化 Default，可以绑定到 Vessel)
    if (paramMap.count("Vessel_E")) paramMap["Default_E"] = paramMap["Vessel_E"];
    if (paramMap.count("Vessel_Nu")) paramMap["Default_Nu"] = paramMap["Vessel_Nu"];


    // 2. 构建模型 (Models)
    std::vector<Simulation::Model*> models;

    // (A) 加载支架 - 抽离到辅助函数，使代码整洁
    loadStentModel(models);

    // (B) 加载血管 - 使用 Config 中的路径
    // 注意：这里需要根据你的 TetModel 构造函数适配
    models.push_back(new Simulation::TetModel(m_config.vesselInpPath, m_config.vesselExpandedPath, "vessel"));

    // 设置边界条件
    std::vector<int> pt_ids_0;
    Real miny = 1e60;
    Real maxy = -1e60;
    for (int i = 0; i < models[0]->get_Vertice().size(); i++)
    {
        Vector3r pt = models[0]->get_Vertice()[i];
        miny = std::min(miny, pt[1]);
        maxy = std::max(maxy, pt[1]);
    }
    Real party = miny * 0.99 + maxy * 0.01;
    printf("miny %f maxy %f party:%f\n", miny, maxy, party);
    for (int i = 0; i < models[0]->get_Vertice().size(); i++)
    {
        Vector3r pt = models[0]->get_Vertice()[i];
        if (pt[1] < party)
            pt_ids_0.push_back(i);
    }

    std::vector<int> aorta_boundary;
    auto& aorta_nsets = static_cast<TetModel*>(models.back())->get_Inp_Loader().get_NodeSets();
    for (size_t i = 0; i < aorta_nsets.size(); i++)
    {
        auto& nset = aorta_nsets[i];
        if (nset.name.find("BOUNDARY") != std::string::npos)
        {
            aorta_boundary.insert(aorta_boundary.end(), nset.nodes.begin(), nset.nodes.end());
        }
    }

    // 3. 应用材料参数
    if (m_mapper) {
        m_mapper->applyMaterials(models.back(), paramMap, 2.5);
    }

	exportElasticModulusToTecplot( static_cast<Simulation::TetModel*>(models.back()), m_config.outputRoot + "elastic_modulus.dat",	"Vessel_ElasticModulus");

    // 4. 初始化引擎 & 5. 运行循环 (保留原逻辑)
    // 4. 初始化引擎
    cuda_Engine* engine = new cuda_Engine();
    engine->init(models, 0.1, 1.0, 20, 1.0, m_config.meshRoot, true);
    engine->set_Collision_Coefficient(30);
    // 设置约束
    engine->add_Displacement_Constraint(0, std::make_pair<Real, Real>(0, 100), pt_ids_0, 1, 0);
    engine->add_Sheathing_Constraint(0, { 0,100 }, Vector3r(0, 1, 0), 10, 1);
    engine->add_Collision_Objects(0, 1, std::make_pair<Real, Real>(0, 100));
    engine->add_Collision_Objects(1, 1, std::make_pair<Real, Real>(0, 100));
    engine->add_Displacement_Constraint(1, std::make_pair<Real, Real>(0, 100), aorta_boundary, Vector3r(0, 0, 0));
    // 设置输出
    engine->set_Output_Open(0, true);
    engine->set_Output_Open(1, true);
    engine->set_Output_Stride(20);
    engine->set_Output_Path(m_config.outputRoot);

	// 5. 运行仿真循环
    double stopTime = 14.0;
    double currentTime = 0.0;
    double TIMESTEP = engine->get_time_step();
    int nTimeStep = 0;

    bool pause = false;
    do
    {
        nTimeStep++;
        currentTime += TIMESTEP;


        if (engine->solve(models) != SimulationStatus::Success)
        {
            pause = true;
            std::cout << "Simulation Pause!" << std::endl;

            delete engine;
            for (auto m : models) delete m;
			return 1e6;
        }

        for (size_t i = 0; i < models.size(); i++)
        {
            const MatrixXr_RowMajor engine_verts = engine->get_Vertex(i);
            std::vector<Vector3r> model_verts = models[i]->get_Vertice();
            auto& model = models[i];
            for (size_t j = 0; j < model->get_Vertices_Number(); j++)
            {
                model_verts[j] = engine_verts.row(j);
            }

            model->set_Vertice(model_verts);
        }

        if (currentTime > stopTime)
            break;

    } while (!pause);


    // 假设输出结果路径为 resultObjPath
    std::string resultObjPath = m_config.outputRoot + "output/Obj/14.0000_stent.obj"; // 简化示例

    // 6. 计算误差 (根据 Config 决定策略)
    double totalError = 0.0;

    if (m_config.useHausdorff) {
        // [新增] Hausdorff 模式
        if (m_config.targetMeshPath.empty()) {
            std::cerr << "Hausdorff mode enabled but no target mesh provided!" << std::endl;
            return 1e9;
        }
        // 调用新的配准与计算函数
		std::string savedRegisteredPath = m_config.outputRoot + "registered_target.stl";
        auto metrics = GeometryUtils::computeRegistrationAndError(resultObjPath, m_config.targetMeshPath, savedRegisteredPath);

        // 策略选择：
        // 1. 如果你希望尽量减少整体形状差异，用 RMSE (推荐)
        // 2. 如果你希望尽量贴合表面，用 Mean
        // 3. 也可以加权： 0.7 * Mean + 0.3 * Max (既看整体也不放过极端错误)
        totalError = 0.8 * metrics.rmse + 0.2 * metrics.maxDistance;

        // 打印调试信息，方便看 Log 知道配准效果
        std::cout << "  [Eval] Max: " << metrics.maxDistance
            << ", Mean: " << metrics.meanDistance
            << ", RMSE: " << metrics.rmse << std::endl;
    }
    else {
        // [原有] 切片椭圆拟合模式
        for (const auto& target : m_sliceTargets) {
            GeometryUtils::EllipseData simData;
            bool ok = GeometryUtils::computeSliceAndFit(resultObjPath,
                Eigen::Vector3d(0, target.yHeight, 0),
                Eigen::Vector3d(0, 1, 0), simData);

            if (ok) {
                // 加权求和
                totalError += std::abs((target.targetArea - simData.area) / target.targetArea);
                // ... 其他指标 ...
            }
            else {
                totalError += 10.0;
            }
        }
    }

    // 清理内存
    delete engine;
    for (auto m : models) delete m;

    return totalError;
}

bool SimulationRunner::exportElasticModulusToTecplot(
    Simulation::TetModel* model,
    const std::string& filename,
    const std::string& zoneName
) {
    if (!model) return false;

    // 获取父类指针以访问数据（根据你之前的类结构）
    Simulation::Model* baseModel = static_cast<Simulation::Model*>(model);

    const auto& vertices = model->get_Vertice_0(); // 节点
    const auto& indices = model->get_Indices();  // 单元拓扑
    int numNodes = vertices.size();
    int numTets = indices.size();

    // ==========================================
    // 1. 计算节点上的平均弹性模量 (Nodal Averaging)
    // ==========================================
    std::vector<double> nodeE_sum(numNodes, 0.0);
    std::vector<int> node_count(numNodes, 0);

	auto lameToYoungs = [](double lambda, double mu) {
		if (lambda + mu == 0.0) return 0.0; // 防止除零
		return mu * (3.0 * lambda + 2.0 * mu) / (lambda + mu);
	};

    for (int i = 0; i < numTets; ++i) {
        // 获取当前单元的材料参数 (注意：你需要确认类中有 get_Lambda/Mu 接口)
        // 假设 model 存储的是数组，可以通过 index 访问
        double lam = baseModel->get_Lambda(i);
        double mu = baseModel->get_Mu(i);

        // 换算为杨氏模量
        double E_elem = lameToYoungs(lam, mu);

        // 累加到该单元的4个节点上
        const auto& tetNodes = indices[i]; // std::vector<int> or array of size 4
        for (int nid : tetNodes) {
            nodeE_sum[nid] += E_elem;
            node_count[nid]++;
        }
    }

    // 计算平均值
    std::vector<double> nodeE_final(numNodes, 0.0);
    for (int i = 0; i < numNodes; ++i) {
        if (node_count[i] > 0) {
            nodeE_final[i] = nodeE_sum[i] / (double)node_count[i];
        }
        else {
            nodeE_final[i] = 0.0; // 孤立点处理
        }
    }

    // ==========================================
    // 2. 写入 Tecplot 文件
    // ==========================================
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << " for writing." << std::endl;
        return false;
    }

    // 设置浮点数精度
    file << std::fixed << std::setprecision(6);

    // 2.1 写入 Header
    file << "TITLE=\"Finite Element - Elastic Modulus\"" << std::endl;
    // 变量名：坐标 + 模量
    file << "VARIABLES=\"X\",\"Y\",\"Z\",\"ElasticModulus\"" << std::endl;

    // 2.2 写入 Zone 信息
    // DATAPACKING=POINT 表示先写完所有点的X,Y,Z,Val，再写拓扑
    file << "ZONE T=\"" << zoneName << "\"" << std::endl;
    file << "Nodes=" << numNodes << ", Elements=" << numTets << ", ZONETYPE=FETetrahedron" << std::endl;
    file << "DATAPACKING=POINT" << std::endl;

    // 2.3 写入节点数据 (X, Y, Z, E)
    for (int i = 0; i < numNodes; ++i) {
        file << vertices[i][0] << "\t"
            << vertices[i][1] << "\t"
            << vertices[i][2] << "\t"
            << nodeE_final[i] << std::endl;
    }

    // 2.4 写入单元拓扑 (Connectivity)
    // Tecplot 的索引是从 1 开始的，而 C++ 是从 0 开始的，所以需要 +1
    for (int i = 0; i < numTets; ++i) {
        const auto& tet = indices[i];
        file << (tet[0] + 1) << "\t"
            << (tet[1] + 1) << "\t"
            << (tet[2] + 1) << "\t"
            << (tet[3] + 1) << std::endl;
    }

    file.close();
    std::cout << "[Export] Tecplot file saved: " << filename << std::endl;
    return true;
}

