#pragma once
#include <string>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkImplicitPolyDataDistance.h>
#include <Eigen/Dense>
#include <tuple>

class GeometryUtils {
public:
    // [结构体] 拟合后的截面数据
    struct ProfileData {
        bool valid;
        Eigen::Vector3d centroid;       // 截面质心
        std::vector<double> radii;      // 360度极坐标半径 (用于计算 Loss)
        std::vector<Eigen::Vector3d> points3D; // [新增] 拟合后的3D空间点 (用于可视化导出)
        double area;
        double circumference;
        double circularity;
    };

    // [结构体] 基础误差指标
    struct SimilarityMetrics {
        double maxDistance;  // Hausdorff
        double meanDistance; // 平均距离
        double rmse;         // RMSE
    };

    // ================= 文件加载 =================
    static vtkSmartPointer<vtkPolyData> loadSTL(const std::string& filepath);
    static vtkSmartPointer<vtkPolyData> loadOBJ(const std::string& filepath);

    // ================= 核心几何操作 =================

    /**
     * @brief [新增] 仅将 Source 移动到 Target 的质心位置 (不旋转)
     * @param source 需要移动的模型 (通常是 Target/Truth)
     * @param target 基准模型 (通常是 Sim)
     * @return 移动后的新 PolyData
     */
    static vtkSmartPointer<vtkPolyData> alignToCentroid(vtkPolyData* source, vtkPolyData* target);

	/**
    * @brief [新增] 使用 ICP 进行刚性配准 (包含旋转和平移)
	* @param source 需要移动的模型 (通常是 Target/Truth)
	* @param target 基准模型 (通常是 Sim)
	* @return 配准后的新 PolyData
	*/
	static vtkSmartPointer<vtkPolyData> alignToICP(vtkPolyData* source, vtkPolyData* target);

    /**
     * @brief [修改] 切割模型并拟合平滑闭合曲线 (Spline Fitting)
     */
    static bool computeSliceAndFit(vtkPolyData* poly,
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& normal,
        ProfileData& outProfile);

    /**
     * @brief [新增] 将切片数据导出为 CSV 用于调试/绘图
     * @param filepath 输出路径
     * @param profile 拟合好的数据
     */
    static void saveProfileToCSV(const std::string& filepath, const ProfileData& profile);

    /**
     * @brief [新增] 导出切片的三维几何模型 (OBJ格式)
     * @param filepath 输出路径 (建议以 .obj 结尾)
     * @param profile 数据
     */
    static void saveProfileGeometry(const std::string& filepath, const ProfileData& profile);

    // ================= 误差计算 =================

    /**
     * @brief 计算纯几何误差 (Hausdorff, Mean, RMSE)
     * 注意：不再包含配准过程，输入必须是已经对齐好的模型
     */
    static SimilarityMetrics computeErrors(vtkPolyData* source, vtkPolyData* target);

    // 计算点到Mesh的有符号距离
    static double getDistanceToMesh(const Eigen::Vector3d& point, vtkImplicitPolyDataDistance* distanceFunc);

};