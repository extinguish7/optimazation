// Utils/GeometryUtils.h

#pragma once
#include <string>
#include <vector>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkImplicitPolyDataDistance.h>
#include <Eigen/Dense>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkPointData.h>
#include <vtkMassProperties.h>
#include <vtkCenterOfMass.h>
#include <tuple>
#include <vtkSTLWriter.h>

class GeometryUtils {
public:
    struct EllipseData {
        double longAxis;
        double shortAxis;
        double circumference;
        double area;
    };

    // 加载STL
    static vtkSmartPointer<vtkPolyData> loadSTL(const std::string& filepath);
	// 加载OBJ
	static vtkSmartPointer<vtkPolyData> loadOBJ(const std::string& filepath);

    // 计算点到Mesh的有符号距离
    static double getDistanceToMesh(const Eigen::Vector3d& point, vtkImplicitPolyDataDistance* distanceFunc);

    //计算两个 Mesh 之间的 Hausdorff 距离(解决需求 3)
    static double computeHausdorffDistance(const std::string& sourceMeshPath, const std::string& targetMeshPath);

    // 切割平面并拟合椭圆 (整合了你原代码中的 logic)
    static bool computeSliceAndFit(const std::string& objPath,
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& normal,
        EllipseData& outData);


    struct SimilarityMetrics {
        double maxDistance;  // Hausdorff
        double meanDistance; // 平均距离 (反映整体贴合度)
        double rmse;         // 均方根误差 (对大误差敏感，但不至于像Hausdorff那么极端)
    };

    // [新增] 辅助函数：提取点云的主轴 (PCA)
    static void computePCA(vtkPolyData* poly, Eigen::Vector3d& outCenter, Eigen::Matrix3d& outEigenVectors);

    /**
     * @brief 执行自动配准并计算综合误差
     * @param simMeshPath 仿真生成的模型路径
     * @param truthMeshPath 真实的标注模型路径
     * @return 包含多种误差指标的结构体
     */
    static SimilarityMetrics computeRegistrationAndError(
        const std::string& simMeshPath,
        const std::string& truthMeshPath,
        const std::string& saveRegisteredPath = "");
};