// Utils/GeometryUtils.cpp

#include "GeometryUtils.h"
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkStripper.h>
#include <vtkMath.h>
#include <opencv2/opencv.hpp>
#include <vtkHausdorffDistancePointSetFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkTransform.h>
#include <vtkLandmarkTransform.h>
#include <numeric>
#include <cmath>

vtkSmartPointer<vtkPolyData> GeometryUtils::loadSTL(const std::string& filepath) {
    auto reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filepath.c_str());
    reader->Update();
    return reader->GetOutput();
}

vtkSmartPointer<vtkPolyData> GeometryUtils::loadOBJ(const std::string& filepath)
{
	auto reader = vtkSmartPointer<vtkOBJReader>::New();
	reader->SetFileName(filepath.c_str());
	reader->Update();
	return reader->GetOutput();
}

double GeometryUtils::getDistanceToMesh(const Eigen::Vector3d& point, vtkImplicitPolyDataDistance* distanceFunc) {
    double p[3] = { point.x(), point.y(), point.z() };
    return distanceFunc->EvaluateFunction(p);
}

double GeometryUtils::computeHausdorffDistance(const std::string& sourceMeshPath, const std::string& targetMeshPath)
{
    auto sourcePoly = loadOBJ(sourceMeshPath); // 也可以加个 loadOBJ 的判断
    auto targetPoly = loadSTL(targetMeshPath); // 确保 loadSTL 支持或自动判断格式

    if (!sourcePoly || !targetPoly) return 1e6; // 惩罚极大值

    auto hausdorff = vtkSmartPointer<vtkHausdorffDistancePointSetFilter>::New();
    hausdorff->SetInputData(0, sourcePoly);
    hausdorff->SetInputData(1, targetPoly);
    hausdorff->Update();

    // 获取单向或双向距离，通常取最大值
    double dist = hausdorff->GetHausdorffDistance();
    return dist;
}

// 辅助函数：计算两向量旋转矩阵 (原代码逻辑)
static void computeRotMatFromTwoVectors(const Eigen::Vector3d& A_dir, const Eigen::Vector3d& B_dir, Eigen::Matrix4d& rot) {
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(A_dir, B_dir);
    rot = Eigen::Matrix4d::Identity();
    rot.block<3, 3>(0, 0) = q.toRotationMatrix();
}

bool GeometryUtils::computeSliceAndFit(const std::string& objPath, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal, EllipseData& outData) {
    // 1. 读取 OBJ
    auto reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(objPath.c_str());
    reader->Update();
    if (reader->GetOutput()->GetNumberOfPoints() == 0) return false;

    // 2. 切割
    auto plane = vtkSmartPointer<vtkPlane>::New();
    plane->SetOrigin(origin.x(), origin.y(), origin.z());
    plane->SetNormal(normal.x(), normal.y(), normal.z());

    auto cutter = vtkSmartPointer<vtkCutter>::New();
    cutter->SetInputConnection(reader->GetOutputPort());
    cutter->SetCutFunction(plane);
    cutter->Update();

    auto stripper = vtkSmartPointer<vtkStripper>::New();
    stripper->SetInputConnection(cutter->GetOutputPort());
    stripper->Update();

    vtkPoints* pts = stripper->GetOutput()->GetPoints();
    if (!pts || pts->GetNumberOfPoints() < 5) return false;

    // 3. 投影到 2D 平面进行拟合 (OpenCV)
    Eigen::Matrix4d rotMat;
    computeRotMatFromTwoVectors(normal, Eigen::Vector3d(0, 0, 1), rotMat); // 旋转到 Z 轴

    std::vector<cv::Point2f> cvPoints;
    for (vtkIdType i = 0; i < pts->GetNumberOfPoints(); i++) {
        double p[3];
        pts->GetPoint(i, p);
        Eigen::Vector4d p3(p[0], p[1], p[2], 1.0);
        Eigen::Vector4d p2 = rotMat * p3; // 旋转
        // 加上偏移校正，这里简化直接取投影后的xy
        cvPoints.push_back(cv::Point2f((float)p2.x(), (float)p2.y()));
    }

    if (cvPoints.size() < 5) return false;

    cv::RotatedRect ellipse = cv::fitEllipse(cvPoints);
    outData.longAxis = ellipse.size.height / 2.0;
    outData.shortAxis = ellipse.size.width / 2.0;
    outData.area = CV_PI * outData.longAxis * outData.shortAxis;
    // Ramanujan 近似
    double a = outData.longAxis;
    double b = outData.shortAxis;
    outData.circumference = CV_PI * (3 * (a + b) - sqrt((3 * a + b) * (a + 3 * b)));

    return true;
}

GeometryUtils::SimilarityMetrics GeometryUtils::computeRegistrationAndError(const std::string& simMeshPath, const std::string& truthMeshPath, const std::string& saveRegisteredPath)
{
    // 1. 加载模型
    auto simPoly = loadOBJ(simMeshPath);     // 这是不动的基准 (Target)
    auto truthPoly = loadSTL(truthMeshPath); // 这是要移动的模型 (Source)

    if (!simPoly || !truthPoly) return { 1e9, 1e9, 1e9 };

    // ==========================================
    // 步骤 A: 粗配准 (将 Truth 移动到 Sim 的质心)
    // ==========================================
    auto centerFilter = vtkSmartPointer<vtkCenterOfMass>::New();

    // 计算 Simulation 质心
    centerFilter->SetInputData(simPoly);
    centerFilter->Update();
    double simCenter[3];
    centerFilter->GetCenter(simCenter);

    // 计算 Ground Truth 质心
    centerFilter->SetInputData(truthPoly);
    centerFilter->Update();
    double truthCenter[3];
    centerFilter->GetCenter(truthCenter);

    // [修改点 1] 计算位移向量：目标(Sim) - 源(Truth)
    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(simCenter[0] - truthCenter[0],
        simCenter[1] - truthCenter[1],
        simCenter[2] - truthCenter[2]);

    auto transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(truthPoly); // [修改点 2] 移动的是 Truth
    transformFilter->SetTransform(transform);
    transformFilter->Update();
    auto preAlignedTruth = transformFilter->GetOutput();

    // ==========================================
    // 步骤 B: 精细配准 (ICP: Truth -> Sim)
    // ==========================================
    auto icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
    icp->SetSource(preAlignedTruth); // [修改点 3] 源是 Truth
    icp->SetTarget(simPoly);         // [修改点 4] 目标是 Sim
    icp->GetLandmarkTransform()->SetModeToRigidBody();
    icp->SetMaximumNumberOfIterations(50);
    icp->StartByMatchingCentroidsOn();
    icp->Update();

    // 应用变换得到最终的 Truth
    auto finalFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    finalFilter->SetInputData(preAlignedTruth);
    finalFilter->SetTransform(icp);
    finalFilter->Update();
    auto finalTruth = finalFilter->GetOutput();

    // ==========================================
    // [新增] 步骤 Save: 保存配准后的真实模型
    // ==========================================
    if (!saveRegisteredPath.empty()) {
        auto writer = vtkSmartPointer<vtkSTLWriter>::New();
        writer->SetFileName(saveRegisteredPath.c_str());
        writer->SetInputData(finalTruth);
        writer->SetFileTypeToBinary(); // 使用二进制格式减小体积
        writer->Write();
        std::cout << "Registered Ground Truth saved to: " << saveRegisteredPath << std::endl;
    }

    // ==========================================
    // 步骤 C: 计算误差场 (Truth vs Sim)
    // ==========================================
    auto distFilter = vtkSmartPointer<vtkDistancePolyDataFilter>::New();
    distFilter->SetInputData(0, finalTruth); // 计算从配准后的Truth
    distFilter->SetInputData(1, simPoly);    // 到 Sim 的距离
    distFilter->SignedDistanceOff();
    distFilter->Update();

    // ... (后续计算均值/RMSE的代码保持不变) ...

    // 只是为了完整性，这里重复一下后面的代码：
    auto output = distFilter->GetOutput();
    auto distArray = output->GetPointData()->GetScalars();
    if (!distArray) return { 1e9, 1e9, 1e9 };

    double maxDist = 0.0, sumDist = 0.0, sumSqDist = 0.0;
    vtkIdType numPoints = distArray->GetNumberOfTuples();

    for (vtkIdType i = 0; i < numPoints; ++i) {
        double val = distArray->GetTuple1(i);
        sumDist += val;
        sumSqDist += (val * val);
        if (val > maxDist) maxDist = val;
    }

    return { maxDist, sumDist / numPoints, std::sqrt(sumSqDist / numPoints) };
}
