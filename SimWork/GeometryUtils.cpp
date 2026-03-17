#include "GeometryUtils.h"
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkStripper.h>
#include <vtkCenterOfMass.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCardinalSpline.h>
#include <vtkKochanekSpline.h> // [新增] 用于带张力的样条
#include <vtkHausdorffDistancePointSetFilter.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkPointData.h>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>

// ... loadSTL 和 loadOBJ 保持不变 ...
vtkSmartPointer<vtkPolyData> GeometryUtils::loadSTL(const std::string& filepath) {
    auto reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filepath.c_str());
    reader->Update();
    return reader->GetOutput();
}

vtkSmartPointer<vtkPolyData> GeometryUtils::loadOBJ(const std::string& filepath) {
    auto reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName(filepath.c_str());
    reader->Update();
    return reader->GetOutput();
}

// [新增] 质心对齐
vtkSmartPointer<vtkPolyData> GeometryUtils::alignToCentroid(vtkPolyData* source, vtkPolyData* target) {
    if (!source || !target) return nullptr;

    auto centerFilter = vtkSmartPointer<vtkCenterOfMass>::New();

    // Sim Center
    centerFilter->SetInputData(target);
    centerFilter->Update();
    double targetCenter[3];
    centerFilter->GetCenter(targetCenter);

    // Source Center
    centerFilter->SetInputData(source);
    centerFilter->Update();
    double sourceCenter[3];
    centerFilter->GetCenter(sourceCenter);

    // Translation: Target - Source
    auto transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(targetCenter[0] - sourceCenter[0],
        targetCenter[1] - sourceCenter[1],
        targetCenter[2] - sourceCenter[2]);

    auto transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputData(source);
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    return transformFilter->GetOutput();
}

vtkSmartPointer<vtkPolyData> GeometryUtils::alignToICP(vtkPolyData* source, vtkPolyData* target) {
	// 先进行质心对齐，提供一个好的初始位置
	auto alignedSource = alignToCentroid(source, target);

	// 使用 ICP 进行刚性配准
	auto icp = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();
	icp->SetSource(alignedSource);
	icp->SetTarget(target);
	icp->GetLandmarkTransform()->SetModeToRigidBody();
	icp->SetMaximumNumberOfIterations(50);
    icp->StartByMatchingCentroidsOn();
	icp->Update();

	auto transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputData(alignedSource);
	transformFilter->SetTransform(icp);
	transformFilter->Update();

	return transformFilter->GetOutput();
}

bool GeometryUtils::computeSliceAndFit(vtkPolyData* poly, const Eigen::Vector3d& origin, const Eigen::Vector3d& normal, ProfileData& outProfile) {
    outProfile.valid = false;
    if (!poly || poly->GetNumberOfPoints() == 0) return false;

    // 1. 切割
    auto plane = vtkSmartPointer<vtkPlane>::New();
    plane->SetOrigin(origin.x(), origin.y(), origin.z());
    plane->SetNormal(normal.x(), normal.y(), normal.z());

    auto cutter = vtkSmartPointer<vtkCutter>::New();
    cutter->SetInputData(poly);
    cutter->SetCutFunction(plane);
    cutter->Update();

    auto stripper = vtkSmartPointer<vtkStripper>::New();
    stripper->SetInputConnection(cutter->GetOutputPort());
    stripper->Update();

    auto slicePoly = stripper->GetOutput();
    if (slicePoly->GetNumberOfPoints() < 10) return false;

    // 2. 局部坐标系 (u, v) 用于投影和反投影
    Eigen::Vector3d n = normal.normalized();
    Eigen::Vector3d u, v;
    if (std::abs(n.x()) < 0.9) u = n.cross(Eigen::Vector3d(1, 0, 0)).normalized();
    else u = n.cross(Eigen::Vector3d(0, 1, 0)).normalized();
    v = n.cross(u).normalized();

    // 3. 质心计算
    Eigen::Vector3d centroid3D(0, 0, 0);
    vtkIdType numPts = slicePoly->GetNumberOfPoints();
    std::vector<Eigen::Vector3d> points3D;
    for (vtkIdType i = 0; i < numPts; ++i) {
        double p[3];
        slicePoly->GetPoint(i, p);
        Eigen::Vector3d pt(p[0], p[1], p[2]);
        points3D.push_back(pt);
        centroid3D += pt;
    }
    centroid3D /= (double)numPts;
    outProfile.centroid = centroid3D;

    // 4. 分桶 (Binning)
    const int NUM_BINS = 36;
    struct BinData { double sumR = 0; int count = 0; };
    std::vector<BinData> bins(NUM_BINS);

    for (const auto& pt : points3D) {
        Eigen::Vector3d vec = pt - centroid3D;
        double x_loc = vec.dot(u);
        double y_loc = vec.dot(v);
        double theta = std::atan2(y_loc, x_loc);
        if (theta < 0) theta += 2.0 * EIGEN_PI;
        double r = std::sqrt(x_loc * x_loc + y_loc * y_loc);

        int idx = (int)(theta / (2.0 * EIGEN_PI) * NUM_BINS);
        idx = std::clamp(idx, 0, NUM_BINS - 1);
        bins[idx].sumR += r;
        bins[idx].count++;
    }

    // 5. 填补空桶
    for (int i = 0; i < NUM_BINS; ++i) {
        if (bins[i].count == 0) {
            int left = (i - 1 + NUM_BINS) % NUM_BINS;
            while (bins[left].count == 0 && left != i) left = (left - 1 + NUM_BINS) % NUM_BINS;
            int right = (i + 1) % NUM_BINS;
            while (bins[right].count == 0 && right != i) right = (right + 1) % NUM_BINS;

            if (bins[left].count > 0 && bins[right].count > 0) {
                double rL = bins[left].sumR / bins[left].count;
                double rR = bins[right].sumR / bins[right].count;
                bins[i].sumR = (rL + rR) / 2.0;
                bins[i].count = 1;
            }
            else { return false; }
        }
    }

    // 6. 样条拟合
    // =========================================================
    // [新增] 5.5 数据平滑 (Data Smoothing) - 消除局部抖动
    // =========================================================
    // 使用简单的 3 点高斯平滑 [0.25, 0.5, 0.25]
    std::vector<double> smoothedRadii(NUM_BINS);
    for (int i = 0; i < NUM_BINS; ++i) {
        // 处理周期性边界 (圆环首尾相接)
        int prev = (i - 1 + NUM_BINS) % NUM_BINS;
        int next = (i + 1) % NUM_BINS;

        double rPrev = bins[prev].sumR / bins[prev].count;
        double rCurr = bins[i].sumR / bins[i].count;
        double rNext = bins[next].sumR / bins[next].count;

        // 加权平均 (权重可调，中间越大越保持原形，越小越平滑)
        smoothedRadii[i] = 0.25 * rPrev + 0.5 * rCurr + 0.25 * rNext;
    }

    // =========================================================
    // [修改] 6. 样条拟合 (Spline Fitting)
    // =========================================================
    // 改用 KochanekSpline，因为它支持 Tension (张力) 参数
    auto spline = vtkSmartPointer<vtkKochanekSpline>::New();
    spline->ClosedOn();

    // [关键参数] Tension (张力): 范围 [-1, 1]
    // 0.0 = Catmull-Rom (类似 Cardinal Spline，适中)
    // >0.0 (如 0.5) = 曲线更紧凑 (Tight)，减少起伏和过冲，适合支架轮廓
    // <0.0 (如 -0.5) = 曲线更松弛 (Round)，更圆
    spline->SetDefaultTension(0.5);

    for (int i = 0; i < NUM_BINS; ++i) {
        // 使用平滑后的数据
        spline->AddPoint(i, smoothedRadii[i]);
    }

    // 7. 重采样并还原为 3D 坐标
    outProfile.radii.resize(360);
    outProfile.points3D.resize(360); // [新增]

    double areaSum = 0;
    double periSum = 0;
    Eigen::Vector2d prevPt2D(0, 0);

    for (int i = 0; i < 360; ++i) {
        double t = (double)i / 360.0 * NUM_BINS;
        double r = spline->Evaluate(t);
        outProfile.radii[i] = r;

        double theta = (double)i / 360.0 * 2.0 * EIGEN_PI;
        double cosT = std::cos(theta);
        double sinT = std::sin(theta);

        // 计算 3D 坐标: P = C + r*cos(t)*u + r*sin(t)*v
        outProfile.points3D[i] = centroid3D + r * cosT * u + r * sinT * v;

        // 计算面积/周长用的 2D 坐标
        Eigen::Vector2d currPt2D(r * cosT, r * sinT);
        if (i > 0) {
            areaSum += 0.5 * std::abs(prevPt2D.x() * currPt2D.y() - prevPt2D.y() * currPt2D.x());
            periSum += (currPt2D - prevPt2D).norm();
        }
        prevPt2D = currPt2D;
    }
    // Close loop
    double r0 = outProfile.radii[0];
    Eigen::Vector2d pt0_2D(r0, 0);
    areaSum += 0.5 * std::abs(prevPt2D.x() * pt0_2D.y() - prevPt2D.y() * pt0_2D.x());
    periSum += (pt0_2D - prevPt2D).norm();

    outProfile.area = areaSum;
    outProfile.circumference = periSum;
    outProfile.valid = true;
    return true;
}

// 导出 CSV
void GeometryUtils::saveProfileToCSV(const std::string& filepath, const ProfileData& profile) {
    std::ofstream out(filepath);
    if (!out.is_open()) return;
    out << "Angle_Idx,Radius,X_3D,Y_3D,Z_3D\n";
    for (int i = 0; i < 360; ++i) {
        const auto& p = profile.points3D[i];
        out << i << "," << profile.radii[i] << "," << p.x() << "," << p.y() << "," << p.z() << "\n";
    }
    out.close();
}

// [新增] 导出 OBJ 可视化文件
void GeometryUtils::saveProfileGeometry(const std::string& filepath, const ProfileData& profile) {
    std::ofstream out(filepath);
    if (!out.is_open()) return;

    out << "# Slice Profile Curve\n";
    // 1. 写入顶点 (v x y z)
    for (const auto& p : profile.points3D) {
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
    }

    // 2. 写入连线 (l 1 2 3 ... N 1)
    // OBJ 索引从 1 开始
    out << "l";
    for (size_t i = 0; i < profile.points3D.size(); ++i) {
        out << " " << (i + 1);
    }
    out << " 1\n"; // 闭合回到第一个点

    out.close();
}

// 计算基础误差 (不配准)
GeometryUtils::SimilarityMetrics GeometryUtils::computeErrors(vtkPolyData* source, vtkPolyData* target) {
    if (!source || !target) return { 1e9, 1e9, 1e9 };

    auto distFilter = vtkSmartPointer<vtkDistancePolyDataFilter>::New();
    distFilter->SetInputData(0, source);
    distFilter->SetInputData(1, target);
    distFilter->SignedDistanceOff();
    distFilter->Update();

    auto output = distFilter->GetOutput();
    auto distArray = output->GetPointData()->GetScalars();

    double maxDist = 0, sumDist = 0, sumSqDist = 0;
    vtkIdType n = distArray->GetNumberOfTuples();
    for (vtkIdType i = 0; i < n; ++i) {
        double val = distArray->GetTuple1(i);
        if (val > maxDist) maxDist = val;
        sumDist += val;
        sumSqDist += val * val;
    }
    return { maxDist, sumDist / n, std::sqrt(sumSqDist / n) };
}

double GeometryUtils::getDistanceToMesh(const Eigen::Vector3d& point, vtkImplicitPolyDataDistance* distanceFunc) {
    double p[3] = { point.x(), point.y(), point.z() };
    return distanceFunc->EvaluateFunction(p);
}