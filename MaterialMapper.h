// Core/MaterialMapper.h

#pragma once
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <vtkSmartPointer.h>
#include <vtkImplicitPolyDataDistance.h>
#include "solver/TetModel.h" 

struct AnatomicalRegion {
    std::string name;
    std::string stlPath;
    vtkSmartPointer<vtkImplicitPolyDataDistance> distFunc;
    int priority;
};

class MaterialMapper {
public:
    MaterialMapper();
    ~MaterialMapper();

    void addRegion(const std::string& name, const std::string& stlPath, int priority);
    void initialize();
    void applyMaterials(Simulation::Model* model, const std::map<std::string, double>& params, double searchRadius);

private:
    std::vector<AnatomicalRegion> m_regions;
};