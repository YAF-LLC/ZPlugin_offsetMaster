#include "resample.h"

#include "nlohmann/json.hpp"

#include <filesystem>
namespace fs = std::filesystem;

#include "readGoZAndTriangulate.h"
#include "resampleWithMarchingCubes.h"
#include "writeGoZFile.h"

// debug
#include <iostream>

extern "C" DLLEXPORT float resample(char *someText, double optValue, char *outputBuffer, int optBuffer1Size, char *pOptBuffer2, int optBuffer2Size, char **zData)
{
    ////
    // parse parameter (JSON)
    fs::path jsonPath(someText);
    std::ifstream ifs(jsonPath.string());
    nlohmann::json json = nlohmann::json::parse(ifs);
    ifs.close();

    const std::string rootString = json.at("root").get<std::string>();
    fs::path rootPath(rootString);
    rootPath.make_preferred();

    // load GoZ file
    const std::string meshFileRelPathStr = json.at("meshFile").get<std::string>();
    fs::path meshFileRelPath(meshFileRelPathStr);
    fs::path meshFilePath(rootPath);
    meshFilePath /= meshFileRelPath;
    meshFilePath.make_preferred();

    std::string meshName;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> V;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> UV_u;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> UV_v;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> VC;
    Eigen::Matrix<double, Eigen::Dynamic, 1> M;
    Eigen::Matrix<int, Eigen::Dynamic, 1> G;
    readGoZAndTriangulate(meshFilePath, meshName, V, F, UV_u, UV_v, VC, M, G);

    // load parameters
    const double meshHeight = json.at("ZBrush").at("meshHeight").get<double>();
    const double voxelSize = json.at("ZBrush").at("voxelSize").get<double>();
    const double offset = json.at("ZBrush").at("offset").get<double>();
    const bool largestOnly = json.at("ZBrush").at("largestOnly").get<bool>();
    const bool inverted = json.at("ZBrush").at("inverted").get<bool>();
    const bool inward = json.at("ZBrush").at("inward").get<bool>();

    // apply remesh with Dual Contouring
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mcV;
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> mcF;

    igl::resampleWithMarchingCubes(
        V, F,
        meshHeight, voxelSize, offset, largestOnly, inward,
        mcV, mcF);

    // invert if needed
    if (inverted)
    {
        Eigen::Matrix<int, Eigen::Dynamic, 1> tmpCol = mcF.col(1);
        mcF.col(1) = mcF.col(2);
        mcF.col(2) = tmpCol;
    }

    // export GoZ file
    //   covert matrix to vector style
    std::vector<std::vector<double>>
        mcVVec(mcV.rows(), std::vector<double>(mcV.cols()));
    for (int v = 0; v < mcV.rows(); ++v)
    {
        for (int xyz = 0; xyz < mcV.cols(); ++xyz)
        {
            mcVVec.at(v).at(xyz) = mcV(v, xyz);
        }
    }

    std::vector<std::vector<int>> mcFVec(mcF.rows(), std::vector<int>(mcF.cols()));
    for (int f = 0; f < mcF.rows(); ++f)
    {
        for (int fv = 0; fv < mcF.cols(); ++fv)
        {
            mcFVec.at(f).at(fv) = mcF(f, fv);
        }
    }

    // update meshName
    fs::path exportMeshFilePath(meshFilePath.parent_path());
    {
        exportMeshFilePath /= meshFilePath.stem();
        exportMeshFilePath += "_offset";
        exportMeshFilePath += meshFilePath.extension();
    }

    std::string exportMeshName(meshName);
    exportMeshName += "_offset";

    FromZ::writeGoZFile(exportMeshFilePath.string(), exportMeshName, mcVVec, mcFVec);

    return 1.0f;
}
