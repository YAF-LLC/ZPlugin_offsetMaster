#include "resampleWithMarchingCubes.h"

#include "igl/signed_distance.h"

#include "igl/voxel_grid.h"
#include "igl/marching_cubes.h"

#include "igl/facet_components.h"
#include "igl/remove_unreferenced.h"

#include <vector>

// debug
#include "igl/writePLY.h"

template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedMCV,
    typename DerivedMCF>
IGL_INLINE void igl::resampleWithMarchingCubes(
    const Eigen::MatrixBase<DerivedV> &V,
    const Eigen::MatrixBase<DerivedF> &F,
    const typename DerivedV::Scalar &meshHeight,
    const typename DerivedV::Scalar &voxelSize,
    const typename DerivedV::Scalar &offset,
    const bool &largestOnly,
    const bool &inward,
    Eigen::PlainObjectBase<DerivedMCV> &mcV,
    Eigen::PlainObjectBase<DerivedMCF> &mcF)
{
    typedef typename DerivedV::Scalar Scalar;
    typedef Eigen::Matrix<Scalar, 1, 3> RowVector3S;

    ////
    // setup signed distance (with precompute for frequent queryings)
    //   AABB
    // igl::AABB<DerivedV, 3> tree3;
    // tree3.init(V, F);

    Scalar modifiedVoxelSize = voxelSize;
    Scalar modifiedOffset = offset;
    if (meshHeight > 0.0)
    {
        // exported size is not managed by user
        // we need to scale voxelSize based on the height
        const Scalar exportedHeight = V.col(1).maxCoeff() - V.col(1).minCoeff();
        modifiedVoxelSize = voxelSize * exportedHeight / meshHeight;
        modifiedOffset = offset * exportedHeight / meshHeight;
    }

    if (inward)
    {
        modifiedOffset *= -1;
    }

    const Scalar BBLongest = (V.colwise().maxCoeff() - V.colwise().minCoeff()).maxCoeff();
    const int voxelCount = static_cast<int>(std::ceil((BBLongest + modifiedOffset) / modifiedVoxelSize)) + 2;

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> GV;
    Eigen::RowVector3i res;
    igl::voxel_grid(V, std::abs(modifiedOffset), voxelCount, 1, GV, res);

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> S;
    {
        Eigen::Matrix<int, Eigen::Dynamic, 1> I;
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C, N;
        signed_distance(GV, V, F, igl::SIGNED_DISTANCE_TYPE_FAST_WINDING_NUMBER, S, I, C, N);
    }

    igl::marching_cubes(S, GV, res(0), res(1), res(2), modifiedOffset, mcV, mcF);

    if (largestOnly)
    {
        // remove smaller volumes if needed
        Eigen::Matrix<int, Eigen::Dynamic, 1> C;
        igl::facet_components(mcF, C);

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> mcFN;
        igl::per_face_normals(mcV, mcF, mcFN);

        const auto &faceArea = [&](const int &fIdx)
        {
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> v01 = mcV.row(mcF(fIdx, 1)) - mcV.row(mcF(fIdx, 0));
            const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> v02 = mcV.row(mcF(fIdx, 2)) - mcV.row(mcF(fIdx, 0));
            return 0.5 * v01.template head<3>().cross(v02.template head<3>()).norm();
        };
        const auto &signedVolumeFrag = [&](const int &fIdx)
        {
            const Scalar fArea = faceArea(fIdx);
            const Scalar height = mcV.row(mcF(fIdx, 0)).dot(mcFN.row(fIdx));
            return fArea * height / 3.0;
        };

        std::vector<std::vector<int>> faceIdVec;
        std::vector<Scalar> volume;

        faceIdVec.resize(C.maxCoeff() + 1);
        volume.resize(C.maxCoeff() + 1, 0.0);
        for (int f = 0; f < C.rows(); ++f)
        {
            faceIdVec.at(C(f, 0)).push_back(f);
            volume.at(C(f, 0)) += signedVolumeFrag(f);
        }

        Scalar maxVolume = 0;
        int maxVolumeCIdx = -1;
        for (int cIdx = 0; cIdx < faceIdVec.size(); ++cIdx)
        {
            if (maxVolume < volume.at(cIdx))
            {
                maxVolume = volume.at(cIdx);
                maxVolumeCIdx = cIdx;
            }
        }

        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> tmpV = std::move(mcV);
        Eigen::Matrix<typename DerivedMCF::Scalar, Eigen::Dynamic, Eigen::Dynamic> tmpF;
        tmpF.resize(faceIdVec.at(maxVolumeCIdx).size(), mcF.cols());
        for (int fIdx = 0; fIdx < faceIdVec.at(maxVolumeCIdx).size(); ++fIdx)
        {
            tmpF.row(fIdx) = mcF.row(faceIdVec.at(maxVolumeCIdx).at(fIdx));
        }
        Eigen::Matrix<int, Eigen::Dynamic, 1> I;
        igl::remove_unreferenced(tmpV, tmpF, mcV, mcF, I);
    }

    // debug
    igl::writePLY("example/input.ply", V, F);
    igl::writePLY("example/mc.ply", mcV, mcF);
}
