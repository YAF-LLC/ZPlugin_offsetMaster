#ifndef RESAMPLE_WITH_MARCHING_CUBES_H
#define RESAMPLE_WITH_MARCHING_CUBES_H

#include "igl/igl_inline.h"

#include <Eigen/Core>

namespace igl
{
  template <
      typename DerivedV,
      typename DerivedF,
      typename DerivedMCV,
      typename DerivedMCF>
  IGL_INLINE void resampleWithMarchingCubes(
      const Eigen::MatrixBase<DerivedV> &V,
      const Eigen::MatrixBase<DerivedF> &F,
      const typename DerivedV::Scalar &meshHeight,
      const typename DerivedV::Scalar &voxelSize,
      const typename DerivedV::Scalar &offset,
      const bool &largestOnly,
      const bool &inward,
      Eigen::PlainObjectBase<DerivedMCV> &mcV,
      Eigen::PlainObjectBase<DerivedMCF> &mcF);
}

#include "../src/resampleWithMarchingCubes.cpp"

#endif
