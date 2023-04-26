//
// Copyright (C) 2023 Kazutaka Nakashima (kazutaka.nakashima@n-taka.info)
// 
// GPLv3
//
// This file is part of offsetMaster.
// 
// offsetMaster is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// 
// offsetMaster is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with offsetMaster. If not, see <https://www.gnu.org/licenses/>.
//

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
