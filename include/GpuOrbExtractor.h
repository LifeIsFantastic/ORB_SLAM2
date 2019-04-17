/**
* This file is a part of the GPU-accelerated version of ORB-SLAM2.
*
* Author: Tao Han <tao dot han dot my dot cityu dot edu dot hk>
* 
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __GPUORBEXTRACTOR_H__
#define __GPUORBEXTRACTOR_H__

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>

#include "ORBextractor.h"
#include "cuda/Fast.hpp"
#include "cuda/Orb.hpp"

namespace ORB_SLAM2
{

class GpuOrbExtractor : public ORBextractor
{
public:
    
    GpuOrbExtractor(int nfeatures, float scaleFactor, int nlevels,
                    int iniThFAST, int minThFAST);

    virtual ~GpuOrbExtractor() {}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    virtual void operator()(cv::InputArray image,
                            cv::InputArray mask,
                            std::vector<cv::KeyPoint> &keypoints,
                            cv::OutputArray descriptors);

private:

    void computePyramid(cv::Mat image);

    void computeKeyPointsOctTree(std::vector< std::vector<cv::KeyPoint> > &allKeypoints);

    // std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys,
    //                                             const int &minX, const int &maxX,
    //                                             const int &minY, const int &maxY,
    //                                             const int &nFeatures, const int &level);

    // CUDA related members
    bool mbImagePyramidAllocFlag;
    std::vector<cv::cuda::GpuMat> mvImagePyramid;

    cv::cuda::Stream mcvStream;

    cv::Ptr<cv::cuda::Filter> mpGaussianFilter;

    cuda::GpuFast    mGpuFast;
    cuda::GpuIcAngle mGpuIcAngle;
    cuda::GpuOrb     mGpuOrb;

};

} //namespace ORB_SLAM2

#endif /* __GPUORBEXTRACTOR_H__ */