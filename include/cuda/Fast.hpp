#pragma once
#ifndef __CUDA_FAST_HPP__
#define __CUDA_FAST_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_devptrs.hpp>

#include "Thirdparty/uni_fusion/common/include/uni_fusion/common.hpp"
#include "Thirdparty/uni_fusion/common/include/uni_fusion/common/algorithms/cudev.hpp"

namespace ORB_SLAM2 { namespace cuda {

const float FEATURE_SIZE = 7.0;

class GpuFast
{
public:
    GpuFast(int hThres, int lThres, int maxKeypoints = 10000);
    ~GpuFast();

    static void loadUMaxData(const int* umax, int count);

    void detect(cv::InputArray img,
                std::vector<cv::KeyPoint> &keypoints);

    void detectAsync(cv::InputArray img);
    void joinDetectAsync(std::vector<cv::KeyPoint> &keypoints);

    void getAngle(cv::InputArray img,
                  std::vector<cv::KeyPoint> &keypoints,
                  int minBorderX, int minBorderY, int halfPatch,
                  int octave, float size);

    void getAngleAsync(cv::InputArray img,
                       std::vector<cv::KeyPoint> &keypoints,
                       int minBorderX, int minBorderY, int halfPatch,
                       int octave,
                       float size);
    void joinGetAngleAsync(std::vector<cv::KeyPoint> &keypoints);

    cv::cuda::Stream& getCvStream() { return mCvStream; }

protected:
    unsigned int mHighThres;
    unsigned int mLowThres;
    unsigned int mMaxKeypoints;

    int mCountHost;
    int *mpCountDevice;

    cv::KeyPoint* mpKeyptsDevice; 

    cv::cuda::GpuMat scoreMat;

    cv::cuda::Stream mCvStream;
    cudaStream_t mCudaStream;
};

}} // namespace

#endif /* __CUDA_FAST_HPP__ */
