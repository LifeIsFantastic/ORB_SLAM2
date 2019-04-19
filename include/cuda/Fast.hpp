#pragma once
#ifndef __CUDA_FAST_HPP__
#define __CUDA_FAST_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_devptrs.hpp>

namespace ORB_SLAM2
{
namespace cuda
{
const float FEATURE_SIZE = 7.0;

class GpuFast
{
public:
    GpuFast(int hThres, int lThres, int maxKeypoints = 10000);
    ~GpuFast();

    void detect(cv::InputArray img,
                std::vector<cv::KeyPoint> &keypoints);

    void detectAsync(cv::InputArray img);
    void joinDetectAsync(std::vector<cv::KeyPoint> &keypoints);

    void getAngle(cv::InputArray img,
                  std::vector<cv::KeyPoint> &keypoints,
                  int minBorderX, int minBorderY, int halfPatch,
                  int octave, int size); // TODO: size (float or int ??)

    void getAngleAsync(cv::InputArray img,
                       std::vector<cv::KeyPoint> &keypoints,
                       int minBorderX, int minBorderY, int halfPatch,
                       int octave, int size); // TODO: size (float or int ??)
    void jointGetAngleAsync(std::vector<cv::KeyPoint> &keypoints);

    cv::cuda::Stream& getCvStream() { return mCvStream; }

    static void loadUMaxData(const int* umax, int count);

protected:
    unsigned int mHighThres;
    unsigned int mLowThres;
    unsigned int mMaxKeypoints;
    unsigned int mCountHost;

    cv::KeyPoint *mpKeyptsDevice;

    cv::cuda::GpuMat scoreMat;

    cv::cuda::Stream mCvStream;
    cudaStream_t mCudaStream;
};

} // namesapce cuda
} // namespace ORB_SLAM2

#endif /* __CUDA_FAST_HPP__ */