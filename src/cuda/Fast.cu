#include "cuda/Fast.hpp"

namespace ORB_SLAM2
{
namespace cuda
{

GpuFast::GpuFast(int hThres, int lThres, int maxKeypoints)
    : mHighThres(hThres), mLowThres(lThres), mMaxKeypoints(maxKeypoints)
{
    // TODO
}

GpuFast::~GpuFast()
{
    // TODO
}

void GpuFast::detect(cv::InputArray img,
                     std::vector<cv::KeyPoint> &keypoints)
{
    this->detectAsync(img);

    this->joinDetectAsync(keypoints);
}

void GpuFast::detectAsync(cv::InputArray img)
{
    const cv::cuda::GpuMat image = img.getGpuMat();

    if (scoreMat.empty())
    {
        scoreMat = GpuMat(image.size(), CV_32SC1)
    }

    scoreMat.setTo(Scalar::all(0), mcvStream);
}

} // namespace cuda
} // namespace ORB_SLAM2