#include "cuda/Fast.hpp"

#include <helper_cuda.h>
#include <cooperative_groups.h>
namespace cg = cooperative_groups;

#include <opencv2/core/cuda/common.hpp>
#include <opencv2/core/cuda/utility.hpp>
#include <opencv2/core/cuda/reduce.hpp>
#include <opencv2/core/cuda/functional.hpp>

namespace ORB_SLAM2
{
namespace cuda
{
using namespace cv;
using namespace cv::cuda;

__global__
void detectKeypointsKernel(const unsigned int maxKeypoints,
                           const unsigned int highThres,
                           const unsigned int lowThres,
                           const PtrStepSz<unsigned char> image,
                           PtrStepSz<int> scoreMat,
                           KeyPoint *pKeypts)
{
    // set smem data types
    __shared__ int smem_warp_scan[32]; // for warp-level exclusive scan

    __shared__ int smem_block_count;

    cg::thread_block cta = cg::this_thread_block();

    const uint warp_id = cta.thread_rank() / warpSize;
    const uint block_warp_size = cta.size() / warpSize;

    const int j = threadIdx.x + blockIdx.x * blockDim.x + 3;
    const int i = (threadIdx.y + blockIdx.y * blockDim.y) * 4 + 3;

    bool isKeyPt[4] = {false, false, false, false};

    if (cta.thread_rank() == 0)
    {
        smem_block_count = 0;
    }

    cta.sync();

    for (int t = 0; t < 4; ++t)
    {
        if (i+t < image.rows-3 &&
            j < image.cols-3)
        {
            isKeyPt[t] = isKeyPoint2(image, i+t, j, highThres, scoreMat);
        }
    }

    cta.sync();

    for (int t = 0; t < 4; ++t)
    {
        short2 loc = make_short2(j, i+t);
        int score = scoreMat(loc.y, loc.x);

        bool findKeyPt = (isKeyPt[t] && isMax(loc));

        cg::coalesced_group active = cg::coalesced_threads();

        uint mask = active.ballot(findKeyPt);

        int total = __popc(mask);

        // if (active.thread_rank() == 0)
        //     smem_warp_scan[warp_id] = total;

        // cta.sync();

        // if (warp_id == 0)
        // {
        //     int warp_old = 0;

        //     // inclusive scan
        //     int warp_raw = (active.thread_rank() < block_warp_size) ? smem_warp_scan[active.thread_rank()] : 0;
        //     int warp_sum = warp_raw;

        //     uint valid_sum_mask = active.ballot(warp_sum > 0);

        //     if (__popc(valid_sum_mask) > 0)
        //     {
        //     for (int i = 1; i < block_warp_size; i*=2)
        //     {
        //         int prev = active.shfl_up(warp_sum, i);

        //         if (active.thread_rank() >= i) warp_sum += prev;
        //     }

        //     // atomic add based on shared memory
        //     if (active.thread_rank() == (block_warp_size-1))
        //     {
        //         warp_old = smem_block_count;

        //         smem_block_count += warp_sum;
        //     }

        //     // broadcast
        //     warp_old = active.shfl(warp_old, block_warp_size-1);
        //     }

        //     // convert to exclusive scan
        //     warp_sum -= warp_raw;

        //     smem_warp_scan[active.thread_rank()] = warp_sum + warp_old;
        // }

        isKeyPt[t] = false;
    }
}

GpuFast::GpuFast(int hThres, int lThres, int maxKeypoints)
    : mHighThres(hThres), mLowThres(lThres), mMaxKeypoints(maxKeypoints)
{
    checkCudaErrors(cudaStreamCreate(&mCudaStream));
    mCvStream = StreamAccessor::wrapStream(mCudaStream);

    checkCudaErrors(cudaMalloc(&mpKeyptsDevice, sizeof(KeyPoint) * mMaxKeypoints));
}

GpuFast::~GpuFast()
{
    mCvStream.~Stream();
    checkCudaErrors(cudaFree(mpKeyptsDevice));
    checkCudaErrors(cudaStreamDestroy(mCudaStream));
}

void GpuFast::detect(InputArray img,
                     std::vector<KeyPoint> &keypoints)
{
    this->detectAsync(img);

    this->joinDetectAsync(keypoints);
}

void GpuFast::detectAsync(InputArray img)
{
    const GpuMat image = img.getGpuMat();

    if (scoreMat.empty())
    {
        scoreMat = GpuMat(image.size(), CV_32SC1)
    }

    scoreMat.setTo(Scalar::all(0), mCvStream);

    dim3 dimBlock(32, 8);
    dim3 dimGrid(divUp(image.cols, dimBlock.x), divUp(image.rows, dimBlock.y * 4));

    // Use 7x7 kernel to detect FAST and Harris feature (the reason of the +3/-3 and *4/+4 tricks)
    detectKeypointsKernel<<<dimGrid, dimBlock, 0, mCudaStream>>>(
        mMaxKeypoints,
        mHighThres,
        mLowThres,
        image,
        scoreMat,
        mpKeyptsDevice
    );
    checkCudaErrors(cudaGetLastError());
}

} // namespace cuda
} // namespace ORB_SLAM2