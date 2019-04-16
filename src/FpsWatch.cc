#include "FpsWatch.h"

namespace ORB_SLAM2
{
void FpsWatch::init(const double &timeStamp)
{
    mdCheckPoint = timeStamp;

    mnFrameCount = 0;
}

bool FpsWatch::update(const double &timeStamp, int &fpsOut)
{
    if (timeStamp - mdCheckPoint > 1)
    {
        fpsOut = (++mnFrameCount);

        this->init(timeStamp);

        return true;
    }
    else
    {
        fpsOut = (++mnFrameCount);

        return false;
    }
}

} // namespace ORB_SLAM2