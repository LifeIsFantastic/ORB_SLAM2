/**
* This file is an extension of ORB-SLAM2.
*/

#ifndef FPSWATCH_H
#define FPSWATCH_H

namespace ORB_SLAM2
{
class FpsWatch
{
public:
    FpsWatch() {}

    // Initialize the checkpoints and frame counter
    void init(const double &timeStamp);

    // Return true if there is a fresh count
    bool update(const double &timeStamp, int &fpsOut);

private:
    double mdCheckPoint; // in seconds

    int mnFrameCount;
};

} // namespace ORB_SLAM2

#endif // FPSWATCH_H