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