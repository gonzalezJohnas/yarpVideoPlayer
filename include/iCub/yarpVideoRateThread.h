// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2017  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: jonas gonzalez
  * email: 
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file yarpVideoRateThread.h
 * @brief Definition of a thread that receives an data from input port and sends it to the output port.
 */


#ifndef _yarpVideoRateThread_RATETHREAD_H_
#define _yarpVideoRateThread_RATETHREAD_H_


#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>

class yarpVideoRateThread : public yarp::os::RateThread {
private:
    bool result;                    //result of the processing

    std::string robot;              // name of the robot
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string name;               // rootname of all the ports opened by this thread

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outputVideoPort;

    // Parameters video
    std::unique_ptr<cv::VideoCapture> m_capVideo;
    double videoFPS, readingTimeFrame;
    std:: string videoPath;
    bool changedVideo;

public:
    /**
    * constructor default
    */
    yarpVideoRateThread(std::string videoFilePath);

    /**
    * constructor 
    * @param robotname name of the robot
    */
    yarpVideoRateThread(std::string robotname, std::string videoFilePath);

    /**
     * destructor
     */
    ~yarpVideoRateThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run();

    /**
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);

    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char *p);

    /**
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /**
     * Set the member varibale videoFPS
     * @param t_fps
     */
    void setVideoFPS(double t_fps);

    /**
     * set the member variable videoPath
     * @param t_videoPath
     */
    void setVideoPath(std::string t_videoPath);

    // Processing functions

    /**
     * Load video by extrating all frames and saved them into /tmp directory
     * @return
     */
    bool loadVideo();

    /**
     * Calcul in seconds how long it's take to extract a frame from the VideoCapture object
     * @return
     */
    double computeReadingTime();

};

#endif  //_yarpVideoRateThread_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

