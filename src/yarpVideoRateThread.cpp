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
 * @file yarpVideoRateThreadRatethread.cpp
 * @brief Implementation of the eventDriven thread (see yarpVideoRateThreadRatethread.h).
 */

#include "../include/iCub/yarpVideoRateThread.h"
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 50 //ms

//********************interactionEngineRatethread******************************************************

yarpVideoRateThread::yarpVideoRateThread(std::string t_videoFilePath) : RateThread(THRATE) {
    robot = "icub";
    this->videoPath = t_videoFilePath;
    changedVideo = false;
}

yarpVideoRateThread::yarpVideoRateThread(string _robot, std::string t_videoFilePath) : RateThread(THRATE) {
    robot = _robot;
    this->videoPath = t_videoFilePath;
}

yarpVideoRateThread::~yarpVideoRateThread() {
    // do nothing
}

bool yarpVideoRateThread::threadInit() {


    if (!outputVideoPort.open(getName("/video:o").c_str())) {
        std::cout << ": unable to open port /video:o " << std::endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!loadVideo()) {
        yError("Unable to load video in memory");
        return false;
    }

    this->videoFPS = m_capVideo->get(CV_CAP_PROP_FPS);

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void yarpVideoRateThread::setName(string str) {
    this->name = str;
}


std::string yarpVideoRateThread::getName(const char *p) {
    string str(name);
    str.append(p);
    return str;
}

void yarpVideoRateThread::setInputPortName(string InpPort) {

}

void yarpVideoRateThread::run() {


    if (outputVideoPort.getOutputCount() > 0) {
        cv::Mat temporaryFrameHolder;

        while (m_capVideo->read(temporaryFrameHolder) && !changedVideo) {

            *m_capVideo >> temporaryFrameHolder;

            if (!temporaryFrameHolder.empty()) {
                IplImage temporaryIplFrame = (IplImage) temporaryFrameHolder;
                yarp::sig::ImageOf<yarp::sig::PixelBgr> *processingRgbImageBis = &outputVideoPort.prepare();
                processingRgbImageBis->resize(temporaryIplFrame.width, temporaryIplFrame.height);
                processingRgbImageBis->wrapIplImage(&temporaryIplFrame); //temp_r_ipl
                outputVideoPort.write();


                double waitTime = (1.0 + readingTimeFrame) / videoFPS;
                SystemClock::delaySystem(waitTime);
            }


        }

        if(changedVideo){
            loadVideo();
            changedVideo = false;
        }

        m_capVideo->set(CV_CAP_PROP_POS_MSEC, 0);

    }


}


void yarpVideoRateThread::threadRelease() {
    // nothing

}


void yarpVideoRateThread::setVideoFPS(double t_fps) {
    this->videoFPS = t_fps;

}

void yarpVideoRateThread::setVideoPath(std::string t_videoPath) {
    this->videoPath = t_videoPath;
    changedVideo = true;


}

bool yarpVideoRateThread::loadVideo() {

    m_capVideo = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(this->videoPath)); // open a video file

    if (!m_capVideo->isOpened())  // check if succeeded
    {
        yError(" file  %s not found or could not be opened", this->videoPath.c_str());
        return false;
    }


    computeReadingTime();
    return true;
}

double yarpVideoRateThread::computeReadingTime() {

    clock_t startTime = clock();

    cv::Mat temporaryFrameHolder;

    *m_capVideo >> temporaryFrameHolder;

    IplImage temporaryIplFrame = (IplImage) temporaryFrameHolder;
    yarp::sig::ImageOf<yarp::sig::PixelBgr> *processingRgbImageBis = &outputVideoPort.prepare();
    processingRgbImageBis->resize(temporaryIplFrame.width, temporaryIplFrame.height);
    processingRgbImageBis->wrapIplImage(&temporaryIplFrame); //temp_r_ipl


    clock_t endTime = clock();

    readingTimeFrame = ( (endTime - startTime) / (CLOCKS_PER_SEC / 1000) ) / 1000;

    return readingTimeFrame;
}


