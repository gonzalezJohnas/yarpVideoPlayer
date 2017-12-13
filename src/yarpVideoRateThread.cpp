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

#include <utility>

#include "../include/iCub/yarpVideoRateThread.h"


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 50 //ms

//********************interactionEngineRatethread******************************************************

yarpVideoRateThread::yarpVideoRateThread(std::string t_videoFilePath) : RateThread(THRATE) {
    robot = "icub";
    this->videoPath = std::move(t_videoFilePath);
    changedVideo = false;
    cropVideo = false;
}

yarpVideoRateThread::yarpVideoRateThread(string _robot, std::string t_videoFilePath) : RateThread(THRATE) {
    robot = std::move(_robot);
    this->videoPath = std::move(t_videoFilePath);
}

yarpVideoRateThread::~yarpVideoRateThread() = default;

bool yarpVideoRateThread::threadInit() {


    if (!outputVideoPort.open(getName("/video:o").c_str())) {
        std::cout << ": unable to open port /video:o " << std::endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!inputYarpviewClickPort.open(getName("/inputClick:i").c_str())) {
        std::cout << ": unable to open port /inputClik:i " << std::endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!loadVideo()) {
        yError("Unable to load video in memory");
        return false;
    }

    this->videoFPS = m_capVideo->get(CV_CAP_PROP_FPS);
    this->processingRgbImageBis = new ImageOf<PixelBgr>();

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void yarpVideoRateThread::setName(string str) {
    this->name = std::move(str);
}


std::string yarpVideoRateThread::getName(const char *p) {
    string str(name);
    str.append(p);
    return str;
}

void yarpVideoRateThread::setInputPortName(string InpPort) {
}

void yarpVideoRateThread::run() {
    double adjustFactor = 0.0;
    int currentFPS = 0;


    if (outputVideoPort.getOutputCount() && m_capVideo->isOpened()) {
        cv::Mat temporaryFrameHolder;
        while (m_capVideo->read(temporaryFrameHolder) && !changedVideo) {

            const double startTime = time(nullptr);
            *m_capVideo >> temporaryFrameHolder;


            if (!temporaryFrameHolder.empty()) {
                temporaryIplFrame = cvCreateImage(cvSize(temporaryFrameHolder.cols, temporaryFrameHolder.rows),
                                                  IPL_DEPTH_8U, 3);
                *temporaryIplFrame = (IplImage) temporaryFrameHolder;

                if (cropVideo) {

                    iplCropFrme = cvCreateImage(cvSize(rectCropedArea.width, rectCropedArea.height), IPL_DEPTH_8U, 3);
                    cvSetImageROI(temporaryIplFrame, rectCropedArea);
                    cvCopy(temporaryIplFrame, iplCropFrme, nullptr);

                    cvResetImageROI(temporaryIplFrame);

                    temporaryIplFrame = iplCropFrme;

                }


                processingRgbImageBis = &outputVideoPort.prepare();
                processingRgbImageBis->resize(temporaryIplFrame->width, temporaryIplFrame->height);
                processingRgbImageBis->wrapIplImage(temporaryIplFrame);


                const auto waitTime = (1.0 + readingTimeFrame) / videoFPS;
                SystemClock::delaySystem(waitTime + adjustFactor);

                const auto timeElapsed = time(nullptr) - startTime;
                ++currentFPS;


                if (timeElapsed == 1) {

                    if (currentFPS > videoFPS) {
                        adjustFactor += 0.01;
                    } else if (currentFPS < videoFPS) {
                        adjustFactor -= 0.01;
                    }

                    currentFPS = 0;

                }

                outputVideoPort.write();


            }


            if (inputYarpviewClickPort.getInputCount()) {
                const Bottle *inputClickBotlle = inputYarpviewClickPort.read(false);
                if (inputClickBotlle != nullptr) {
                    const int xInputClick = inputClickBotlle->get(0).asInt();
                    const int yInputClick = inputClickBotlle->get(1).asInt();

                    processClickCoordinate(xInputClick, yInputClick);
                }

            }
        }


        if (changedVideo) {
            loadVideo();
            changedVideo = false;
        }
        m_capVideo->set(CV_CAP_PROP_POS_MSEC, 0);
    }


}


void yarpVideoRateThread::threadRelease() {
    outputVideoPort.interrupt();
    inputYarpviewClickPort.interrupt();
    m_capVideo.release();
    free(temporaryIplFrame);
    free(iplCropFrme);
}


void yarpVideoRateThread::setVideoFPS(double t_fps) {
    this->videoFPS = t_fps;

}

void yarpVideoRateThread::setVideoPath(std::string t_videoPath) {
    this->videoPath = std::move(t_videoPath);
    changedVideo = true;


}

bool yarpVideoRateThread::loadVideo() {

    m_capVideo = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(this->videoPath)); // open a video file

    if (!m_capVideo->isOpened())  // check if succeeded
    {
        yError(" file  %s not found or could not be opened", this->videoPath.c_str());
        return false;
    }


    cv::Mat temporaryFrameHolder;

    *m_capVideo >> temporaryFrameHolder;

    widthInputVideo = temporaryFrameHolder.cols;
    heightInputVideo = temporaryFrameHolder.rows;

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

    readingTimeFrame = ((endTime - startTime) / (CLOCKS_PER_SEC / 1000)) / 1000;

    m_capVideo->set(CV_CAP_PROP_POS_MSEC, 0);


    return readingTimeFrame;
}

bool yarpVideoRateThread::computeCropArea(int x1, int y1, int x2, int y2) {


    const int width = x2 - x1;
    const int height = y2 - y1;

    rectCropedArea.x = cropVideo ? x1 + rectCropedArea.x : x1;
    rectCropedArea.y = cropVideo ? y1 + rectCropedArea.y : y1;


    if (width > 0 && height > 0 && width < widthInputVideo && height < heightInputVideo) {
        rectCropedArea.width = width;
        rectCropedArea.height = height;
        cropVideo = true;
    } else {
        cropVideo = false;
    }

    return cropVideo;


}

void yarpVideoRateThread::setCropVideo(bool cropVideo) {
    this->cropVideo = cropVideo;
}


void yarpVideoRateThread::processClickCoordinate(const int x, const int y) {

    if (x1Click == -1) {

        x1Click = x;
        y1Click = y;
    } else {
        x2Click = x;
        y2Click = y;

        computeCropArea(x1Click, y1Click, x2Click, y2Click);
        x1Click = y1Click = x2Click = y2Click = -1;
    }
}



