# YarpVideoPlayer
Simple yarpVideoPlayer that load a video from file and allow to :

    - Change the FPS of the yarpview dynamically
    - Change the video dsplayi on run
    - Crop the video by giving left top corner point and right botton corner

## Dependency
- YARP
- OpenCV

## Yarp Input Port
NONE

## Yarp Output Port
**yarpVideoModule/video:o** :
    Output the video stream loaded

## RPC port
 **set video <path_to_video>** : Change the video to be display by providing absolute path <br>
 **set fps <fps>** : Change the fps of the yarpview <br>
 **set crop <x1> <y1> <x2> <y2>** : Crop the video from Point(x1, y1) to Point(x2, y2) <br>
 **set crop reset** : Reset the size of the video to its original size

## Parameters
### Mandatory
**videoPath** : Absolute path to the video

### Optional
**fps** : Absolute path to the video

**xTopLeft** : x top left corner coordinate of the desired crop area

**yTopLeft** : y top left corner coordinate of the desired crop area

**xBottomRight** : x bottom right corner coordinate of the desired crop area

**yBottomRight** : y top left corner coordinate of the desired crop area

## Run testing
This module was only test on **Linux distribution**

### Example Build && Run
Build the module

    mkdir build && cd build
    ccmake ..
    make install

Run a yarpview module

        yarpview --name /viewer --out /outputClick

Run the yarpVideoModule

        ./yarpVideoModule --videoPath <absolute_path_to_videoFile>

Connect the output video to viewer and viewer output click to yarpVideoModule inputClick

    yarp connect /yarpVideoModule/video:o /viewer
    yarp connect /outputClick /yarpVideoModule/inputClick:i

