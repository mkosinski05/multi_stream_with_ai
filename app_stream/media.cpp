#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/gstmessage.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/videodev2.h>
#include "media.h"


/*****************************************
* Function Name : xioctl
* Description   : ioctl calling
* Arguments     : fd = V4L2 file descriptor
*                 request = V4L2 control ID defined in videodev2.h
*                 arg = set value
* Return value  : int = output parameter
******************************************/
static int8_t xioctl(int8_t fd, int32_t request, void * arg)
{
    int8_t r;
    do r = ioctl(fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

std::vector<videodev_t> query_device_status(const std::string& device_type) {

    std::vector<videodev_t> media_ports;
    std::string id;

    /* Linux command to be executed */
    const char* command = "v4l2-ctl --list-devices";
    /* Open a pipe to the command and execute it */
    FILE* pipe = popen(command, "r");
    if (!pipe) {
        std::cerr << "[ERROR] Unable to open the pipe." << std::endl;
        return media_ports;
    }
    /* Read the command output line by line */
    char buffer[128];
    bool device_found = false;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string response(buffer);
        

        if (response.find(device_type) != std::string::npos) {
            /* filter search MIPI device_type*/
            device_found = true;
            
            /* Get Device ID */
            std::string token = "platform:";
            std::string delimiter = ".";
            
            size_t start = response.find(token) + token.length();
            size_t end = response.find(delimiter, start);
            /* Save for later use */
            id = response.substr(start, end - start);
            
        } else if (device_found && response.find("/dev/") != std::string::npos) {
            videodev_t device;

            // Save id and set for media control 
            device.id = id;
            device.id[5] = '4'; // Requried for media control

            /* Get dev/video */
            response.erase(response.find("\n")); // Remove newline
            device.dev_name = response;

            /* Next line contains associated /dev/media */
            fgets(buffer, sizeof(buffer), pipe); // read next line
            std::string response2(buffer); 
            response2.erase(response2.find("\n")); // Remove newline
            device.media_name = response2;

            /* Get Media channel number */
            device.channel = response2.back();

            /* Save /dev/video as key and /dev/media as value*/
            media_ports.push_back(device);

        } else if (device_found && response.find("\n") != std::string::npos) {
            device_found = false;  // End of current device listing
        }
    }
    pclose(pipe);
    return media_ports;
}
int media_init ( videodev_t * video, int width, int height) {

    char cmd[100];
/*
    media-ctl -d /dev/media1 -l "'rzg2l_csi2 16010400.csi21':1 -> 'CRU output':0 [1]"
    media-ctl -d /dev/media1 -V "'rzg2l_csi2 16010400.csi21':1 [fmt:UYVY8_2X8/$imx462_res field:none]"
    media-ctl -d /dev/media1 -V "'imx462 1-001f':0 [fmt:UYVY8_2X8/$imx462_res field:none]"
*/
    const char* commands[] =
    {
        "media-ctl -d %s -r",
        "media-ctl -d %s -l \"\'rzg2l_csi2 %s.csi2%s\':1 -> \'CRU output\':0 [1]\"",
        "media-ctl -d %s -V \"\'rzg2l_csi2 %s.csi2%s\':1 [fmt:UYVY8_2X8/%dx%d field:none]\"",
        "media-ctl -d %s -V \"\'imx462 0-001f\':0 [fmt:UYVY8_2X8/%dx%d field:none]\"",
        "media-ctl -d %s -V \"\'imx462 1-001f\':0 [fmt:UYVY8_2X8/%dx%d field:none]\"",
        "media-ctl -d %s -V \"\'imx462 6-001f\':0 [fmt:UYVY8_2X8/%dx%d field:none]\"",
        "media-ctl -d %s -V \"\'imx462 7-001f\':0 [fmt:UYVY8_2X8/%dx%d field:none]\""
    };

    printf("%s\n", video->dev_name.c_str());
    sprintf(cmd, commands[0],video->media_name.c_str());
    printf("%s\n",cmd);
    system(cmd);
    sprintf(cmd, commands[1],video->media_name.c_str(), video->id.c_str(), video->channel.c_str());
    printf("%s\n",cmd);
    system(cmd);
    sprintf(cmd, commands[2],video->media_name.c_str(), video->id.c_str(), video->channel.c_str(), width, height);
    printf("%s\n",cmd);
    system(cmd);
    int n = std::stoi(video->channel.c_str()) + 3;

    sprintf(cmd, commands[n],video->media_name.c_str(), width, height);
    printf("%s\n",cmd);
    system(cmd);
    
}