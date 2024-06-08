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

#include <vector>
#include <iostream>
#include <map>
#include <string>


#define NUM_PIPELINES 2

#define CAMERA_WIDTH    1920
#define CAMERA_HEIGHT   1080

typedef struct {
    GstElement *pipeline;
    GstElement *sink;
    GstBus *bus;
    pthread_t thread_id;
    int running;
    char *pipeline_name;
    GMainLoop *main_loop;
    struct timespec start_time;
    struct timespec end_time;
} PipelineData;

typedef struct _videodev_t{
    std::string dev_name;
    std::string media_name;
    std::string channel;
    std::string id;
} videodev_t;

/*****************************************
* Function Name : timedifference_msec
* Description   : compute the time differences in ms between two moments
* Arguments     : t0 = start time
*                 t1 = stop time
* Return value  : the time difference in ms
******************************************/
static double timedifference_msec(struct timespec t0, struct timespec t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}


// Callback function for new samples
static GstFlowReturn new_sample_callback(GstAppSink *appsink, gpointer user_data) {
    PipelineData* data = (PipelineData*) user_data;
    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo info;
    float cap_time = 0;

    sample = gst_app_sink_pull_sample(appsink);
    if (sample) {
        buffer = gst_sample_get_buffer(sample);
        gst_buffer_map(buffer, &info, GST_MAP_READ);
        
        timespec_get(&data->end_time, TIME_UTC);
        cap_time = (float)((timedifference_msec(data->start_time, data->end_time)));
        timespec_get(&data->start_time, TIME_UTC);
        
        

        // Process the buffer data (info.data, info.size)
        //g_print("Pipeline %s (Thread %lu): Buffer size: %zu\n", data->pipeline_name, pthread_self(), info.size);
        g_print("Capture : %f\n", cap_time);

        // Cleanup
        gst_buffer_unmap(buffer, &info);
        gst_sample_unref(sample);
    }
    return GST_FLOW_OK;
}

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
int init_camera ( videodev_t * video, int width, int height) {

    char cmd[100];

    const char* commands[4] =
    {
        "media-ctl -d %s -r",
        "media-ctl -d %s -l \"\'rzg2l_csi2 %s.csi2%s\':1 -> \'CRU output\':0 [1]\"",
        "media-ctl -d %s -V \"\'imx462 0-001f\':0 [fmt:UYVY8_2X8/%dx%d field:none]\"",
        "media-ctl -d %s -V \"\'rzg2l_csi2 %s.csi2%s\':1 [fmt:UYVY8_2X8/%dx%d field:none]\""
    };

    printf("%s\n", video->dev_name.c_str());
    sprintf(cmd, commands[0],video->media_name.c_str());
    printf("%s\n",cmd);
    system(cmd);
    sprintf(cmd, commands[1],video->media_name.c_str(), video->id.c_str(), video->channel.c_str());
    printf("%s\n",cmd);
    system(cmd);
    sprintf(cmd, commands[2],video->media_name.c_str(), width, height);
    printf("%s\n",cmd);
    system(cmd);
    sprintf(cmd, commands[3],video->media_name.c_str(), video->id.c_str(), video->channel.c_str(), width, height);
    printf("%s\n",cmd);
    system(cmd);
}
   

void* pipeline_thread(void* arg) {
    PipelineData* data = (PipelineData*) arg;
    GstMessage *msg;

    // Start playing the pipeline
    GstStateChangeReturn ret = gst_element_set_state(data->pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline %s to the playing state.\n", data->pipeline_name);
        gst_object_unref(data->pipeline);
        data->running = 0;
        return NULL;
    }

    g_print("Pipeline %s thread started: %lu\n", data->pipeline_name, pthread_self());

    // Create and run the main loop for this pipeline
    data->main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(data->main_loop);

    // Free resources
    gst_element_set_state(data->pipeline, GST_STATE_NULL);
    gst_object_unref(data->pipeline);
    g_main_loop_unref(data->main_loop);
    return NULL;
}

int main(int argc, char *argv[]) {
    gst_init(&argc, &argv);

    PipelineData pipelines[NUM_PIPELINES];
    gchar* pipeline_str[NUM_PIPELINES] = {
        "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,framerate=60/1 ! appsink name=sink0",
        "v4l2src device=/dev/video1 ! videoconvert ! video/x-raw,framerate=60/1 ! appsink name=sink1"
    };

    int i = 0;
    std::vector<videodev_t> cameras = query_device_status("RZG2L_CRU");
    for ( auto& camera : cameras) { 


        init_camera( &camera, CAMERA_WIDTH, CAMERA_HEIGHT);
        
        pipelines[i].pipeline = gst_parse_launch(pipeline_str[i], NULL);
        if (!pipelines[i].pipeline) {
            g_printerr("Failed to create pipeline %d.\n", i);
            return -1;
        }

        pipelines[i].pipeline_name = g_strdup_printf("pipeline%d", i);
        gst_element_set_name(pipelines[i].pipeline, pipelines[i].pipeline_name);

        pipelines[i].sink = gst_bin_get_by_name(GST_BIN(pipelines[i].pipeline), g_strdup_printf("sink%d", i));
        if (!pipelines[i].sink) {
            g_printerr("Failed to get appsink from pipeline %d.\n", i);
            gst_object_unref(pipelines[i].pipeline);
            return -1;
        }

        g_object_set(pipelines[i].sink, "emit-signals", TRUE, "sync", FALSE, NULL);

        // Connect the callback to the "new-sample" signal
        g_signal_connect(pipelines[i].sink, "new-sample", G_CALLBACK(new_sample_callback), &pipelines[i]);

        pipelines[i].bus = gst_element_get_bus(pipelines[i].pipeline);
        pipelines[i].running = 1;

        if (pthread_create(&pipelines[i].thread_id, NULL, pipeline_thread, &pipelines[i]) != 0) {
            g_printerr("Failed to create thread for pipeline %d.\n", i);
            return -1;
        }
        i++;
        
    }

    for (int i = 0; i < NUM_PIPELINES; i++) {
        pthread_join(pipelines[i].thread_id, NULL);
        g_free(pipelines[i].pipeline_name);
        
    }

    return 0;
}
