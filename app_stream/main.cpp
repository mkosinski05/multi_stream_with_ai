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
#include <queue>
#include <mutex>
#include <condition_variable>

#include "util.h"
#include "media.h"
#include "stream.h"

#define NUM_PIPELINES 2

#define CAMERA_WIDTH    1920
#define CAMERA_HEIGHT   1080


/* Shared data between OMX's callbacks */
omx_data_t omx_data;

/* Queues for buffers of input and output ports */
queue_t in_queue;
queue_t out_queue;

/* Data for threads */
in_data_t  in_data;
out_data_t out_data;


void * thread_input(void * p_param);
void * thread_output(void * p_param); 
void * thread_infer(void * p_param); 

/* Mutexes for condition variables */
pthread_mutex_t mut_in;
pthread_mutex_t mut_out;

/* Condition variables */
pthread_cond_t cond_in_available;
pthread_cond_t cond_out_available;

bool g_infThreadIsCreated = false;

volatile sig_atomic_t g_int_signal = 0;


void sigint_handler(int signum, siginfo_t * p_info, void * p_ptr);

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
    float cap_time = 0;

    sample = gst_app_sink_pull_sample(appsink);
    if (sample) {
        buffer = gst_sample_get_buffer(sample);
        timespec_get(&data->end_time, TIME_UTC);
        cap_time = (float)((timedifference_msec(data->start_time, data->end_time)));
        timespec_get(&data->start_time, TIME_UTC);
        g_print("ID: %X,\tCapture : %f\n", data->thread_id, cap_time);

        if (buffer) {
            gst_buffer_ref(buffer);  // Increment ref count to ensure buffer is valid during processing

            // Add the buffer to the queue
            {
                std::lock_guard<std::mutex> lock(data->queue_mutex);
                data->buffer_queue.push(buffer);
            }
            data->queue_cond.notify_all();
        }
        gst_sample_unref(sample);
    }
    return GST_FLOW_OK;
}

void* pipeline_thread(void* arg) {
    PipelineData* data = (PipelineData*) arg;
    GstMessage *msg;

    // Save the thread ID of the setup thread
    data->setup_thread_id = pthread_self();

    // Start playing the pipeline
    GstStateChangeReturn ret = gst_element_set_state(data->pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        g_printerr("Unable to set the pipeline %s to the playing state.\n", data->pipeline_name);
        gst_object_unref(data->pipeline);
        data->running = 0;
        return NULL;
    }

    g_print("Pipeline %s setup thread started: %lu\n", data->pipeline_name, data->setup_thread_id);

    /* Threads */
    pthread_t thread_in_id;
    pthread_t thread_out_id;
    pthread_t thread_inf_id;

    if ( !g_infThreadIsCreated ) {
        if (pthread_create(&thread_inf_id, NULL, thread_infer, data) != 0) {
            g_printerr("Failed to create processing thread for pipeline %s.\n", data->pipeline_name);
            return NULL;
        }
        g_infThreadIsCreated = true;
    }
#if 1
    // Start processing thread
    data->in_data = &in_data;
    data->out_data = &out_data;
    if (pthread_create(&thread_in_id, NULL, thread_input, data) != 0) {
        g_printerr("Failed to create processing thread for pipeline %s.\n", data->pipeline_name);
        return NULL;
    }

    data->in_data = &in_data;
    /* Set Unique output filename for each gst pipline */
    data->out_filename = "out_h265_" + std::to_string(data->setup_thread_id) + ".264";
    data->out_data = &out_data;
    if (pthread_create(&thread_out_id, NULL, thread_output, data) != 0) {
        g_printerr("Failed to create processing thread for pipeline %s.\n", data->pipeline_name);
        return NULL;
    }
#endif

    // Create and run the main loop for this pipeline
    data->main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(data->main_loop);

    // Stop processing thread
    {
        std::lock_guard<std::mutex> lock(data->queue_mutex);
        data->running = 0;
    }
    data->queue_cond.notify_all();
    pthread_join(thread_in_id, NULL);
    pthread_join(thread_out_id, NULL);

    // Free resources
    gst_element_set_state(data->pipeline, GST_STATE_NULL);
    gst_object_unref(data->pipeline);
    g_main_loop_unref(data->main_loop);
    return NULL;
}

int main(int argc, char *argv[]) {

    /* Interrupt signal */
    struct sigaction sig_act;

    /* NV12 buffers */
    mmngr_buf_t * p_nv12_bufs = NULL;

    /* YUYV buffer */
    mmngr_buf_t * p_yuyv_buf = NULL;

    /* Handle of media component */
    OMX_HANDLETYPE handle;

    /* Callbacks used by media component */
    OMX_CALLBACKTYPE callbacks =
    {
    .EventHandler    = omx_event_handler,
    .EmptyBufferDone = omx_empty_buffer_done,
    .FillBufferDone  = omx_fill_buffer_done
    };

    /* Buffers for input and output ports */
    OMX_BUFFERHEADERTYPE ** pp_in_bufs  = NULL;
    OMX_BUFFERHEADERTYPE ** pp_out_bufs = NULL;

    /**************************************************************************
     *                STEP 1: SET UP INTERRUPT SIGNAL HANDLER                 *
     **************************************************************************/

    sig_act.sa_sigaction = sigint_handler;
    sig_act.sa_flags     = SA_SIGINFO | SA_RESTART;

    sigaction(SIGINT,  &sig_act, NULL);
    sigaction(SIGTERM, &sig_act, NULL);
    sigaction(SIGQUIT, &sig_act, NULL);

    /**************************************************************************
     *               STEP 4: USE MMNGR TO ALLOCATE NV12 BUFFERS               *
     **************************************************************************/

    p_nv12_bufs = mmngr_alloc_nv12_dmabufs(NV12_BUFFER_COUNT,
                                           NV12_FRAME_SIZE_IN_BYTES);
    assert(p_nv12_bufs != NULL);
    
    p_yuyv_buf = mmngr_alloc_nv12_dmabufs(YUYV_BUFFER_COUNT,
                                           YUYV_FRAME_SIZE_IN_BYTES);
    assert(p_yuyv_buf != NULL);

    /**************************************************************************
     *                         STEP 5: SET UP OMX IL                          *
     **************************************************************************/

    /* Initialize OMX IL core */
    assert(OMX_Init() == OMX_ErrorNone);

    /* Locate Renesas's H.264 encoder.
     * If successful, the component will be in state LOADED */
    assert(OMX_ErrorNone == OMX_GetHandle(&handle,
                                          RENESAS_VIDEO_ENCODER_NAME,
                                          (OMX_PTR)&omx_data, &callbacks));

    /* Print role of the component to console */
    omx_print_mc_role(handle);

    /* Configure input port */
    assert(omx_set_in_port_fmt(handle,
                               FRAME_WIDTH_IN_PIXELS, FRAME_HEIGHT_IN_PIXELS,
                               OMX_COLOR_FormatYUV420SemiPlanar, FRAMERATE));

    assert(omx_set_port_buf_cnt(handle, 0, NV12_BUFFER_COUNT));

    /* Configure output port */
    assert(omx_set_out_port_fmt(handle, H264_BITRATE, OMX_VIDEO_CodingAVC));

    assert(omx_set_port_buf_cnt(handle, 1, H264_BUFFER_COUNT));

    /* Transition into state IDLE */
    assert(OMX_ErrorNone == OMX_SendCommand(handle,
                                            OMX_CommandStateSet,
                                            OMX_StateIdle, NULL));

    /**************************************************************************
     *                STEP 6: ALLOCATE BUFFERS FOR INPUT PORT                 *
     **************************************************************************/

    pp_in_bufs = omx_use_buffers(handle, 0, p_nv12_bufs, NV12_BUFFER_COUNT);
    assert(pp_in_bufs != NULL);

    /* Create queue from these buffers */
    in_queue = queue_create_full(pp_in_bufs, NV12_BUFFER_COUNT,
                                 sizeof(OMX_BUFFERHEADERTYPE *));

    /**************************************************************************
     *                STEP 7: ALLOCATE BUFFERS FOR OUTPUT PORT                *
     **************************************************************************/

    pp_out_bufs = omx_alloc_buffers(handle, 1);
    assert(pp_out_bufs != NULL);

    /* Create empty queue whose size is equal to 'pp_out_bufs' */
    out_queue = queue_create_empty(H264_BUFFER_COUNT,
                                   sizeof(OMX_BUFFERHEADERTYPE *));

    /* Wait until the component is in state IDLE */
    omx_wait_state(handle, OMX_StateIdle);

    /**************************************************************************
     *             STEP 8: CREATE MUTEXES AND CONDITION VARIABLES             *
     **************************************************************************/

    pthread_mutex_init(&mut_in, NULL);
    pthread_mutex_init(&mut_out, NULL);

    pthread_cond_init(&cond_in_available, NULL);
    pthread_cond_init(&cond_out_available, NULL);

    /**************************************************************************
     *          STEP 9: PREPARE SHARED DATA BETWEEN OMX'S CALLBACKS           *
     **************************************************************************/
    omx_data.p_in_queue           = &in_queue;
    omx_data.p_out_queue          = &out_queue;
    omx_data.p_mut_in             = &mut_in;
    omx_data.p_mut_out            = &mut_out;
    omx_data.p_cond_in_available  = &cond_in_available;
    omx_data.p_cond_out_available = &cond_out_available;

    /**************************************************************************
     *            STEP 10: MAKE OMX READY TO SEND/RECEIVE BUFFERS             *
     **************************************************************************/

    /* Transition into state EXECUTING */
    assert(OMX_ErrorNone == OMX_SendCommand(handle,
                                            OMX_CommandStateSet,
                                            OMX_StateExecuting, NULL));
    omx_wait_state(handle, OMX_StateExecuting);

    /* Send buffers in 'pp_out_bufs' to output port */
    assert(omx_fill_buffers(handle, pp_out_bufs, H264_BUFFER_COUNT));

    /**************************************************************************
     *                 STEP 12: PREPARE DATA FOR INPUT THREAD                 *
     **************************************************************************/

    //in_data.cam_fd           = cam_fd;
    in_data.p_yuyv_bufs      = p_yuyv_buf;
    in_data.p_nv12_bufs      = p_nv12_bufs;
    in_data.handle           = handle;
    in_data.pp_bufs          = pp_in_bufs;
    in_data.p_queue          = &in_queue;
    in_data.p_mutex          = &mut_in;
    in_data.p_cond_available = &cond_in_available;

    /**************************************************************************
     *                STEP 13: PREPARE DATA FOR OUTPUT THREAD                 *
     **************************************************************************/

    out_data.handle           = handle;
    out_data.p_queue          = &out_queue;
    out_data.p_mutex          = &mut_out;
    out_data.p_cond_available = &cond_out_available;


    //inf_data.cam_fd           = cam_fd;
    //inf_data.p_yuyv_bufs      = p_yuyv_bufs;

    /**************************************************************************
     *                          Initialize GSTREAMER    
     **************************************************************************/
    gst_init(&argc, &argv);

    PipelineData pipelines[NUM_PIPELINES];
    gchar* pipeline_str[NUM_PIPELINES] = {
        "v4l2src device=/dev/video0 ! videoconvert ! appsink name=sink0",
        "v4l2src device=/dev/video1 ! videoconvert ! appsink name=sink1"
    };

     /**************************************************************************
 *                          QUERY V4L2 for all MIPI Cameras     
     **************************************************************************/
    int i = 0;
    std::vector<videodev_t> cameras = query_device_status("RZG2L_CRU");

     /**************************************************************************
 *                          LOOP THROUGH EACH CAMERA     
     **************************************************************************/
    for ( auto& camera : cameras) { 

        /**************************************************************************
 *                          Initialize Media      
        **************************************************************************/
        media_init( &camera, CAMERA_WIDTH, CAMERA_HEIGHT);
        
        pipelines[i].pipeline = gst_parse_launch(pipeline_str[i], NULL);
        if (!pipelines[i].pipeline) {
            g_printerr("Failed to create pipeline %d.\n", i);
            return -1;
        }
        /* Set the GSTREAMER PIPLINE NAME */
        pipelines[i].pipeline_name = g_strdup_printf("pipeline%d", i);
        gst_element_set_name(pipelines[i].pipeline, pipelines[i].pipeline_name);

        /* GET THE GSTREAMER SINK */
        pipelines[i].sink = gst_bin_get_by_name(GST_BIN(pipelines[i].pipeline), g_strdup_printf("sink%d", i));
        if (!pipelines[i].sink) {
            g_printerr("Failed to get appsink from pipeline %d.\n", i);
            gst_object_unref(pipelines[i].pipeline);
            return -1;
        }

        /* ENABLE GSTREAMER PIPLINE SINK SIGNAL */
        g_object_set(pipelines[i].sink, "emit-signals", TRUE, "sync", FALSE, NULL);

        // Connect the callback to the "new-sample" signal
        g_signal_connect(pipelines[i].sink, "new-sample", G_CALLBACK(new_sample_callback), &pipelines[i]);

        pipelines[i].bus = gst_element_get_bus(pipelines[i].pipeline);
        pipelines[i].running = 1;

        /* 
        *   CREATE THREAD FOR EACH CAMAERA 
        *   PASS THE PIPELINE INFORMATION TO THE THREAD ARGUMENT
        */
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
    
    /**************************************************************************
     *                          STEP 16: CLEAN UP OMX                         *
     **************************************************************************/

    pthread_mutex_destroy(&mut_in);
    pthread_mutex_destroy(&mut_out);

    pthread_cond_destroy(&cond_in_available);
    pthread_cond_destroy(&cond_out_available);

    /* Transition into state IDLE */
    assert(OMX_ErrorNone == OMX_SendCommand(handle,
                                            OMX_CommandStateSet,
                                            OMX_StateIdle, NULL));
    omx_wait_state(handle, OMX_StateIdle);

    /* Transition into state LOADED */
    assert(OMX_ErrorNone == OMX_SendCommand(handle,
                                            OMX_CommandStateSet,
                                            OMX_StateLoaded, NULL));

    /* Release buffers and buffer headers from the component.
     *
     * The component shall free only the buffer headers if it allocated only
     * the buffer headers ('OMX_UseBuffer').
     *
     * The component shall free both the buffers and the buffer headers if it
     * allocated both the buffers and buffer headers ('OMX_AllocateBuffer') */
    omx_dealloc_all_port_bufs(handle, 0, pp_in_bufs);
    omx_dealloc_all_port_bufs(handle, 1, pp_out_bufs);

    queue_delete(&in_queue);
    queue_delete(&out_queue);

    /* Wait until the component is in state LOADED */
    omx_wait_state(handle, OMX_StateLoaded);

    /* Free the component's handle */
    assert(OMX_FreeHandle(handle) == OMX_ErrorNone);

    /* Deinitialize OMX IL core */
    assert(OMX_Deinit() == OMX_ErrorNone);

    /* Deallocate NV12 buffers */
    mmngr_dealloc_nv12_dmabufs(p_nv12_bufs, NV12_BUFFER_COUNT);


    return 0;
}

/**************************** FOR SIGNAL HANDLING *****************************/

void sigint_handler(int signum, siginfo_t * p_info, void * p_ptr)
{
    /* Mark parameters as unused */
    UNUSED(p_ptr);
    UNUSED(p_info);
    UNUSED(signum);

    g_int_signal = 1;
}