//#ifndef __DEFINES_H__
//#define __DEFINES_H__

#include <stdio.h>
#include <assert.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "omx.h"
#include "util.h"
#include "v4l2.h"
#include "mmngr.h"
#include "queue.h"

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/gstmessage.h>


/******************************************************************************
 *                              MACRO VARIABLES                               *
 ******************************************************************************/

#define FONT_FILE "LiberationSans-Regular.ttf"

#define FRAME_WIDTH_IN_PIXELS  1920

#define FRAME_HEIGHT_IN_PIXELS 1080

#define YUYV_FRAME_WIDTH_IN_BYTES (FRAME_WIDTH_IN_PIXELS * 2)

#define YUYV_FRAME_SIZE_IN_BYTES (FRAME_WIDTH_IN_PIXELS  * \
                                  FRAME_HEIGHT_IN_PIXELS * 2)

#define NV12_FRAME_WIDTH_IN_BYTES (FRAME_WIDTH_IN_PIXELS)

#define NV12_FRAME_SIZE_IN_BYTES (FRAME_WIDTH_IN_PIXELS  * \
                                  FRAME_HEIGHT_IN_PIXELS * 1.5f)

#define FRAMERATE 30 /* FPS */
/********************************** FOR STREAMS *******************************/
#define NUM_PIPELINES 2

/********************************** FOR V4L2 **********************************/

/* The sample app is tested OK with:
 *   - Logitech C270 HD Webcam.
 *   - Logitech C920 HD Pro Webcam.
 *   - Logitech C930e Business Webcam.
 *   - Logitech BRIO Ultra HD Pro Business Webcam */
#define USB_CAMERA_FD "/dev/video0"

/* The number of buffers to be allocated for the camera */
#define YUYV_BUFFER_COUNT 1

#define CAM_COUNT 2


/********************************** FOR OMX ***********************************/

/* The number of buffers to be allocated for input port of media component */
#define NV12_BUFFER_COUNT 2

/* The number of buffers to be allocated for output port of media component */
#define H264_BUFFER_COUNT 2


/* The bitrate is related to the quality of output file and compression level
 * of video encoder. For example:
 *   - With 1 Mbit/s, the encoder produces ~1.2 MB of data for 10-second video.
 *   - With 5 Mbit/s, the encoder produces ~6 MB of data for 10-second video
 *                                          and the quality should be better */
#define H264_BITRATE 5000000 /* 5 Mbit/s */

#define H264_FILE_NAME "out-h264-640x480.264"


/******************************** FOR THREADS *********************************/

/* Try to call 'OMX_EmptyThisBuffer' whenerver there is an available
 * input buffer and interrupt signal is not raised.
 *
 * Warning: Only create 1 thread for this routine */
void * thread_input(void * p_param);

/* Try to call 'OMX_FillThisBuffer' whenever there is an available
 * output buffer and End-of-Stream event is not raised.
 *
 * Warning: Only create 1 thread for this routine */
void * thread_output(void * p_param);

void * thread_infer(void * p_param);


/******************************************************************************
 *                           STRUCTURE DEFINITIONS                            *
 ******************************************************************************/

/********************************** FOR VSPM ISU ***********************************/
typedef struct {
	unsigned long job_id;
	long ercd;

	pthread_mutex_t mutex;
	pthread_cond_t cond;
} vspm_tp_cb_info_t;

/********************************** PERFORMACE ********************************/
typedef struct _perf_info_t {
    unsigned long job_id;
    long ercd;

    float convert_time;

    struct timespec encode_start;
    struct timespec encode_end;
    float encode_time;

} perf_info_t;
/********************************** FOR OMX ***********************************/

/* This structure is shared between OMX's callbacks */
typedef struct
{
    queue_t * p_in_queue;
    queue_t * p_out_queue;

    pthread_mutex_t * p_mut_in;
    pthread_mutex_t * p_mut_out;

    pthread_cond_t * p_cond_in_available;
    pthread_cond_t * p_cond_out_available;

} omx_data_t;

/******************************** FOR THREADS *********************************/


/* This structure is for input thread */
typedef struct
{
    /* Camera */
    int cam_fd;

    /* YUYV buffers */
    mmngr_buf_t * p_yuyv_bufs;

    /* NV12 buffers */
    mmngr_buf_t * p_nv12_bufs;

    /* Handle of media component */
    OMX_HANDLETYPE handle;

    /* Buffers for input port */
    OMX_BUFFERHEADERTYPE ** pp_bufs;

    /* The queue contains some buffers in 'pp_bufs' ready to be overlaid and
     * sent to input port */
    queue_t * p_queue;

    /* The mutex structure is used to synchronize 'wait' and 'signal' events
     * of 'p_cond_available' */
    pthread_mutex_t * p_mutex;

    /* When signaled, the condition variable confirms there is a possible
     * buffer in 'p_queue' */
    pthread_cond_t * p_cond_available;

} in_data_t;

/* This structure is for output thread */
typedef struct
{
    /* Handle of media component */
    OMX_HANDLETYPE handle;

    /* This queue contains some buffers received from output port and ready
     * to be written to output file */
    queue_t * p_queue;

    /* The mutex structure is used to synchronize 'wait' and 'signal' events
     * of 'p_cond_available' */
    pthread_mutex_t * p_mutex;

    /* When signaled, the condition variable confirms there is a possible
     * buffer in 'p_queue' */
    pthread_cond_t * p_cond_available;

} out_data_t;

typedef struct _inf_data_t {


    /* YUYV buffers */
    mmngr_buf_t * p_yuyv_bufs;
} inf_data_t;

/********************************** GSTREAMER PIPLINE *****************************************/

typedef struct {
    GstElement *pipeline;
    GstElement *sink;
    GstBus *bus;
    pthread_t thread_id;
    int running;
    char *pipeline_name;
    GMainLoop *main_loop;
    pthread_t setup_thread_id;
    std::queue<GstBuffer*> buffer_queue;
    std::mutex queue_mutex;
    std::condition_variable queue_cond;
    /* Camera Capture time */
    struct timespec start_time;
    struct timespec end_time;
    /* OMX Decode */
    in_data_t *in_data;
    out_data_t *out_data;
    inf_data_t *inf_data;

    FILE * p_h264_fd;
    std::string out_filename;

    perf_info_t metrics;
    bool processing_thread = false;

} PipelineData;
/********************************** FOR OMX ***********************************/

/* The method is used to notify the application when an event of interest
 * occurs within the component.
 *
 * Events are defined in the 'OMX_EVENTTYPE' enumeration.
 * Please see that enumeration for details of what will be returned for
 * each type of event.
 *
 * Callbacks should not return an error to the component, so if an error
 * occurs, the application shall handle it internally.
 *
 * Note: This is a blocking call */
OMX_ERRORTYPE omx_event_handler(OMX_HANDLETYPE hComponent, OMX_PTR pAppData,
                                OMX_EVENTTYPE eEvent, OMX_U32 nData1,
                                OMX_U32 nData2, OMX_PTR pEventData);

/* This method is used to return emptied buffers from an input port back to
 * the application for reuse.
 *
 * This is a blocking call so the application should not attempt to refill
 * the buffers during this call, but should queue them and refill them in
 * another thread.
 *
 * Callbacks should not return an error to the component, so the application
 * shall handle any errors generated internally */
OMX_ERRORTYPE omx_empty_buffer_done(OMX_HANDLETYPE hComponent,
                                    OMX_PTR pAppData,
                                    OMX_BUFFERHEADERTYPE * pBuffer);

/* The method is used to return filled buffers from an output port back to
 * the application for emptying and then reuse.
 *
 * This is a blocking call so the application should not attempt to empty
 * the buffers during this call, but should queue them and empty them in
 * another thread.
 *
 * Callbacks should not return an error to the component, so the application
 * shall handle any errors generated internally */
OMX_ERRORTYPE omx_fill_buffer_done(OMX_HANDLETYPE hComponent,
                                   OMX_PTR pAppData,
                                   OMX_BUFFERHEADERTYPE * pBuffer);

/********************************** FOR VSPM ***********************************/

void vspm_isu_callback ( unsigned long job_id, long result, void *user_data );
//#endif /*__DEFINES_H__*/