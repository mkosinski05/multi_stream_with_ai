

#include "stream.h"
#include <opencv2/opencv.hpp>
#include "isu.h"

#include <atomic>
#include <queue>

using namespace cv;

static std::atomic<int> omx_share_processing (0);


extern volatile sig_atomic_t g_int_signal; 


static double timedifference_msec(struct timespec t0, struct timespec t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
}
void yuyv_to_nv12(uint8_t* yuyv, uint8_t* y_plane, uint8_t* uv_plane, int width, int height) {
    int frame_size = width * height;
    uint8_t* y_ptr = y_plane;
    uint8_t* uv_ptr = uv_plane;
    uint8_t* yuyv_ptr = yuyv;

    for (int y = 0; y < height; y += 2) {
        for (int x = 0; x < width; x += 16) {
            // Load 16 pixels (8 YUYV pairs)
            uint8x16x2_t yuyv_data = vld2q_u8(yuyv_ptr);

            // Extract Y components
            uint8x16_t y1 = yuyv_data.val[0];
            uint8x16_t y2 = yuyv_data.val[1];

            // Extract U and V components
            uint8x16_t u = vld1q_u8(yuyv_ptr + 1);
            uint8x16_t v = vld1q_u8(yuyv_ptr + 3);

            uint8x8x2_t uv;
            uv.val[0] = vrhadd_u8(vget_low_u8(u), vget_high_u8(u));
            uv.val[1] = vrhadd_u8(vget_low_u8(v), vget_high_u8(v));

            // Store Y components for the current row
            vst1q_u8(y_ptr, y1);
            y_ptr += 16;

            // Store Y components for the next row
            vst1q_u8(y_ptr, y2);
            y_ptr += 16;

            // Store UV components
            vst1_u8(uv_ptr, uv.val[0]);
            uv_ptr += 8;
            vst1_u8(uv_ptr, uv.val[1]);
            uv_ptr += 8;

            yuyv_ptr += 32; // Move to the next 16 pixels (8 YUYV pairs)
        }
    }
}

static void convertYUYVtoNV12(const Mat& yuyvImage, Mat& nv12Image) {
    int width = yuyvImage.cols;
    int height = yuyvImage.rows;

    // Create Y, U, and V planes
    Mat yPlane(height, width, CV_8UC1);
    Mat uPlane(height / 2, width / 2, CV_8UC1);
    Mat vPlane(height / 2, width / 2, CV_8UC1);

    // Extract Y, U, V from YUYV
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j += 2) {
            int index = i * width + j;
            uchar y0 = yuyvImage.data[index * 2];
            uchar u0 = yuyvImage.data[index * 2 + 1];
            uchar y1 = yuyvImage.data[index * 2 + 2];
            uchar v0 = yuyvImage.data[index * 2 + 3];

            yPlane.at<uchar>(i, j) = y0;
            yPlane.at<uchar>(i, j + 1) = y1;
            if (i % 2 == 0 && j % 2 == 0) {
                uPlane.at<uchar>(i / 2, j / 2) = u0;
                vPlane.at<uchar>(i / 2, j / 2) = v0;
            }
        }
    }

    // Create UV interleaved plane
    Mat uvPlane(height / 2, width, CV_8UC1);
    for (int i = 0; i < height / 2; i++) {
        for (int j = 0; j < width / 2; j++) {
            uvPlane.at<uchar>(i, 2 * j) = uPlane.at<uchar>(i, j);
            uvPlane.at<uchar>(i, 2 * j + 1) = vPlane.at<uchar>(i, j);
        }
    }

    // Combine Y and UV planes into NV12 format
    nv12Image.create(height + height / 2, width, CV_8UC1);
    yPlane.copyTo(nv12Image(Rect(0, 0, width, height)));
    uvPlane.copyTo(nv12Image(Rect(0, height, width, height / 2)));
}

/********************************** FOR OMX ***********************************/

OMX_ERRORTYPE omx_event_handler(OMX_HANDLETYPE hComponent, OMX_PTR pAppData,
                                OMX_EVENTTYPE eEvent, OMX_U32 nData1,
                                OMX_U32 nData2, OMX_PTR pEventData)
{
    /* Mark parameters as unused */
    UNUSED(pAppData);
    UNUSED(hComponent);
    UNUSED(pEventData);

    char * p_state_str = NULL;

    switch (eEvent)
    {
        case OMX_EventCmdComplete:
        {
            if (nData1 == OMX_CommandStateSet)
            {
                p_state_str = omx_state_to_str((OMX_STATETYPE)nData2);
                if (p_state_str != NULL)
                {
                    /* Print OMX state */
                    printf("OMX state: '%s'\n", p_state_str);
                    free(p_state_str);
                }
            }
        }
        break;

        case OMX_EventBufferFlag:
        {
            /* The buffer contains the last output picture data */
            printf("OMX event: 'End-of-Stream'\n");
        }
        break;

        case OMX_EventError:
        {
            /* Section 2.1.2 in document 'R01USxxxxEJxxxx_vecmn_v1.0.pdf' */
            printf("OMX error event: '0x%x'\n", nData1);
        }
        break;

        default:
        {
            /* Intentionally left blank */
        }
        break;
    }

    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_empty_buffer_done(OMX_HANDLETYPE hComponent,
                                    OMX_PTR pAppData,
                                    OMX_BUFFERHEADERTYPE * pBuffer)
{
    /* Mark parameter as unused */
    UNUSED(hComponent);

    omx_data_t * p_data = (omx_data_t *)pAppData;

    /* Check parameter */
    assert(p_data != NULL);

    if (pBuffer != NULL)
    {
        assert(pthread_mutex_lock(p_data->p_mut_in) == 0);

        /* At this point, the queue must not be full */
        assert(!queue_is_full(p_data->p_in_queue));

        /* Enqueue empty buffer to be used by Input Thread */
        assert(queue_enqueue(p_data->p_in_queue, &pBuffer));

        /* Now, there is an element inside the queue.
         * Input thread should woken up in case it's sleeping */
        assert(pthread_cond_signal(p_data->p_cond_in_available) == 0);

        assert(pthread_mutex_unlock(p_data->p_mut_in) == 0);
    }

    //printf("EmptyBufferDone exited\n");
    return OMX_ErrorNone;
}

OMX_ERRORTYPE omx_fill_buffer_done(OMX_HANDLETYPE hComponent,
                                   OMX_PTR pAppData,
                                   OMX_BUFFERHEADERTYPE * pBuffer)
{
    /* Mark parameter as unused */
    UNUSED(hComponent);

    omx_data_t * p_data = (omx_data_t *)pAppData;

    /* Check parameter */
    assert(p_data != NULL);

    if ((pBuffer != NULL) && (pBuffer->nFilledLen > 0))
    {
        assert(pthread_mutex_lock(p_data->p_mut_out) == 0);

        /* Add this point, the queue must not be full */
        assert(!queue_is_full(p_data->p_out_queue));

        /* Enqueue full buffer and send to Out Thread */
        assert(queue_enqueue(p_data->p_out_queue, &pBuffer));

        /* Now, there is an element inside the queue.
         * Output thread should woken up in case it's sleeping */
        assert(pthread_cond_signal(p_data->p_cond_out_available) == 0);

        assert(pthread_mutex_unlock(p_data->p_mut_out) == 0);
    }

    //printf("FillBufferDone exited\n");
    return OMX_ErrorNone;
}

/******************************** FOR THREADS *********************************/

void* thread_input(void* p_param)
{
    PipelineData* p_pipeline = (PipelineData*)p_param;
    assert(p_pipeline != NULL);

    in_data_t* p_data = (in_data_t*)p_pipeline->in_data;
    assert(p_data != NULL);

    printf("Worker in ID: %X\n", p_pipeline->thread_id);

    OMX_BUFFERHEADERTYPE* p_buf = NULL;
    GstBuffer* buffer = NULL;
    
    GstMapInfo info;

    /**************************************************************************
     *             STEP 1: INITIALIZE VSPM ISU             *
     **************************************************************************/

    isu_job_t *p_vspm_job = create_job_yuyv_to_nv12 ( FRAME_WIDTH_IN_PIXELS, FRAME_HEIGHT_IN_PIXELS);
    assert ( p_vspm_job != NULL );
    p_vspm_job->handle = p_data->vspm_handle;

    vspm_tp_cb_info_t *cb_info = (vspm_tp_cb_info_t*)malloc(sizeof(vspm_tp_cb_info_t));
    assert(cb_info != NULL);
	memset(cb_info, 0, sizeof( vspm_tp_cb_info_t));

    pthread_mutex_init(&cb_info->mutex, NULL);
	pthread_cond_init(&cb_info->cond, NULL);

    cb_info->queue = queue_create_empty( NV12_BUFFER_COUNT, sizeof(unsigned long) );


    mmngr_buf_t *p_yuyv_buf = mmngr_alloc_nv12_dmabufs(YUYV_BUFFER_COUNT,
                                           YUYV_FRAME_SIZE_IN_BYTES);
    assert(p_yuyv_buf != NULL);


    struct timespec convert_start;
    struct timespec convert_end;

    while (p_pipeline->running)
    {
        //timespec_get(&p_pipeline->metrics.fps_end, TIME_UTC);
        //float time = (float)((timedifference_msec(p_pipeline->metrics.fps_start, p_pipeline->metrics.fps_end)));
        //printf("Captue time: %f\n", time);
        //p_pipeline->metrics.fps = (1/time)*1000;
        

        /**************************************************************************
         *             STEP : GSTREAMER Wait for Capture                          *
         **************************************************************************/
        {
            /* Lock GStreamer and wait for image dequeue */
            std::unique_lock<std::mutex> lock(p_pipeline->queue_mutex);
            p_pipeline->queue_cond.wait(lock, [&]{ return !p_pipeline->buffer_queue.empty() || !p_pipeline->running; });

            /* Check that GStreamer is still running */
            if (!p_pipeline->running) break;

            /* Pop GStreamer buffer from appsink */
            buffer = p_pipeline->buffer_queue.front();
            p_pipeline->buffer_queue.pop();
        }
        assert(buffer != NULL);

        

        /**************************************************************************
         *             STEP : GSTREAMER Capture Buffer                            *
         **************************************************************************/

        if (gst_buffer_map(buffer, &info, GST_MAP_READ))
        {
            /* Copy GSTreamer buffer to thead buffer */
            /* GSTREAMER RECOMMENDS THIS IF PROCESSING WILL TAKE LONGER THAT THE GSTREAMER FPS */
            memcpy((void*)p_yuyv_buf->virt_addr, info.data, YUYV_FRAME_SIZE_IN_BYTES);

            /* Release GSTreamer buffer */
            gst_buffer_unmap(buffer, &info);
            gst_buffer_unref(buffer);
        }
        /* Start Conversion and Compression */
        timespec_get(&p_pipeline->metrics.fps_start, TIME_UTC);

        /**************************************************************************
         *         STEP : Wait for OMX Input port                                 *
        **************************************************************************/
        assert(pthread_mutex_lock(p_data->p_mutex) == 0);

        while (queue_is_empty(p_data->p_queue))
        {
            assert(0 == pthread_cond_wait(p_data->p_cond_available, p_data->p_mutex));
        }
        assert(!queue_is_empty(p_data->p_queue));

        /*************************************************************************
         *         STEP : OMX Get Input Port Buffer                              *
        **************************************************************************/
        
        p_buf = *(OMX_BUFFERHEADERTYPE**)(queue_dequeue(p_data->p_queue));
        assert(p_buf != NULL);

        int index = omx_get_index(p_buf, p_data->pp_bufs, NV12_BUFFER_COUNT);
        assert(index != -1);

        assert(pthread_mutex_unlock(p_data->p_mutex) == 0);

        /**************************************************************************
         *             STEP : CONVERT YUYV TO NV12                                *
        **************************************************************************/
        /* Start Convert Counter */
        timespec_get(&convert_start, TIME_UTC);

        /* 
        * Start VSPM Job  
        * VSPM Job Queue is limited. For now we will limit the queue to one.
        */
        pthread_mutex_lock(p_data->p_vspm_mutex);
        
        pthread_mutex_lock(&cb_info->mutex);

        assert ( start_job( p_vspm_job, 
                        (void *)(uintptr_t)p_yuyv_buf->hard_addr, 
                        //g_frame.data,
                        (void *)(uintptr_t)p_data->p_nv12_bufs[index].hard_addr, 
                        vspm_isu_callback, (void*)cb_info ) != -1);


    
        while (queue_is_empty(&cb_info->queue))
        {
            pthread_cond_wait(&cb_info->cond, &cb_info->mutex);
        }
        queue_dequeue(&cb_info->queue);
        pthread_mutex_unlock(&cb_info->mutex);

        pthread_mutex_unlock(p_data->p_vspm_mutex);

        /* End Capture and Format Converion */
        timespec_get(&convert_end, TIME_UTC);

        p_pipeline->metrics.convert_time = (float)((timedifference_msec(convert_start, convert_end)));
        
        
    
        /**************************************************************************
         *         STEP : Send OMX Request                            *
        **************************************************************************/

        timespec_get(&p_pipeline->metrics.encode_start, TIME_UTC);

        /* Send OMX signal next image is ready for processing */
        p_buf->nFilledLen = NV12_FRAME_SIZE_IN_BYTES;
        p_buf->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;
        p_buf->hMarkTargetComponent = p_data->handle;
        p_buf->pMarkData = &p_pipeline->index;

        if (g_int_signal)
        {
            p_buf->nFlags |= OMX_BUFFERFLAG_EOS;
            p_pipeline->running = false;
        }

        assert(OMX_EmptyThisBuffer(p_data->handle, p_buf) == OMX_ErrorNone);
        
    }

    printf("Thread '%s' exited\n", __FUNCTION__);
    return NULL;
}


void* thread_output(void* p_param)
{
    PipelineData* p_pipeline = (PipelineData*)p_param;
    assert(p_pipeline != NULL);

    out_data_t* p_data = (out_data_t*)p_pipeline->out_data;
    assert(p_data != NULL);

    printf("Worker out ID: %X\n", p_pipeline->thread_id);

    OMX_BUFFERHEADERTYPE* p_buf = NULL;
    p_pipeline->p_h264_fd = fopen(p_pipeline->out_filename.c_str(), "wb");
    assert(p_pipeline->p_h264_fd != NULL);

    while (p_pipeline->running)
    {   
        /* Wait till data is ready for this thread */
        //while ( omx_share_processing.load() != p_pipeline->index ) usleep(5);        

        assert(pthread_mutex_lock(p_data->p_mutex) == 0);

        while (queue_is_empty(p_data->p_queue))
        {
            assert(0 == pthread_cond_wait(p_data->p_cond_available, p_data->p_mutex));
        }


        assert(!queue_is_empty(p_data->p_queue));
        p_buf = *(OMX_BUFFERHEADERTYPE**)(queue_dequeue(p_data->p_queue));
        assert(p_buf != NULL);

        assert(pthread_mutex_unlock(p_data->p_mutex) == 0);

        //int index = omx_share_processing.load();
        //index = (++index) % NUM_PIPELINES;
        //omx_share_processing.store(index);

        timespec_get(&p_pipeline->metrics.encode_end, TIME_UTC);

        fwrite((char*)(p_buf->pBuffer), 1, p_buf->nFilledLen, p_pipeline->p_h264_fd);
        
        

        if (p_buf->nFlags & OMX_BUFFERFLAG_EOS)
        {
            p_pipeline->running = 0;
        }
        else
        {
            p_buf->nFilledLen = 0;
            p_buf->nFlags = 0;
            assert(OMX_FillThisBuffer(p_data->handle, p_buf) == OMX_ErrorNone);
        }
        timespec_get(&p_pipeline->metrics.fps_end, TIME_UTC);
        float total_time = (float)((timedifference_msec(p_pipeline->metrics.fps_start, p_pipeline->metrics.fps_end)));
        float encode_time = (float)((timedifference_msec(p_pipeline->metrics.encode_start, p_pipeline->metrics.encode_end)));
        
        printf("ID: %X,\tCompression/Storage:\tFPS: %f\tConvert: %f\tEncode: %f\n ", 
                                    p_pipeline->thread_id,
                                    (1000/total_time),
                                    p_pipeline->metrics.convert_time, 
                                    encode_time);
        
    }

    fclose(p_pipeline->p_h264_fd);
    printf("Thread '%s' exited\n", __FUNCTION__);
    return NULL;
}

void vspm_isu_callback ( unsigned long job_id, long result, void *user_data )
{
    vspm_tp_cb_info_t *cb_info =
		(vspm_tp_cb_info_t *)user_data;

	pthread_mutex_lock(&cb_info->mutex);
	
    queue_enqueue(&cb_info->queue, &job_id );

	pthread_cond_signal(&cb_info->cond);
	pthread_mutex_unlock(&cb_info->mutex);


}

