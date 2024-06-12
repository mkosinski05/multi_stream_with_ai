
#include <queue>
#include "stream.h"
#include <opencv2/opencv.hpp>

using namespace cv;

extern volatile sig_atomic_t g_int_signal; 

static double timedifference_msec(struct timespec t0, struct timespec t1)
{
    return (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1000000.0;
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

        /* Add 'pBuffer' to the queue */
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

        /* Add 'pBuffer' to the queue */
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

void * thread_input(void * p_param)
{
    PipelineData *p_pipeline = (PipelineData*)p_param;
    assert(p_pipeline != NULL);

    in_data_t * p_data = (in_data_t*)p_pipeline->in_data;
    assert(p_data != NULL);

    /* Buffer of input port */
    int index = -1;
    OMX_BUFFERHEADERTYPE * p_buf = NULL;

    float convert_time = 0.0f;

    GstBuffer *buffer = NULL;

    // Create Mat objects for Y and UV planes
    //cv::Mat y_plane(FRAME_HEIGHT_IN_PIXELS, FRAME_WIDTH_IN_PIXELS, CV_8UC1, nv12_array);
    //cv::Mat uv_plane(FRAME_HEIGHT_IN_PIXELS / 2, FRAME_WIDTH_IN_PIXELS / 2, CV_8UC2, nv12_array + y_plane_size);

    // Merge Y and UV planes into a single NV12 Mat
    //std::vector<cv::Mat> nv12_planes = { y_plane, uv_plane };
    cv::Mat nv12_image;
    //cv::vconcat(nv12_planes, nv12_image);

    GstMapInfo info;

 
    /**************************************************************************
     *                       STEP 7: THREAD'S MAIN LOOP                       *
     **************************************************************************/

    while ( p_pipeline->running)
    {
        /* Capture Camera image */
        // Wait for a buffer to be available
        {
            std::unique_lock<std::mutex> lock(p_pipeline->queue_mutex);
            p_pipeline->queue_cond.wait(lock, [&]{ return !p_pipeline->buffer_queue.empty() || !p_pipeline->running; });

            if (!p_pipeline->running) break;

            buffer = p_pipeline->buffer_queue.front();
            p_pipeline->buffer_queue.pop();
        }
        assert(buffer != NULL);

        timespec_get(&p_pipeline->metrics.yuv2nv12_start, TIME_UTC);


        if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
                   
            // Copy the capture buffer to the CMA Buffer
            // Do this because software conversion is slow (> 122mseconds)
            memcpy((void*)p_pipeline->in_data->p_yuyv_bufs->virt_addr, info.data, YUYV_FRAME_SIZE_IN_BYTES);
            
            
            // Release GST Buffer
            gst_buffer_unmap(buffer, &info);
            gst_buffer_unref(buffer);
#if 0
            FILE * fp = fopen("out.yuyv", "wb");
            fwrite((void*)p_pipeline->in_data->p_yuyv_bufs->virt_addr, 1, YUYV_FRAME_SIZE_IN_BYTES, fp);
            fclose(fp);
#endif
        }

        /* Convert the YUYV image to NV12 using OpenCV */
        cv::Mat yuyv_image( FRAME_HEIGHT_IN_PIXELS, 
                            FRAME_WIDTH_IN_PIXELS, CV_8UC2, 
                            (void*)p_pipeline->in_data->p_yuyv_bufs->virt_addr);
#if 1
        convertYUYVtoNV12(yuyv_image, nv12_image);
#endif
        timespec_get(&p_pipeline->metrics.yuv2nv12_end, TIME_UTC);
        convert_time = (float)((timedifference_msec(
                                    p_pipeline->metrics.yuv2nv12_start, 
                                    p_pipeline->metrics.yuv2nv12_end)));

        printf("ID: %X,\tYUUV to NV12: %f\n", p_pipeline->thread_id,convert_time);

        
        
#if 1
        /* OMX ENCODE VIDEO STREAM */
        /* Wait for OMX is ready */
        assert(pthread_mutex_lock(p_data->p_mutex) == 0);

        while (queue_is_empty(p_data->p_queue))
        {
            /* Thread will sleep until the queue is not empty */
            assert(0 == pthread_cond_wait(p_data->p_cond_available,
                                          p_data->p_mutex));
        }

        /* At this point, the queue must have something in it */
        assert(!queue_is_empty(p_data->p_queue));

        /* Receive buffer (of input port) from the queue */
        p_buf = *(OMX_BUFFERHEADERTYPE **)(queue_dequeue(p_data->p_queue));
        assert(p_buf != NULL);

        assert(pthread_mutex_unlock(p_data->p_mutex) == 0);

        /* Get OMX buffer's index */
        index = omx_get_index(p_buf, p_data->pp_bufs, NV12_BUFFER_COUNT);
        assert(index != -1);

        // Copy OpenCV Nuffer to OMX buffer
    #if 0
        memcpy((void*)p_data->p_nv12_bufs[index].virt_addr, nv12_image.data, NV12_FRAME_SIZE_IN_BYTES);
        FILE * fp = fopen("out.yuv", "wb");
        fwrite((void*)p_pipeline->in_data->p_yuyv_bufs->virt_addr, 1, YUYV_FRAME_SIZE_IN_BYTES, fp);
        fclose(fp);
    #endif
        

        /* Copy the NV12 image to the OMX buffer */ 
        /* If 'p_buf' contains data, 'nFilledLen' must not be zero */
        p_buf->nFilledLen = NV12_FRAME_SIZE_IN_BYTES;

        /* Section 6.14.1 in document 'R01USxxxxEJxxxx_vecmn_v1.0.pdf' */
        p_buf->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;

        if (g_int_signal)
        {
            p_buf->nFlags |= OMX_BUFFERFLAG_EOS;

            /* Exit loop */
            p_pipeline->running = false;
        }

        /* Send the buffer to the input port of the component */
        assert(OMX_EmptyThisBuffer(p_data->handle, p_buf) == OMX_ErrorNone);
#endif
    }

    printf("Thread '%s' exited\n", __FUNCTION__);
    return NULL;
}

void * thread_output(void * p_param)
{
    PipelineData *p_pipeline = (PipelineData*)p_param;
    assert(p_pipeline != NULL);

    out_data_t * p_data = (out_data_t *)p_pipeline->out_data;
    assert(p_data != NULL);

    /* Buffer of output port */
    OMX_BUFFERHEADERTYPE * p_buf = NULL;

    /* Check parameter */
    assert(p_data != NULL);

    /* Open file */
    p_pipeline->p_h264_fd = fopen( p_pipeline->out_filename.c_str(), "wb");
    assert(p_pipeline->p_h264_fd != NULL);


    while (p_pipeline->running)
    {
        assert(pthread_mutex_lock(p_data->p_mutex) == 0);

        while (queue_is_empty(p_data->p_queue))
        {
            /* The thread will sleep until the queue is not empty */
            assert(0 == pthread_cond_wait(p_data->p_cond_available,
                                          p_data->p_mutex));
        }

        /* At this point, the queue must have something in it */
        assert(!queue_is_empty(p_data->p_queue));

        /* Receive buffer from the queue */
        p_buf = *(OMX_BUFFERHEADERTYPE **)(queue_dequeue(p_data->p_queue));
        assert(p_buf != NULL);

        assert(pthread_mutex_unlock(p_data->p_mutex) == 0);

        /* Write H.264 data to a file */
        fwrite((char *)(p_buf->pBuffer), 1, p_buf->nFilledLen, p_pipeline->p_h264_fd);

        if (p_buf->nFlags & OMX_BUFFERFLAG_EOS)
        {
            /* Exit loop */
            p_pipeline->running = 0;
        }
        else
        {
            p_buf->nFilledLen = 0;
            p_buf->nFlags     = 0;

            /* Send the buffer to the output port of the component */
            assert(OMX_FillThisBuffer(p_data->handle, p_buf) == OMX_ErrorNone);
        }
    }

    /* Close file */
    fclose(p_pipeline->p_h264_fd);

    printf("Thread '%s' exited\n", __FUNCTION__);
    return NULL;
}

