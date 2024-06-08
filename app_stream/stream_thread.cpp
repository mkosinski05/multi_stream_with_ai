
#include "defines.h"


extern volatile sig_atomic_t g_int_signal;

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

    printf("EmptyBufferDone exited\n");
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

    printf("FillBufferDone exited\n");
    return OMX_ErrorNone;
}

/******************************** FOR THREADS *********************************/

void * thread_input(void * p_param)
{
    in_data_t * p_data = (in_data_t *)p_param;

    /* Check parameter */
    assert(p_data != NULL);

    /* true:  The loop is running.
     * false: The loop just stopped */
    bool is_running = true;

    /* V4L2 buffer */
    struct v4l2_buffer cam_buf;

    /* Buffer of input port */
    int index = -1;
    OMX_BUFFERHEADERTYPE * p_buf = NULL;

    vspm_tp_cb_info_t *cb_info = NULL;

    /**************************************************************************
     *             STEP 7: INITIALIZE VSPM ISU             *
     **************************************************************************/

    assert(isu_init() != NULL);
    isu_job_t *p_vspm_job = create_job_yuyv_to_nv12 ( FRAME_WIDTH_IN_PIXELS, FRAME_HEIGHT_IN_PIXELS);
    assert ( p_vspm_job != NULL );

    cb_info = (vspm_tp_cb_info_t*)malloc(sizeof(vspm_tp_cb_info_t));
    assert(cb_info != NULL);
	memset(cb_info, 0, sizeof( vspm_tp_cb_info_t));

	pthread_mutex_init(&cb_info->mutex, NULL);
	pthread_cond_init(&cb_info->cond, NULL);


    /**************************************************************************
     *                       STEP 7: THREAD'S MAIN LOOP                       *
     **************************************************************************/

    while (is_running)
    {
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

        /* Get buffer's index */
        index = omx_get_index(p_buf, p_data->pp_bufs, NV12_BUFFER_COUNT);
        assert(index != -1);

        /* Receive camera's buffer */
        assert(v4l2_dequeue_buf(p_data->cam_fd, &cam_buf));

        //paddr = (void *)(uintptr_t)p_data->p_nv12_bufs[index].virt_addr;
        //memcpy ( paddr, p_data->p_yuyv_bufs[cam_buf.index].p_virt_addr, p_data->p_nv12_bufs->size);
        
        assert (start_job( p_vspm_job, 
            (void*)p_data->p_yuyv_bufs[cam_buf.index].hard_addr, 
            (void *)(uintptr_t)p_data->p_nv12_bufs[index].hard_addr, 
            vspm_isu_callback, cb_info ) != -1);

        /* VSPM Job wait callback */
        pthread_mutex_lock(&cb_info->mutex);
        pthread_cond_wait(&cb_info->cond, &cb_info->mutex);
        pthread_mutex_unlock(&cb_info->mutex);

        /* check callback information */
        assert ((cb_info->ercd != 0) || (cb_info->job_id != p_vspm_job->job_id));
            

        /* Reuse camera's buffer */
        assert(v4l2_enqueue_buf(p_data->cam_fd, cam_buf.index));

        /* If 'p_buf' contains data, 'nFilledLen' must not be zero */
        p_buf->nFilledLen = NV12_FRAME_SIZE_IN_BYTES;

        /* Section 6.14.1 in document 'R01USxxxxEJxxxx_vecmn_v1.0.pdf' */
        p_buf->nFlags = OMX_BUFFERFLAG_ENDOFFRAME;

        if (g_int_signal)
        {
            p_buf->nFlags |= OMX_BUFFERFLAG_EOS;

            /* Exit loop */
            is_running = false;
        }

        /* Send the buffer to the input port of the component */
        assert(OMX_EmptyThisBuffer(p_data->handle, p_buf) == OMX_ErrorNone);
    }

    printf("Thread '%s' exited\n", __FUNCTION__);
    return NULL;
}

void * thread_output(void * p_param)
{
    out_data_t * p_data = (out_data_t *)p_param;

    /* true:  The loop is running.
     * false: The loop just stopped */
    bool is_running = true;

    /* Buffer of output port */
    OMX_BUFFERHEADERTYPE * p_buf = NULL;

    /* File for writing H.264 data */
    FILE * p_h264_fd = NULL;

    /* Check parameter */
    assert(p_data != NULL);

    /* Open file */
    p_h264_fd = fopen(H264_FILE_NAME, "w");
    assert(p_h264_fd != NULL);

    while (is_running)
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
        fwrite((char *)(p_buf->pBuffer), 1, p_buf->nFilledLen, p_h264_fd);

        if (p_buf->nFlags & OMX_BUFFERFLAG_EOS)
        {
            /* Exit loop */
            is_running = false;
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
    fclose(p_h264_fd);

    printf("Thread '%s' exited\n", __FUNCTION__);
    return NULL;
}

/************************ VSPM IF ISU ******************************************/
void vspm_isu_callback ( unsigned long job_id, long result, void *user_data )
{
    vspm_tp_cb_info_t *cb_info =
		(vspm_tp_cb_info_t *)user_data;

	pthread_mutex_lock(&cb_info->mutex);
	cb_info->job_id = job_id;
	cb_info->ercd = result;
	pthread_cond_signal(&cb_info->cond);
	pthread_mutex_unlock(&cb_info->mutex);
}
