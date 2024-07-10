
/*****************************************
* includes
******************************************/
#include "MeraDrpRuntimeWrapper.h"
#include "PreRuntime.h"
#include <linux/drpai.h>
#include <linux/input.h>
#include <builtin_fp16.h>
#include <opencv2/opencv.hpp>

//add
#include<iostream>
#include<fstream>

#include "stream.h"
#include "inference.h"

using namespace std;
using namespace cv;

/******************************************************************************
*           Global Variables
*******************************************************************************/
static float drpai_output_buf[INF_OUT_SIZE];
static int32_t drp_max_freq;
static int32_t drp_freq;

extern void yolov2_parser(float *floatarr);
extern uint32_t  draw_bounding_box(FILE *fp);

/*****************************************
* Function Name : get_drpai_start_addr
* Description   : Function to get the start address of DRPAImem.
* Arguments     : drpai_fd: DRP-AI file descriptor
* Return value  : If non-zero, DRP-AI memory start address.
*                 0 is failure.
******************************************/
uint64_t get_drpai_start_addr(int drpai_fd)

{
    int ret = 0;
    drpai_data_t drpai_data;

    errno = 0;

    /* Get DRP-AI Memory Area Address via DRP-AI Driver */
    ret = ioctl(drpai_fd , DRPAI_GET_DRPAI_AREA, &drpai_data);
    if (-1 == ret)
    {
        std::cerr << "[ERROR] Failed to get DRP-AI Memory Area : errno=" << errno << std::endl;
        return 0;
    }

    return drpai_data.address;
}

/*****************************************
* Function Name : set_drpai_freq
* Description   : Function to set the DRP and DRP-AI frequency.
* Arguments     : drpai_fd: DRP-AI file descriptor
* Return value  : 0 if succeeded
*                 not 0 otherwise
******************************************/
int set_drpai_freq(int drpai_fd)
{
    int ret = 0;
    uint32_t data;

    errno = 0;
    data = DRP_MAX_FREQ;
    ret = ioctl(drpai_fd , DRPAI_SET_DRP_MAX_FREQ, &data);
    if (-1 == ret)
    {
        std::cerr << "[ERROR] Failed to set DRP Max Frequency : errno=" << errno << std::endl;
        return -1;
    }

    errno = 0;
    data = DRPAI_FREQ;
    ret = ioctl(drpai_fd , DRPAI_SET_DRPAI_FREQ, &data);
    if (-1 == ret)
    {
        std::cerr << "[ERROR] Failed to set DRP-AI Frequency : errno=" << errno << std::endl;
        return -1;
    }

    return 0;
}


/*****************************************
* Function Name : init_drpai
* Description   : Function to initialize DRP-AI.
* Arguments     : drpai_fd: DRP-AI file descriptor
* Return value  : If non-zero, DRP-AI memory start address.
*                 0 is failure.
******************************************/
uint64_t init_drpai(int drpai_fd)

{
    int ret = 0;
    uint64_t drpai_addr = 0;

    /*Get DRP-AI memory start address*/
    drpai_addr = get_drpai_start_addr(drpai_fd);
    if (drpai_addr == 0)
    {
        return 0;
    }

    /*Set DRP-AI frequency*/
    ret = set_drpai_freq(drpai_fd);
    if (ret != 0)
    {
        return 0;
    }


    return drpai_addr;
}

/*****************************************
 * Function Name     : float16_to_float32
 * Description       : Function by Edgecortex. Cast uint16_t a into float value.
 * Arguments         : a = uint16_t number
 * Return value      : float = float32 number
 ******************************************/
float float16_to_float32(uint16_t a)
{
    return __extendXfYf2__<uint16_t, uint16_t, 10, float, uint32_t, 23>(a);
}

/*****************************************
 * Function Name     : load_label_file
 * Description       : Load label list text file and return the label list that contains the label.
 * Arguments         : label_file_name = filename of label list. must be in txt format
 * Return value      : vector<string> list = list contains labels
 *                     empty if error occurred
 ******************************************/
vector<string> load_label_file(string label_file_name)
{
    vector<string> list = {};
    vector<string> empty = {};
    ifstream infile(label_file_name);

    if (!infile.is_open())
    {
        return list;
    }

    string line = "";
    while (getline(infile, line))
    {
        list.push_back(line);
        if (infile.fail())
        {
            return empty;
        }
    }

    return list;
}

void * thread_infer(void * p_param) 
{
    int ret = 0;
    PipelineData *p_pipeline = (PipelineData*)p_param;
    assert(p_pipeline != NULL);

    inf_data_t * p_data = (inf_data_t *)p_pipeline->inf_data;
    assert(p_data != NULL);

    
    /* DRP-AI TVM[*1] Runtime object */
    MeraDrpRuntimeWrapper runtime;
    /* Pre-processing Runtime object */
    PreRuntime preruntime;

    float FPS = 0;
    long long PREPROCESS_START_TIME = 0;

    float fps = 0;
    float TOTAL_TIME = 0;
    float INF_TIME= 0;
    float POST_PROC_TIME = 0;
    float PRE_PROC_TIME = 0;
    int32_t HEAD_COUNT= 0;

    GstMapInfo info;

    /* Output variables for Pre-processing Runtime */
    void *output_ptr;
    uint32_t out_size;

    Size size(MODEL_IN_H, MODEL_IN_W);

    int drpai_fd = open("/dev/drpai0", O_RDWR);
    assert (0 < drpai_fd);

    /*Load Label from label_list file*/
    label_file_map = load_label_file(label_list);

    /*Initialzie DRP-AI (Get DRP-AI memory address and set DRP-AI frequency)*/
    uint64_t drpaimem_addr_start = init_drpai(drpai_fd);

    assert (drpaimem_addr_start != 0);

    /*Load pre_dir object to DRP-AI */
    ret = preruntime.Load(pre_dir);
    if (0 < ret)
    {
        std::cerr << "[ERROR] Failed to run Pre-processing Runtime Load()." << std::endl;
        return 0;
    }

    /*Load model_dir structure and its weight to runtime object */
    runtime.LoadModel(model_dir, drpaimem_addr_start + DRPAI_MEM_OFFSET);

    //ofstream outputfile("ai_output.txt", ios::app);
    FILE *outputfile = fopen("ai_output.txt", "w");

    std::vector<std::chrono::milliseconds> timestamps;

     while (p_pipeline->running)
    {
        /* Receive camera's buffer */
        //assert(v4l2_dequeue_buf(p_data->cam_fd, &cam_buf));

        
        /**********************************************************************
         *      DRP-AI TVM Preprocessing 
        ***********************************************************************/
       

        auto tp = std::chrono::system_clock::now();
        auto tp_msec = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
        
        GstBuffer *buffer = NULL;

        // Wait for a buffer to be available
        {
            std::unique_lock<std::mutex> lock(p_pipeline->queue_mutex);
            p_pipeline->queue_cond.wait(lock, [&]{ return !p_pipeline->buffer_queue.empty() || !p_pipeline->running; });

            if (!p_pipeline->running) break;

            buffer = p_pipeline->buffer_queue.front();
            p_pipeline->buffer_queue.pop();
        }

        gst_buffer_map(buffer, &info, GST_MAP_READ);

        s_preproc_param_t in_param;
        in_param.pre_in_addr    = (uint64_t)info.data;

        /* Preprocess time start */
        auto t0 = std::chrono::high_resolution_clock::now();

        /*Run pre-processing*/
        if (0 < preruntime.Pre(&in_param, &output_ptr, &out_size))
        {
            std::cerr << "[ERROR] Failed to run Pre-processing Runtime Pre()." << std::endl;
            continue;
        }
        // Release GST Buffer
        gst_buffer_unmap(buffer, &info);
        gst_buffer_unref(buffer);

        /* Preprocess time ends*/
        auto t1 = std::chrono::high_resolution_clock::now();

        /*start inference using drp runtime*/
        runtime.SetInput(0, (float*)output_ptr);

        /**********************************************************************
         *      DRP-AI TVM Runtime inference
        ***********************************************************************/

        

        /* Inference time start */
        auto t2 = std::chrono::high_resolution_clock::now();
        runtime.Run();
        /* Inference time end */
        auto t3 = std::chrono::high_resolution_clock::now();
        auto inf_duration = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

        /* Postprocess time start */
        auto t4 = std::chrono::high_resolution_clock::now();

        /*load inference out on drpai_out_buffer*/
        int32_t i = 0;
        int32_t output_num = 0;
        std::tuple<InOutDataType, void *, int64_t> output_buffer;
        int64_t output_size;
        uint32_t size_count = 0;

        /* Get the number of output of the target model. */
        output_num = runtime.GetNumOutput();

        size_count = 0;
        /*GetOutput loop*/
        for (i = 0; i < output_num; i++)
        {
            
            /* output_buffer below is tuple, which is { data type, address of output data, number of elements } */
            output_buffer = runtime.GetOutput(i);

            /*Output Data Size = std::get<2>(output_buffer). */
            output_size = std::get<2>(output_buffer);

            /*Output Data Type = std::get<0>(output_buffer)*/
            if (InOutDataType::FLOAT16 == std::get<0>(output_buffer))
            {
                /*Output Data = std::get<1>(output_buffer)*/
                uint16_t *data_ptr = reinterpret_cast<uint16_t *>(std::get<1>(output_buffer));
                for (int j = 0; j < output_size; j++)
                {
                    /*FP16 to FP32 conversion*/
                    drpai_output_buf[j + size_count] = float16_to_float32(data_ptr[j]);
                }
            }
            else if (InOutDataType::FLOAT32 == std::get<0>(output_buffer))
            {
                /*Output Data = std::get<1>(output_buffer)*/
                float *data_ptr = reinterpret_cast<float *>(std::get<1>(output_buffer));
                for (int j = 0; j < output_size; j++)
                {
                    drpai_output_buf[j + size_count] = data_ptr[j];
                }
            }
            else
            {
                std::cerr << "[ERROR] Output data type : not floating point." << std::endl;
                ret = -1;
                break;
            }
            size_count += output_size;

        }
    
        if (ret != 0)
        {
            std::cerr << "[ERROR] DRP Inference Not working !!! " << std::endl;
            break;
        }

    /* Do post process to get bounding boxes */
        yolov2_parser(drpai_output_buf);
        fprintf(outputfile, "{%lld:\t[\n", tp_msec
        );
    
        HEAD_COUNT = draw_bounding_box(outputfile);

        /* Postprocess time end */
        auto t5 = std::chrono::high_resolution_clock::now();

        auto r_post_proc_time = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
        auto pre_proc_time = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        POST_PROC_TIME = r_post_proc_time/1000.0;
        PRE_PROC_TIME = pre_proc_time/1000.0;
        INF_TIME = inf_duration/1000.0;

        float total_time = float(inf_duration/1000.0) + float(POST_PROC_TIME) + float(pre_proc_time/1000.0);
        TOTAL_TIME = total_time;

        // Calculate Total FPS
        FPS=1000/TOTAL_TIME;

        
        //outputfile << PREPROCESS_START_TIME << " " << HEAD_COUNT  << " " << FPS << " " << TOTAL_TIME << " " << INF_TIME << " " << PRE_PROC_TIME << " " << POST_PROC_TIME << "\n";
        fprintf(outputfile, "], count: %d, fps: %f, total_time: %f, Inference time: %f, Pre Proc time: %f, Post Process time: %f}\n",
                HEAD_COUNT, FPS, TOTAL_TIME, INF_TIME, PRE_PROC_TIME, POST_PROC_TIME);
        printf ( "ID: %X,\tInference:\t\tFPS: %f \n",  p_pipeline->thread_id, FPS);
        auto FILE_END_TIME = std::chrono::seconds(1s).count();
    }
    // file close
    //outputfile.close();
    fclose(outputfile);
}

