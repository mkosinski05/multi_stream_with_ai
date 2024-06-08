#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <queue>
#include <mutex>
#include <condition_variable>

#define NUM_PIPELINES 2

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
} PipelineData;

// Function to process image data with OpenCV
void process_image_data(guint8 *data, gsize size) {
    // Assuming the data is in RGB format and has known width and height
    int width = 640;  // replace with actual width
    int height = 480; // replace with actual height

    // Create an OpenCV Mat from the buffer data
    cv::Mat img(cv::Size(width, height), CV_8UC3, data);

    // Process the image using OpenCV
    cv::Mat processed_img;
    cv::cvtColor(img, processed_img, cv::COLOR_RGB2BGR); // Example processing

    // Display the processed image
    cv::imshow("Processed Image", processed_img);
    cv::waitKey(1);
}

// Thread function for processing the queue
void* processing_thread(void* arg) {
    PipelineData* data = (PipelineData*) arg;
    while (data->running) {
        GstBuffer *buffer = NULL;

        // Wait for a buffer to be available
        {
            std::unique_lock<std::mutex> lock(data->queue_mutex);
            data->queue_cond.wait(lock, [&]{ return !data->buffer_queue.empty() || !data->running; });

            if (!data->running) break;

            buffer = data->buffer_queue.front();
            data->buffer_queue.pop();
        }

        if (buffer) {
            GstMapInfo info;
            if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
                // Process image data using OpenCV
                process_image_data(info.data, info.size);
                gst_buffer_unmap(buffer, &info);
            }
            gst_buffer_unref(buffer);
        }
    }
    return NULL;
}

// Callback function for new samples
static GstFlowReturn new_sample_callback(GstAppSink *appsink, gpointer user_data) {
    PipelineData* data = (PipelineData*) user_data;
    GstSample *sample;
    GstBuffer *buffer;

    sample = gst_app_sink_pull_sample(appsink);
    if (sample) {
        buffer = gst_sample_get_buffer(sample);
        if (buffer) {
            gst_buffer_ref(buffer);  // Increment ref count to ensure buffer is valid during processing

            // Add the buffer to the queue
            {
                std::lock_guard<std::mutex> lock(data->queue_mutex);
                data->buffer_queue.push(buffer);
            }
            data->queue_cond.notify_one();
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

    // Start processing thread
    pthread_t proc_thread_id;
    if (pthread_create(&proc_thread_id, NULL, processing_thread, data) != 0) {
        g_printerr("Failed to create processing thread for pipeline %s.\n", data->pipeline_name);
        return NULL;
    }

    // Create and run the main loop for this pipeline
    data->main_loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(data->main_loop);

    // Stop processing thread
    {
        std::lock_guard<std::mutex> lock(data->queue_mutex);
        data->running = 0;
    }
    data->queue_cond.notify_all();
    pthread_join(proc_thread_id, NULL);

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
        "v4l2src device=/dev/video0 ! videoconvert ! appsink name=sink0",
        "v4l2src device=/dev/video1 ! videoconvert ! appsink name=sink1"
    };

    for (int i = 0; i < NUM_PIPELINES; i++) {
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
    }

    for (int i = 0; i < NUM_PIPELINES; i++) {
        pthread_join(pipelines[i].thread_id, NULL);
        g_free(pipelines[i].pipeline_name);
    }

    return 0;
}
