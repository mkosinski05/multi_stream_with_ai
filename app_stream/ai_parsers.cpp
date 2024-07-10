
#include "inference.h"
#include "box.h"
#include <builtin_fp16.h>
#include <opencv2/opencv.hpp>

//add
#include<iostream>
#include<fstream>

using namespace std;
using namespace cv;

static vector<detection> det;

/*****************************************
* Function Name : offset
* Description   : Get the offset number to access the bounding box attributes
*                 To get the actual value of bounding box attributes, use yolo_index() after this function.
* Arguments     : b = Number to indicate which bounding box in the region [0~4]
*                 y = Number to indicate which region [0~13]
*                 x = Number to indicate which region [0~13]
* Return value  : offset to access the bounding box attributes.
******************************************/
int32_t offset(int32_t b, int32_t y, int32_t x)
{
    return b * (NUM_CLASS + 5) * NUM_GRID_X * NUM_GRID_Y + y * NUM_GRID_X + x;
}

/*****************************************
* Function Name : index
* Description   : Get the index of the bounding box attributes based on the input offset
* Arguments     : offs = offset to access the bounding box attributes
*                 channel = channel to access each bounding box attribute.
* Return value  : index to access the bounding box attribute.
******************************************/
int32_t index(int32_t offs, int32_t channel)
{
    return offs + channel * NUM_GRID_X * NUM_GRID_Y;
}

/*****************************************
 * Function Name : sigmoid
 * Description   : Helper function for YOLO Post Processing
 * Arguments     : x = input argument for the calculation
 * Return value  : sigmoid result of input x
 ******************************************/
double sigmoid(double x)
{
    return 1.0 / (1.0 + exp(-x));
}

/*****************************************
 * Function Name : softmax
 * Description   : Helper function for YOLO Post Processing
 * Arguments     : val[] = array to be computed softmax
 * Return value  : -
 ******************************************/

void softmax(float val[NUM_CLASS])
{
    float max_num = -FLT_MAX;
    float sum = 0;
    int32_t i;
    for (i = 0 ; i<NUM_CLASS ; i++)
    {
        max_num = max(max_num, val[i]);
    }
    for (i = 0 ; i<NUM_CLASS ; i++)
    {
        val[i]= (float) exp(val[i] - max_num);
        sum+= val[i];
    }
    for (i = 0 ; i<NUM_CLASS ; i++)
    {
        val[i] = val[i]/sum;
    }
    return;
}

/*****************************************
 * Function Name : R_Post_Proc
 * Description   : Process CPU post-processing for YOLOv2
 * Arguments     : floatarr = drpai output address
 * Return value  : -
 ******************************************/
void yolov2_parser(float *floatarr)
{
    /* Following variables are required for correct_region_boxes in Darknet implementation*/
    /* Note: This implementation refers to the "darknet detector test" */

    float new_w, new_h;
    float correct_w = 1.;
    float correct_h = 1.;
    if ((float)(MODEL_IN_W / correct_w) < (float)(MODEL_IN_H / correct_h))
    {
        new_w = (float)MODEL_IN_W;
        new_h = correct_h * MODEL_IN_W / correct_w;
    }
    else
    {
        new_w = correct_w * MODEL_IN_H / correct_h;
        new_h = MODEL_IN_H;
    }

//    int32_t n = 0;
    int32_t b = 0;
    int32_t y = 0;
    int32_t x = 0;
    int32_t offs = 0;
    int32_t i = 0;
    float tx = 0;
    float ty = 0;
    float tw = 0;
    float th = 0;
    float tc = 0;
    float center_x = 0;
    float center_y = 0;
    float box_w = 0;
    float box_h = 0;
    float objectness = 0;
    Box bb;
//    uint8_t num_grid = 0;
//    uint8_t anchor_offset = 0;

    float classes[NUM_CLASS];
    float max_pred = 0;
    int32_t pred_class = -1;
    float probability = 0;
    detection d;
    /* Clear the detected result list */
    det.clear();

/*
    for (n = 0; n < NUM_INF_OUT_LAYER; n++)
    {
        num_grid = num_grids[n];
        anchor_offset = 2 * NUM_BB * (NUM_INF_OUT_LAYER - (n + 1));
*/
    for (b = 0; b < NUM_BB; b++)
    {
        for (y = 0; y < NUM_GRID_Y; y++)
        {
            for (x = 0; x < NUM_GRID_X; x++)
            {
//                offs = yolo_offset(n, b, y, x);
                offs = offset(b, y, x);
                tx = floatarr[offs];
//                ty = floatarr[yolo_index(n, offs, 1)];
//                tw = floatarr[yolo_index(n, offs, 2)];
//                th = floatarr[yolo_index(n, offs, 3)];
//                tc = floatarr[yolo_index(n, offs, 4)];
                ty = floatarr[index(offs, 1)];
                tw = floatarr[index(offs, 2)];
                th = floatarr[index(offs, 3)];
                tc = floatarr[index(offs, 4)];

                /* Compute the bounding box */
                /*get_region_box*/
                center_x = ((float)x + sigmoid(tx)) / (float)NUM_GRID_X;
                center_y = ((float)y + sigmoid(ty)) / (float)NUM_GRID_Y;

//                box_w = (float)exp(tw) * anchors[anchor_offset + 2 * b + 0] / (float)MODEL_IN_W;
//                box_h = (float)exp(th) * anchors[anchor_offset + 2 * b + 1] / (float)MODEL_IN_W;
                box_w = (float)exp(tw) * anchors[2 * b + 0] / (float)NUM_GRID_X;
                box_h = (float)exp(th) * anchors[2 * b + 1] / (float)NUM_GRID_Y;

                /* Adjustment for VGA size */
                /* correct_region_boxes */
                center_x = (center_x - (MODEL_IN_W - new_w) / 2. / MODEL_IN_W) / ((float)new_w / MODEL_IN_W);
                center_y = (center_y - (MODEL_IN_H - new_h) / 2. / MODEL_IN_H) / ((float)new_h / MODEL_IN_H);
                box_w *= (float)(MODEL_IN_W / new_w);
                box_h *= (float)(MODEL_IN_H / new_h);

                center_x = round(center_x * DRPAI_IN_WIDTH);
                center_y = round(center_y * DRPAI_IN_HEIGHT);
                box_w = round(box_w * DRPAI_IN_WIDTH);
                box_h = round(box_h * DRPAI_IN_HEIGHT);

                objectness = sigmoid(tc);

//                Box bb = {center_x, center_y, box_w, box_h};
                bb = {center_x, center_y, box_w, box_h};

                /* Get the class prediction */
                for (i = 0; i < NUM_CLASS; i++)
                {
//                    classes[i] = sigmoid(floatarr[yolo_index(n, offs, 5 + i)]);
                    classes[i] = floatarr[index(offs, 5 + i)];
                }
                softmax(classes);
                max_pred = 0;
                pred_class = -1;
                /*Get the predicted class */
                for (i = 0; i < NUM_CLASS; i++)
                {
                    if (classes[i] > max_pred)
                    {
                        pred_class = i;
                        max_pred = classes[i];
                    }
                }

                /* Store the result into the list if the probability is more than the threshold */
                probability = max_pred * objectness;
                if (probability > TH_PROB)
                {
                    d = {bb, pred_class, probability};
                    det.push_back(d);
                }
            }
        }
//        }
    }
}
/*****************************************
 * Function Name : draw_bounding_box
 * Description   : Draw bounding box on image.
 * Arguments     : -
 * Return value  : 0 if succeeded
 *               not 0 otherwise
 ******************************************/
uint32_t draw_bounding_box(FILE *fp)
{
    stringstream stream;
    string str = "";
    string result_str;
    int32_t result_cnt =0;
    uint32_t x = HEAD_COUNT_STR_X;
    uint32_t y = HEAD_COUNT_STR_X;
    /* Draw bounding box on RGB image. */
    int32_t i = 0;
    for (i = 0; i < det.size(); i++)
    {
        /* Skip the overlapped bounding boxes */
        if (det[i].prob < 0.7)
        {
            continue;
        }
        result_cnt++;
        /* Clear string stream for bounding box labels */
//        stream.str("");
        /* Draw the bounding box on the image */
//        stream << fixed << setprecision(2) << det[i].prob;
//        result_str = label_file_map[det[i].c] + " " + stream.str();

        int32_t x_min = (int)det[i].bbox.x - round((int)det[i].bbox.w / 2.);
        int32_t y_min = (int)det[i].bbox.y - round((int)det[i].bbox.h / 2.);
        int32_t x_max = (int)det[i].bbox.x + round((int)det[i].bbox.w / 2.) - 1;
        int32_t y_max = (int)det[i].bbox.y + round((int)det[i].bbox.h / 2.) - 1;

        /* Check the bounding box is in the image range */
        x_min = x_min < 1 ? 1 : x_min;
        x_max = ((DRPAI_IN_WIDTH - 2) < x_max) ? (DRPAI_IN_WIDTH - 2) : x_max;
        y_min = y_min < 1 ? 1 : y_min;
        y_max = ((DRPAI_IN_HEIGHT - 2) < y_max) ? (DRPAI_IN_HEIGHT - 2) : y_max;

//        int32_t x2_min = x_min + BOX_THICKNESS;
//        int32_t y2_min = y_min + BOX_THICKNESS;
//        int32_t x2_max = x_max - BOX_THICKNESS;
//        int32_t y2_max = y_max - BOX_THICKNESS;

//        x2_min = ((DRPAI_IN_WIDTH - 2) < x2_min) ? (DRPAI_IN_WIDTH - 2) : x2_min;
//        x2_max = x2_max < 1 ? 1 : x2_max;
//        y2_min = ((DRPAI_IN_HEIGHT - 2) < y2_min) ? (DRPAI_IN_HEIGHT - 2) : y2_min;
//        y2_max = y2_max < 1 ? 1 : y2_max;
#if 0
        Point topLeft(x_min, y_min);
        Point bottomRight(x_max, y_max);

        Point topLeft2(x2_min, y2_min);
        Point bottomRight2(x2_max, y2_max);

        /* Creating bounding box and class labels */
        /*cordinates for solid rectangle*/
        Point textleft(x_min,y_min+CLASS_LABEL_HEIGHT);
        Point textright(x_min+CLASS_LABEL_WIDTH,y_min);

        rectangle(g_frame, topLeft, bottomRight, Scalar(0, 0, 0), BOX_THICKNESS);
        rectangle(g_frame, topLeft2, bottomRight2, Scalar(255, 255, 255), BOX_THICKNESS);
        /*solid rectangle over class name */
        rectangle(g_frame, textleft, textright, Scalar(59, 94, 53), -1);
        putText(g_frame, result_str, textleft, FONT_HERSHEY_SIMPLEX, CHAR_SCALE_XS, Scalar(255, 255, 255), BOX_CHAR_THICKNESS);
#else
#if 1
        fprintf(fp, "\t{person, %f, %ld, %ld, %ld, %ld}\n,", det[i].prob, x_min, y_min, x_max, y_max);
#endif
#endif
    }
    return result_cnt++;
}