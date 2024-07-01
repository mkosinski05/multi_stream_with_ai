/*
 * Copyright (c) 2018 Renesas Electronics Corporation
 * Released under the MIT license
 * http://opensource.org/licenses/mit-license.php
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <unistd.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <pthread.h>


#include "isu.h"



void* isu_init( void )
{
    long ercd;
    struct vspm_init_t init_par;
    void *phandle;

    /* init vsp manager */
	memset(&init_par, 0, sizeof(struct vspm_init_t));
	init_par.use_ch = VSPM_EMPTY_CH;
	init_par.mode = VSPM_MODE_OCCUPY;
	init_par.type = VSPM_TYPE_ISU_AUTO;

	ercd = vspm_init_driver(&phandle, &init_par);
	if (ercd != R_VSPM_OK) {
		printf("Error: failed to vspm_init_driver() ercd=%d\n", (int)ercd);
		return NULL;
	}

    return phandle;
}

isu_job_t* create_job_yuyv_to_nv12( int width, int height )
{
    isu_job_t *job;

    /******************************************************************************
     *           Allocate Job Structure 
     ******************************************************************************/
    job = malloc(sizeof(isu_job_t));
	if (job == NULL) {
		printf("Error: failed to allocate memory!!\n");
        free(job);
		return NULL;
	}
    /******************************************************************************
     *           Allocate ISU Start Structure 
     ******************************************************************************/
    job->isu_start = malloc(sizeof(struct isu_start_t));
	if (job->isu_start == NULL) {
		printf("Error: failed to allocate memory!!\n");
        free(job->isu_start);
        free(job);
		return NULL;
	}

    /******************************************************************************
     *  Allocate ISU Source Structure 
     ******************************************************************************/
    struct isu_src_t *p_src = malloc(sizeof(struct isu_src_t));
	if (p_src == NULL) {
		printf("Error: failed to allocate memory!!\n");
		return NULL;
	}
    p_src->stride = width * 2;
    p_src->stride_c = 0;
    p_src->width = width;
    p_src->height = height;
    p_src->format = 0x21; /* YCbCr422-8bit alias YUYV */
    p_src->swap = 0; //(ISU_SWAP_B|ISU_SWAP_W|ISU_SWAP_L|ISU_SWAP_LL);
    p_src->td = NULL;
    p_src->alpha = NULL;
    p_src->uv_conv = ISU_UV_CONV_OFF;

    job->isu_start->src_par = p_src;
    job->isu_start->dl_hard_addr = 0ul;

    /* Allocate ISU Alpha Structure for Source*/
    struct isu_alpha_unit_t *p_alpha = malloc(sizeof(struct isu_alpha_unit_t));
	if (p_alpha == NULL) {
		printf("Error: failed to allocate memory!!\n");
		return NULL;
	}
    memset (p_alpha, 0, sizeof(struct isu_alpha_unit_t));
    p_src->alpha = p_alpha;

    /****************************************************************************** 
    *       Allocate ISU Destination Structure 
    *******************************************************************************/
    struct isu_dst_t *p_des = malloc(sizeof(struct isu_dst_t));
	if (p_des == NULL) {
		printf("Error: failed to allocate memory!!\n");
		return NULL;
	}
    p_des->stride = p_src->stride/2;
    // For NV12 this must be non-zero and multiple of 32
    p_des->stride_c = p_src->stride/2;
    p_des->format = 0x23; // YCbCr420-8bit alias NV12 format
    p_des->swap = ISU_SWAP_NO;
    p_des->csc = NULL; // TODO this cannot be NULL

    /* Allocate ISU Alpha Structure for Destination */
    struct isu_alpha_unit_t *p_alpha2 = malloc(sizeof(struct isu_alpha_unit_t));
	if (p_alpha2 == NULL) {
		printf("Error: failed to allocate memory!!\n");
		return NULL;
	}
    memset (p_alpha2, 0, sizeof(struct isu_alpha_unit_t));
    p_des->alpha = p_alpha2;

    /* Allocate ISU CSC Structure for Destination */
    struct isu_csc_t *p_csc = malloc(sizeof(struct isu_csc_t));
	if (p_csc == NULL) {
		printf("Error: failed to allocate memory!!\n");
		return NULL;
	}
    memset (p_csc, 0, sizeof(struct isu_csc_t));
    p_csc->csc = ISU_CSC_CUSTOM;
    
    p_csc->k_matrix[0][0] = 0x400; // ISU_WPF_MUL1 0000_0400h K11 = 400h
    p_csc->k_matrix[1][1] = 0x400; // ISU_WPF_MUL4 0400_0000h K22 = 400h
    p_csc->k_matrix[2][2] = 0x400; // ISU_WPF_MUL6 0000_0400h K33 = 400h
  
    p_csc->clip[0][0] = 0x00; // CLP (MAX/MIN)_A = FFh, 00h
    p_csc->clip[0][1] = 0xFF;
    p_csc->clip[1][0] = 0x00; // CLP (MAX/MIN)_B = FFh, 00h
    p_csc->clip[1][1] = 0xFF;
    p_csc->clip[2][0] = 0x00; // CLP (MAX/MIN)_C = FFh, 00h
    p_csc->clip[2][1] = 0xFF;
    p_des->csc = p_csc;

    job->isu_start->dst_par = p_des;

    job->isu_start->rs_par = NULL; /* No Scaling*/

    /****************************************************************************** 
    *       Allocate Job Info Structure 
    *******************************************************************************/
    struct vspm_job_t *vspm_job = malloc(sizeof(struct vspm_job_t));
	if (vspm_job == NULL) {
		printf("Error: failed to allocate memory!!\n");
        free(vspm_job);
		return NULL;
	}
    vspm_job->type = VSPM_TYPE_ISU_AUTO;
    vspm_job->par.isu = job->isu_start;
    job->vspm_job = vspm_job;

    return job;

}
int start_job( isu_job_t *job_info, void* in, void* out, PFN_VSPM_COMPLETE_CALLBACK cb_func , void* parm )
{
    int ret = 0;
    long ercd;
    struct isu_src_t *pSrc_parm = job_info->isu_start->src_par;
    struct isu_dst_t *pDes_parm = job_info->isu_start->dst_par;

    if ( NULL == cb_func || NULL == in || NULL == out || NULL == job_info ) {
        return -1;
    }
    
    /******************************************************************************
    *  Set Job Buffers
    *******************************************************************************/
    /* Set Input and Output Buffers */
   pSrc_parm->addr = (uint64_t)in;
   pSrc_parm->addr_c = (uint64_t)NULL;
   pDes_parm->addr = (uint64_t)out;
   pDes_parm->addr_c = (uint64_t)(out + (pSrc_parm->width * pSrc_parm->height));

   
   /******************************************************************************
    *   Queue Job
    *******************************************************************************/
	ercd = vspm_entry_job(job_info->handle, &job_info->job_id, 126, job_info->vspm_job, (void *)parm, cb_func);
	if (ercd != R_VSPM_OK) {
		printf("Error: failed to vspm_entry_job() ercd=%d\n", (int)ercd);
		return -1;
	}

    return ret;
}

void isu_destroy( isu_job_t *job )
{

    (void)vspm_quit_driver( job->handle );
    free(job->isu_start->src_par);
    free(job->isu_start->dst_par);
    free(job->isu_start);

}