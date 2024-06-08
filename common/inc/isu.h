

#include "vspm_public.h"
#include "mmngr_user_public.h"

#ifdef __cplusplus
extern "C" {
#endif

/* git://github.com/renesas-rcar/vspmif_lib.git */
typedef struct _isu_job_t {
    struct isu_start_t *isu_start;
    struct vspm_job_t *vspm_job;
    PFN_VSPM_COMPLETE_CALLBACK cb_func;
    unsigned long job_id;
    void *handle;
    
} isu_job_t;

void* isu_init( void );

isu_job_t* create_job_yuyv_to_nv12( int width, int height );

int isu_fmt_convert( isu_job_t *vspm_job, int width, int height, int iformat, int oformat );

int start_job( isu_job_t *job_info, void* in, void* out, PFN_VSPM_COMPLETE_CALLBACK cb_func , void* parm );


void isu_destroy( isu_job_t *job );

#ifdef __cplusplus
}
#endif