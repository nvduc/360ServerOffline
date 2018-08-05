#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <string.h>

#include "common.h"
#include "AdaptationLogic.h"
#include "Viewport.h"
#include "Metadata.h"
#include "GaussianFilter.h"

int frame_id = 0;
// static int decision_id = 0;
int GOP_SIZE = 16;
int NUM_FRAME_SKIP = 180;
bool SKIP_FRAME = true;
bool SKIP_FRAME_2 = false;
int last_requested_size = -1;
int erpSize = 0;
int dec_id = 0;
int cur_dec_id = 0;
double* tileSize = NULL; 
pthread_t decsion_engine_thread;
int AI;
double* decision_t;
double* decision_avail_t;
double* frame_send_t;
int tile_size[64][9][10000];
int DASH_frame_size[10000][9];
double DASH_frame_psnr[10000][9];
int No_frame; 
int selectVer = -1;
double bandwidth[1000];
//
//
AdaptationLogic *adaptLogic;
Viewport *viewport;
Metadata *metadata;
//
struct timeval t_sess_start;
struct timeval t_now;
//
int rows;
int cols;
int tiles;
//
FILE *log_tile_ver;
FILE *log_tile_ver_linear;
FILE *log_dec;
FILE *log_frame_psnr;
char fname[200]; 
int max_bw;
// adaptation logic parameters
struct ViewportParam
{
  double Fh;// Horizontal FoV (rad)
  double Fv;// Vertical FoV (rad)
  int vp_W;// Viewport's width (Number of pixels)
  int vp_H;// Viewport's height (Number of pixels)
  int erp_W;// ERP's width
  int erp_H;// ERP's height
  int tile_W;// tile's width
  int tile_H;// tile's height
  double SD; // segment duration in secs
  int SESS_DUR; // session duration in secs
  int NO_VER;// Number of available versions
  int FPS;// Frame per second
  int NO_SEG;// Number of segment from trace data
  int BUFFSIZE; // buffer size as number of frames
  int NO_SEG_FULL;// Numer of segments in the sessions
  int INTERVAL;// adaptation interval in frames
  int METHOD;// adaptation method
  double margin;// bandwidth estimation margin
  const char* head_pos_trace;  // head position trace
  double speed;// head movement speed
  double delay; // network delay
  int NO_FRAME;// total number of frames in a session.
  int NO_FRAME_ORIGIN;// total number of frames in the considered video.
  string head_trace;
  string est_head_trace;
  string DASH_frame_info;
  string tile_frame_info;
  string videoname;
  int TRACE;
  int VP_EST;
  int VP_EST_METHOD;
};
struct ViewportParam vpParam;
/* define functions */
struct ViewportParam read_viewport_info(char* filename);
void init();
void write_result();

int main(int argc, char* argv[]){
  printf("Hello, server_offline\n");
  fflush(NULL);
  int i, j,k, METHOD, INTER, bw_id;
  //int BW_LIST[] = {2000, 6000, 8000, 10000}; //kbps
  int BW_LIST[] = {10000};
  int BW_NUM = sizeof(BW_LIST)/sizeof(BW_LIST[0]);
  //int METHOD_LIST[] = {0,1,5,20,21};
  int METHOD_LIST[] = {1, 20, 21};
  int METHOD_NUM = sizeof(METHOD_LIST) / sizeof(METHOD_LIST[0]);
  int INTER_LIST[] = {32};
  int INTER_NUM = sizeof(INTER_LIST) / sizeof(INTER_LIST[0]);
  string TRACE_LIST[] = {"head_trace/xyz_vid_5_uid_0.txt","head_trace/xyz_vid_5_uid_1.txt", "head_trace/xyz_vid_5_uid_2.txt", "head_trace/xyz_vid_5_uid_3.txt","head_trace/xyz_vid_5_uid_4.txt","head_trace/xyz_vid_5_uid_5.txt","head_trace/xyz_vid_5_uid_7.txt", "head_trace/xyz_vid_5_uid_8.txt","head_trace/xyz_vid_5_uid_9.txt","head_trace/xyz_vid_5_uid_10.txt","head_trace/xyz_vid_5_uid_11.txt", "head_trace/xyz_vid_5_uid_12.txt", "head_trace/xyz_vid_5_uid_13.txt","head_trace/xyz_vid_5_uid_14.txt", "head_trace/xyz_vid_5_uid_15.txt", "head_trace/xyz_vid_5_uid_17.txt", "head_trace/xyz_vid_5_uid_18.txt", "head_trace/xyz_vid_5_uid_19.txt", "head_trace/xyz_vid_5_uid_20.txt", "head_trace/xyz_vid_5_uid_21.txt", "head_trace/xyz_vid_5_uid_22.txt", "head_trace/xyz_vid_5_uid_23.txt", "head_trace/xyz_vid_5_uid_24.txt", "head_trace/xyz_vid_5_uid_25.txt", "head_trace/xyz_vid_5_uid_26.txt", "head_trace/xyz_vid_5_uid_27.txt", "head_trace/xyz_vid_5_uid_28.txt", "head_trace/xyz_vid_5_uid_29.txt", "head_trace/xyz_vid_5_uid_30.txt", "head_trace/xyz_vid_5_uid_31.txt", "head_trace/xyz_vid_5_uid_32.txt", "head_trace/xyz_vid_5_uid_33.txt", "head_trace/xyz_vid_5_uid_34.txt", "head_trace/xyz_vid_5_uid_35.txt", "head_trace/xyz_vid_5_uid_36.txt", "head_trace/xyz_vid_5_uid_38.txt", "head_trace/xyz_vid_5_uid_39.txt", "head_trace/xyz_vid_5_uid_40.txt", "head_trace/xyz_vid_5_uid_41.txt", "head_trace/xyz_vid_5_uid_42.txt", "head_trace/xyz_vid_5_uid_44.txt", "head_trace/xyz_vid_5_uid_46.txt", "head_trace/xyz_vid_5_uid_47.txt", "head_trace/xyz_vid_5_uid_48.txt", "head_trace/xyz_vid_5_uid_49.txt", "head_trace/xyz_vid_5_uid_50.txt", "head_trace/xyz_vid_5_uid_51.txt", "head_trace/xyz_vid_5_uid_52.txt", "head_trace/xyz_vid_5_uid_53.txt", "head_trace/xyz_vid_5_uid_54.txt", "head_trace/xyz_vid_5_uid_56.txt","head_trace/xyz_vid_5_uid_57.txt", "head_trace/xyz_vid_5_uid_58.txt", "head_trace/xyz_vid_5_uid_59.txt", "head_trace/xyz_vid_5_uid_60.txt", "head_trace/xyz_vid_5_uid_61.txt", "head_trace/xyz_vid_5_uid_62.txt",
    "head_trace/xyz_vid_1_uid_0.txt","head_trace/xyz_vid_1_uid_10.txt","head_trace/xyz_vid_1_uid_11.txt","head_trace/xyz_vid_1_uid_12.txt","head_trace/xyz_vid_1_uid_13.txt","head_trace/xyz_vid_1_uid_14.txt","head_trace/xyz_vid_1_uid_15.txt","head_trace/xyz_vid_1_uid_17.txt","head_trace/xyz_vid_1_uid_18.txt","head_trace/xyz_vid_1_uid_19.txt","head_trace/xyz_vid_1_uid_1.txt","head_trace/xyz_vid_1_uid_20.txt","head_trace/xyz_vid_1_uid_21.txt","head_trace/xyz_vid_1_uid_22.txt","head_trace/xyz_vid_1_uid_23.txt","head_trace/xyz_vid_1_uid_24.txt","head_trace/xyz_vid_1_uid_25.txt","head_trace/xyz_vid_1_uid_26.txt","head_trace/xyz_vid_1_uid_27.txt","head_trace/xyz_vid_1_uid_28.txt","head_trace/xyz_vid_1_uid_29.txt","head_trace/xyz_vid_1_uid_2.txt","head_trace/xyz_vid_1_uid_30.txt","head_trace/xyz_vid_1_uid_31.txt","head_trace/xyz_vid_1_uid_32.txt","head_trace/xyz_vid_1_uid_33.txt","head_trace/xyz_vid_1_uid_34.txt","head_trace/xyz_vid_1_uid_35.txt","head_trace/xyz_vid_1_uid_36.txt","head_trace/xyz_vid_1_uid_38.txt","head_trace/xyz_vid_1_uid_39.txt","head_trace/xyz_vid_1_uid_3.txt","head_trace/xyz_vid_1_uid_40.txt","head_trace/xyz_vid_1_uid_41.txt","head_trace/xyz_vid_1_uid_42.txt","head_trace/xyz_vid_1_uid_44.txt","head_trace/xyz_vid_1_uid_46.txt","head_trace/xyz_vid_1_uid_47.txt","head_trace/xyz_vid_1_uid_48.txt","head_trace/xyz_vid_1_uid_49.txt","head_trace/xyz_vid_1_uid_4.txt","head_trace/xyz_vid_1_uid_50.txt","head_trace/xyz_vid_1_uid_51.txt","head_trace/xyz_vid_1_uid_52.txt","head_trace/xyz_vid_1_uid_53.txt","head_trace/xyz_vid_1_uid_54.txt","head_trace/xyz_vid_1_uid_56.txt","head_trace/xyz_vid_1_uid_57.txt","head_trace/xyz_vid_1_uid_58.txt","head_trace/xyz_vid_1_uid_59.txt","head_trace/xyz_vid_1_uid_5.txt","head_trace/xyz_vid_1_uid_60.txt","head_trace/xyz_vid_1_uid_61.txt","head_trace/xyz_vid_1_uid_62.txt","head_trace/xyz_vid_1_uid_7.txt","head_trace/xyz_vid_1_uid_8.txt","head_trace/xyz_vid_1_uid_9.txt","head_trace/xyz_vid_2_uid_0.txt","head_trace/xyz_vid_2_uid_10.txt","head_trace/xyz_vid_2_uid_11.txt","head_trace/xyz_vid_2_uid_12.txt","head_trace/xyz_vid_2_uid_13.txt","head_trace/xyz_vid_2_uid_14.txt","head_trace/xyz_vid_2_uid_15.txt","head_trace/xyz_vid_2_uid_17.txt","head_trace/xyz_vid_2_uid_18.txt","head_trace/xyz_vid_2_uid_19.txt","head_trace/xyz_vid_2_uid_1.txt","head_trace/xyz_vid_2_uid_20.txt","head_trace/xyz_vid_2_uid_21.txt","head_trace/xyz_vid_2_uid_22.txt","head_trace/xyz_vid_2_uid_23.txt","head_trace/xyz_vid_2_uid_24.txt","head_trace/xyz_vid_2_uid_25.txt","head_trace/xyz_vid_2_uid_26.txt","head_trace/xyz_vid_2_uid_27.txt","head_trace/xyz_vid_2_uid_28.txt","head_trace/xyz_vid_2_uid_29.txt","head_trace/xyz_vid_2_uid_2.txt","head_trace/xyz_vid_2_uid_30.txt","head_trace/xyz_vid_2_uid_31.txt","head_trace/xyz_vid_2_uid_32.txt","head_trace/xyz_vid_2_uid_33.txt","head_trace/xyz_vid_2_uid_34.txt","head_trace/xyz_vid_2_uid_35.txt","head_trace/xyz_vid_2_uid_36.txt","head_trace/xyz_vid_2_uid_38.txt","head_trace/xyz_vid_2_uid_39.txt","head_trace/xyz_vid_2_uid_3.txt","head_trace/xyz_vid_2_uid_40.txt","head_trace/xyz_vid_2_uid_41.txt","head_trace/xyz_vid_2_uid_42.txt","head_trace/xyz_vid_2_uid_44.txt","head_trace/xyz_vid_2_uid_46.txt","head_trace/xyz_vid_2_uid_47.txt","head_trace/xyz_vid_2_uid_48.txt","head_trace/xyz_vid_2_uid_49.txt","head_trace/xyz_vid_2_uid_4.txt","head_trace/xyz_vid_2_uid_50.txt","head_trace/xyz_vid_2_uid_51.txt","head_trace/xyz_vid_2_uid_52.txt","head_trace/xyz_vid_2_uid_53.txt","head_trace/xyz_vid_2_uid_54.txt","head_trace/xyz_vid_2_uid_56.txt","head_trace/xyz_vid_2_uid_57.txt","head_trace/xyz_vid_2_uid_58.txt","head_trace/xyz_vid_2_uid_59.txt","head_trace/xyz_vid_2_uid_5.txt","head_trace/xyz_vid_2_uid_60.txt","head_trace/xyz_vid_2_uid_61.txt","head_trace/xyz_vid_2_uid_62.txt","head_trace/xyz_vid_2_uid_7.txt","head_trace/xyz_vid_2_uid_8.txt","head_trace/xyz_vid_2_uid_9.txt","head_trace/xyz_vid_4_uid_0.txt","head_trace/xyz_vid_4_uid_10.txt","head_trace/xyz_vid_4_uid_11.txt","head_trace/xyz_vid_4_uid_12.txt","head_trace/xyz_vid_4_uid_15.txt","head_trace/xyz_vid_4_uid_17.txt","head_trace/xyz_vid_4_uid_19.txt","head_trace/xyz_vid_4_uid_1.txt","head_trace/xyz_vid_4_uid_21.txt","head_trace/xyz_vid_4_uid_23.txt","head_trace/xyz_vid_4_uid_24.txt","head_trace/xyz_vid_4_uid_25.txt","head_trace/xyz_vid_4_uid_26.txt","head_trace/xyz_vid_4_uid_27.txt","head_trace/xyz_vid_4_uid_28.txt","head_trace/xyz_vid_4_uid_30.txt","head_trace/xyz_vid_4_uid_32.txt","head_trace/xyz_vid_4_uid_33.txt","head_trace/xyz_vid_4_uid_34.txt","head_trace/xyz_vid_4_uid_35.txt","head_trace/xyz_vid_4_uid_36.txt","head_trace/xyz_vid_4_uid_38.txt","head_trace/xyz_vid_4_uid_39.txt","head_trace/xyz_vid_4_uid_40.txt","head_trace/xyz_vid_4_uid_42.txt","head_trace/xyz_vid_4_uid_44.txt","head_trace/xyz_vid_4_uid_45.txt","head_trace/xyz_vid_4_uid_46.txt","head_trace/xyz_vid_4_uid_48.txt","head_trace/xyz_vid_4_uid_51.txt","head_trace/xyz_vid_4_uid_52.txt","head_trace/xyz_vid_4_uid_57.txt","head_trace/xyz_vid_4_uid_59.txt","head_trace/xyz_vid_4_uid_60.txt","head_trace/xyz_vid_4_uid_62.txt","head_trace/xyz_vid_4_uid_7.txt","head_trace/xyz_vid_4_uid_8.txt","head_trace/xyz_vid_3_uid_0.txt","head_trace/xyz_vid_3_uid_10.txt","head_trace/xyz_vid_3_uid_11.txt","head_trace/xyz_vid_3_uid_12.txt","head_trace/xyz_vid_3_uid_13.txt","head_trace/xyz_vid_3_uid_14.txt","head_trace/xyz_vid_3_uid_15.txt","head_trace/xyz_vid_3_uid_17.txt","head_trace/xyz_vid_3_uid_18.txt","head_trace/xyz_vid_3_uid_19.txt","head_trace/xyz_vid_3_uid_1.txt","head_trace/xyz_vid_3_uid_20.txt","head_trace/xyz_vid_3_uid_21.txt","head_trace/xyz_vid_3_uid_22.txt","head_trace/xyz_vid_3_uid_23.txt","head_trace/xyz_vid_3_uid_24.txt","head_trace/xyz_vid_3_uid_25.txt","head_trace/xyz_vid_3_uid_26.txt","head_trace/xyz_vid_3_uid_27.txt","head_trace/xyz_vid_3_uid_28.txt","head_trace/xyz_vid_3_uid_29.txt","head_trace/xyz_vid_3_uid_2.txt","head_trace/xyz_vid_3_uid_30.txt","head_trace/xyz_vid_3_uid_31.txt","head_trace/xyz_vid_3_uid_32.txt","head_trace/xyz_vid_3_uid_33.txt","head_trace/xyz_vid_3_uid_34.txt","head_trace/xyz_vid_3_uid_35.txt","head_trace/xyz_vid_3_uid_36.txt","head_trace/xyz_vid_3_uid_38.txt","head_trace/xyz_vid_3_uid_39.txt","head_trace/xyz_vid_3_uid_3.txt","head_trace/xyz_vid_3_uid_40.txt","head_trace/xyz_vid_3_uid_41.txt","head_trace/xyz_vid_3_uid_42.txt","head_trace/xyz_vid_3_uid_44.txt","head_trace/xyz_vid_3_uid_46.txt","head_trace/xyz_vid_3_uid_47.txt","head_trace/xyz_vid_3_uid_48.txt","head_trace/xyz_vid_3_uid_49.txt","head_trace/xyz_vid_3_uid_4.txt","head_trace/xyz_vid_3_uid_50.txt","head_trace/xyz_vid_3_uid_51.txt","head_trace/xyz_vid_3_uid_52.txt","head_trace/xyz_vid_3_uid_53.txt","head_trace/xyz_vid_3_uid_54.txt","head_trace/xyz_vid_3_uid_56.txt","head_trace/xyz_vid_3_uid_57.txt","head_trace/xyz_vid_3_uid_58.txt","head_trace/xyz_vid_3_uid_59.txt","head_trace/xyz_vid_3_uid_5.txt","head_trace/xyz_vid_3_uid_60.txt","head_trace/xyz_vid_3_uid_61.txt","head_trace/xyz_vid_3_uid_62.txt","head_trace/xyz_vid_3_uid_7.txt","head_trace/xyz_vid_3_uid_8.txt","head_trace/xyz_vid_3_uid_9.txt"};
  string TRACE_LIST_EST[] = {"head_trace/est_vp_trace/xyz_vid_5_uid_0_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_1_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_2_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_3_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_4_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_5_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_7_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_8_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_9_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_10_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_11_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_12_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_13_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_14_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_15_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_17_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_18_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_19_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_20_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_21_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_22_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_23_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_24_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_25_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_26_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_27_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_28_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_29_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_30_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_31_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_32_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_33_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_34_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_35_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_36_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_38_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_39_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_40_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_41_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_42_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_44_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_46_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_47_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_48_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_49_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_50_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_51_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_52_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_53_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_54_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_56_est.txt","head_trace/est_vp_trace/xyz_vid_5_uid_57_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_58_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_59_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_60_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_61_est.txt", "head_trace/est_vp_trace/xyz_vid_5_uid_62_est.txt"};
  int TRACE_ID[] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 38, 39, 40, 41, 42, 44, 46, 47, 48, 49, 50, 51, 52, 53, 54, 56, 57};
  int TRACE_NUM = sizeof(TRACE_LIST) / sizeof(TRACE_LIST[0]);
  //int TRACE_NUM = 57;
  int s_trace_id, e_trace_id;
  if(argc != 3){
    printf("usage: ./server_offline start_trace_id end_trace_id\n");
    return 0;
  }
  s_trace_id = atoi(argv[1]);
  e_trace_id = atoi(argv[2]);
  printf("s_trace_id: %d, e_trace_id: %d\n", s_trace_id, e_trace_id);
  /* initialize functions and variables */
  vpParam = read_viewport_info("data/Viewport.cfg");
  printf("Hello, server_offline\n");
  //  for(dec_id = 0; dec_id < vpParam.NO_SEG_FULL; dec_id++){
  for(k=s_trace_id; k <= e_trace_id; k++){
    vpParam.head_trace = TRACE_LIST[k];
    cout << vpParam.head_trace << endl;
    if(vpParam.VP_EST == 1){
      char tmp_buff[1024];
      sprintf(tmp_buff, "head_trace/est_vp_trace/xyz_vid_5_uid_%d_INTER_%d_BUFF_%d_est.txt",TRACE_ID[k], vpParam.INTERVAL, vpParam.BUFFSIZE);
      vpParam.est_head_trace = tmp_buff;
      cout << vpParam.est_head_trace <<endl;
    }
    else
      vpParam.est_head_trace =  vpParam.head_trace;
    vpParam.TRACE = k+1;
    for(i=0; i < INTER_NUM; i++){
      vpParam.INTERVAL = INTER_LIST[i];
      //vpParam.BUFFSIZE = INTER_LIST[i];
      vpParam.NO_SEG = vpParam.NO_FRAME_ORIGIN / vpParam.INTERVAL;
      vpParam.NO_SEG_FULL = vpParam.NO_FRAME / vpParam.INTERVAL;
      No_frame = vpParam.NO_FRAME_ORIGIN;
      GOP_SIZE = vpParam.INTERVAL;
      for(j=0; j < METHOD_NUM; j++){
        vpParam.METHOD = METHOD_LIST[j];
        /* load head/bandwidth traces */
        init();
        /* run adaptation */
        printf("#[DUC] NO_SEG_FULL=%d\n", vpParam.NO_SEG_FULL);
        for(bw_id = 0; bw_id < BW_NUM; bw_id++){
          for(dec_id = 0; dec_id < vpParam.NO_SEG_FULL; dec_id++){
            /* set througput */
            adaptLogic->seg_thrp[dec_id] = BW_LIST[bw_id];
            if(vpParam.METHOD == 0){
              adaptLogic->getNextSegmentDASH(dec_id);
            }else
              tileSize = adaptLogic->getNextSegment(dec_id);
          }
          write_result();
        }
      }
    }
  }
  return 1;
}
void init(){
  // init adaptation logic
  printf("#[init]:\n");
  viewport = new Viewport(vpParam.Fh, vpParam.Fv, vpParam.vp_W, vpParam.vp_H, vpParam.erp_W, vpParam.erp_H, vpParam.tile_W, vpParam.tile_H);
  metadata = new Metadata(viewport, vpParam.SD, vpParam.SESS_DUR, vpParam.NO_VER, vpParam.FPS, vpParam.NO_SEG, vpParam.BUFFSIZE, vpParam.NO_SEG_FULL, vpParam.head_trace.c_str(), vpParam.INTERVAL, vpParam.BUFFSIZE);
  metadata->speed = vpParam.speed;
  metadata->delay = vpParam.delay;
  adaptLogic = new AdaptationLogic(metadata, vpParam.INTERVAL, vpParam.METHOD, vpParam.margin);//eror
  adaptLogic->VP_EST_METHOD = vpParam.VP_EST_METHOD;
  rows = viewport->No_tile_h;
  cols = viewport->No_tile_v;
  tiles = viewport->No_tile;
  AI = (int) vpParam.INTERVAL * vpParam.SD * 1000 * 1000;
  printf("AI=%d(usec)\n", AI);
  //
  decision_t = new double[vpParam.NO_SEG_FULL];
  decision_avail_t = new double[vpParam.NO_SEG_FULL];
  frame_send_t = new double[vpParam.NO_SEG_FULL * vpParam.INTERVAL];
  // load tile size per frame
  char buff[1000];
  vector <double> v;
  int rows_;
  int cols_;
  int tid;
  int frame_id;
  int ver_id;
  int dec_id;
  int No_sample_read;
  for(tid = 0; tid < viewport->No_tile; tid++){
    sprintf(buff, "%s/tile_%d_frame.txt",vpParam.tile_frame_info.c_str(), tid);
    metadata->import_matrix_from_txt_file(buff, v, rows_, cols_);
    for(frame_id = 0; frame_id < No_frame; frame_id ++){
      for(ver_id = 0; ver_id < vpParam.NO_VER; ver_id ++){
        tile_size[tid][ver_id][frame_id] = v[frame_id * vpParam.NO_VER + ver_id];
      }
    }
  }
  printf("tile_size[0][0][0]=%d\n", tile_size[0][0][0]);
  /* load erp size per frame of each DASH version */
  sprintf(buff,"%s", vpParam.DASH_frame_info.c_str());
  metadata->import_matrix_from_txt_file(buff, v, rows_, cols_);
  for(frame_id = 0; frame_id < No_frame; frame_id ++){
    for(ver_id = 0; ver_id < vpParam.NO_VER; ver_id ++){
      DASH_frame_size[frame_id][ver_id] = v[frame_id * vpParam.NO_VER * 2 + 2*ver_id];
      DASH_frame_psnr[frame_id][ver_id] = v[frame_id * vpParam.NO_VER * 2 + 2*ver_id + 1];
    }
  }
  printf("%.2f\t%.2f\t%.2f\n", DASH_frame_psnr[0][0], DASH_frame_psnr[0][1], DASH_frame_psnr[0][3]);

  /* load estimated head positions */
  //  sprintf(buff, "data/sin_speed_50_60s.txt");
  /*
     printf("#Load estimated head trace: %sa\n", vpParam.est_head_trace.c_str());
     sprintf(buff, "%s", vpParam.est_head_trace.c_str());
     metadata->import_matrix_from_txt_file(buff, v, rows_, cols_);
     for(frame_id = 0; frame_id < metadata->NO_SEG_FULL * vpParam.INTERVAL; frame_id ++){
     adaptLogic->est_vp_frame[frame_id][0] = v[frame_id * 2];
     adaptLogic->est_vp_frame[frame_id][1] = v[frame_id * 2 + 1];
     if(adaptLogic->est_vp_frame[frame_id][1] > 90)
     adaptLogic->est_vp_frame[frame_id][1] -= 180;
     }
     */
  /* load head trace */
  //  sprintf(buff, vpParam.head_trace);
  sprintf(buff, "%s", vpParam.head_trace.c_str());
  metadata->import_matrix_from_txt_file(buff, v, rows_, cols_);
  if(rows_ - 1 > metadata->NO_SEG_FULL * vpParam.INTERVAL)
    No_sample_read = metadata->NO_SEG_FULL * vpParam.INTERVAL;
  else
    No_sample_read = rows_ - 1;
  for(frame_id = 0; frame_id < No_sample_read; frame_id ++){
    adaptLogic->frame_vp[frame_id][0] = v[(frame_id + 1) * 2]; // ignore the first frame
    adaptLogic->frame_vp[frame_id][1] = v[(frame_id + 1) * 2 + 1];
  }
  if(rows_-1 <  metadata->NO_SEG_FULL * vpParam.INTERVAL){
    for(frame_id = rows_-1; frame_id < metadata->NO_SEG_FULL * vpParam.INTERVAL; frame_id ++){
      adaptLogic->frame_vp[frame_id][0] = adaptLogic->frame_vp[frame_id%(rows_-1)][0];
      adaptLogic->frame_vp[frame_id][1] = adaptLogic->frame_vp[frame_id%(rows_-1)][1];
    }
  }
  // load bandwidth trace
  sprintf(buff, "%s", "data/bw_trace_4G.txt");
  metadata->import_matrix_from_txt_file(buff, v, rows_, cols_);
  for(dec_id = 0; dec_id < metadata->NO_SEG_FULL; dec_id ++){
    bandwidth[dec_id] = v[dec_id * 2 + 1];
  }
}
ViewportParam read_viewport_info(char* filename){
  printf("#[read_viewport_info]:\n");
  ViewportParam vpParam;
  string s;
  string delimiter = "=";
  string comment = "#";
  string key;
  string val_str;
  double val;
  size_t pos_deli = 0;
  size_t pos_comm;
  ifstream infile(filename);
  if(infile == NULL){
    cout << "Cannot open file " << filename << endl;
  }else{
    cout << "Reading file" << endl;
  }
  while(std::getline(infile, s)){
    cout << s << endl;
    if((pos_deli=s.find(delimiter)) != std::string::npos){
      key = s.substr(0, pos_deli-1);
      pos_comm = s.find(comment);
      if(pos_comm != std::string::npos){
        val_str = s.substr(pos_deli + 2, pos_comm-pos_deli-2);
      }else{
        val_str = s.substr(pos_deli + 2, s.length());
      }
      //      if(key.compare("head_pos_trace") != 0)//error here
      //	val = std::stod(val_str);
      //      cout << key << ":" << val << endl;
    }
    printf("%sa\n", val_str.c_str());
    if(key.compare("Fh")==0) vpParam.Fh = std::stod(val_str) * M_PI / 180;
    if(key.compare("Fv")==0) vpParam.Fv= std::stod(val_str) * M_PI / 180;
    if(key.compare("vp_W")==0) vpParam.vp_W = (int) std::stod(val_str);
    if(key.compare("vp_H")==0) vpParam.vp_H = (int) std::stod(val_str);
    if(key.compare("erp_W")==0) vpParam.erp_W = (int) std::stod(val_str);
    if(key.compare("erp_H")==0) vpParam.erp_H = (int) std::stod(val_str);
    if(key.compare("tile_W")==0) vpParam.tile_W = (int) std::stod(val_str);
    if(key.compare("tile_H")==0) vpParam.tile_H = (int) std::stod(val_str);
    if(key.compare("SD")==0) vpParam.SD = std::stod(val_str);
    if(key.compare("SESS_DUR")==0) vpParam.SESS_DUR = (int) std::stod(val_str);
    if(key.compare("NO_VER")==0) vpParam.NO_VER = (int) std::stod(val_str);
    if(key.compare("FPS")==0) vpParam.FPS = (int) std::stod(val_str);
    if(key.compare("NO_SEG")==0) vpParam.NO_SEG = (int) std::stod(val_str);
    if(key.compare("BUFFSIZE")==0) vpParam.BUFFSIZE = std::stod(val_str);
    if(key.compare("NO_SEG_FULL")==0) vpParam.NO_SEG_FULL = (int) std::stod(val_str);
    if(key.compare("INTERVAL")==0) vpParam.INTERVAL = (int) std::stod(val_str);
    if(key.compare("METHOD")==0) vpParam.METHOD = (int) std::stod(val_str);
    if(key.compare("margin")==0) vpParam.margin = std::stod(val_str);
    if(key.compare("speed") == 0) vpParam.speed = std::stod(val_str);
    if(key.compare("delay") == 0) vpParam.delay = std::stod(val_str);
    if(key.compare("NO_FRAME") == 0) vpParam.NO_FRAME = (int) std::stod(val_str);
    if(key.compare("NO_FRAME_ORIGIN") == 0) vpParam.NO_FRAME_ORIGIN = (int) std::stod(val_str);
    if(key.compare("VP_EST") == 0) vpParam.VP_EST = (int) std::stod(val_str);
    if(key.compare("VP_EST_METHOD") == 0) vpParam.VP_EST_METHOD = (int) std::stod(val_str);
    // if(key.compare("head_pos_trace") == 0) {
    //   vpParam.head_pos_trace = val_str.c_str();
    //   cout << vpParam.head_pos_trace << endl;
    // }
    if(key.compare("head_trace") == 0) vpParam.head_trace = val_str;
    if(key.compare("est_head_trace") == 0) {
      vpParam.est_head_trace = val_str;
      printf("est_head_trace: %s\n", vpParam.est_head_trace.c_str());
    }
    if(key.compare("DASH_frame_info") == 0) vpParam.DASH_frame_info = val_str;
    if(key.compare("tile_frame_info") == 0) vpParam.tile_frame_info = val_str;
    if(key.compare("video_name") == 0) vpParam.videoname = val_str;
  }
  vpParam.NO_SEG = vpParam.NO_FRAME_ORIGIN / vpParam.INTERVAL;
  vpParam.NO_SEG_FULL = vpParam.NO_FRAME / vpParam.INTERVAL;
  No_frame = vpParam.NO_FRAME_ORIGIN;
  GOP_SIZE = vpParam.INTERVAL;
  printf("[DUC][read_viewport_info]: NO_FRAME_ORIGIN:%d NO_SEG: %d NO_SEG_FULL: %d\n",No_frame, vpParam.NO_SEG, vpParam.NO_SEG_FULL);
  return vpParam;
}
void write_result(){
  int jj=0; // frame id
  double BR = 0;
  int tid;
  // generate log files
  sprintf(fname, "result/video_%s/log_tile_ver_TRACE_%d_BW_%d_METHOD_%d_INTER_%d_BUFF_%d_EST_%d.txt",vpParam.videoname.c_str(), vpParam.TRACE, max_bw, vpParam.METHOD, vpParam.INTERVAL, vpParam.BUFFSIZE, vpParam.VP_EST_METHOD);
  printf("#%s\n", fname);
  log_tile_ver = fopen(fname, "w");
  sprintf(fname, "result/video_%s/log_tile_ver_linear_TRACE_%d_BW_%d_METHOD_%d_INTER_%d_BUFF_%d_EST_%d.txt",vpParam.videoname.c_str(),vpParam.TRACE,max_bw, vpParam.METHOD, vpParam.INTERVAL, vpParam.BUFFSIZE, vpParam.VP_EST_METHOD);
  log_tile_ver_linear = fopen(fname, "w");
  printf("#%s\n", fname);
  sprintf(fname, "result/video_%s/log_dec_TRACE_%d_BW_%d_METHOD_%d_INTER_%d_BUFF_%d_EST_%d.txt",vpParam.videoname.c_str(),vpParam.TRACE,max_bw, vpParam.METHOD, vpParam.INTERVAL, vpParam.BUFFSIZE, vpParam.VP_EST_METHOD);
  log_dec = fopen(fname, "w");
  sprintf(fname, "result/video_%s/log_frame_TRACE_%d_BW_%d_METHOD_%d_INTER_%d_BUFF_%d_EST_%d.txt",vpParam.videoname.c_str(), vpParam.TRACE, max_bw, vpParam.METHOD, vpParam.INTERVAL, vpParam.BUFFSIZE, vpParam.VP_EST_METHOD);
  printf("#%s\n", fname);
  log_frame_psnr = fopen(fname, "w");
  if(log_tile_ver == NULL || log_tile_ver_linear == NULL || log_dec == NULL){
    printf("#[init] Cannot open log files\n");
    exit(1);
  }
  if(vpParam.METHOD > 0){
    fprintf(log_dec, "id\testThrp(kbps)\ttext_width\tbitrate(kbps)\n");
    fprintf(log_frame_psnr, "fid\tdecid\test_vp_psnr\tphi\ttheta\test_phi\test_theta\terr_phi\terr_theta\text_width\n");
    for(int ii=0; ii < dec_id; ii++){
      fprintf(log_tile_ver, "\nseg #%d calcTime: %d(ms)\n", ii, adaptLogic->calcTime[ii]);
      for(int i=0; i < rows; i++){
        for(int j=0; j < cols; j++){
          fprintf(log_tile_ver, "%d ",adaptLogic->tileVer[ii][i * rows + j]);
          fprintf(log_tile_ver_linear, "%d ",adaptLogic->tileVer[ii][i * rows + j]);
        }
        fprintf(log_tile_ver, "\n");
      }
      fprintf(log_tile_ver_linear, "\n");
      fprintf(log_dec, "%d\t%.2f\t%d\t%.2f\t%.2f\n", ii, adaptLogic->est_thrp[ii], adaptLogic->decide_width[ii], adaptLogic->seg_bitrate[ii], adaptLogic->calcTime[ii]/1000.0/1000.0);
      // calculate viewport psnr of frames in this interval
      // decide the adaptation result of this segment
      for(int k=0; k < vpParam.INTERVAL; k++){
        // printf("#frame #%d\n", ii * vpParam.INTERVAL + k);
        double vp_psnr = adaptLogic->estimateViewportPSNR(ii, adaptLogic->tileVer[ii], adaptLogic->frame_vp[ii * vpParam.INTERVAL + k]);
        fprintf(log_frame_psnr, "%d\t%d\t%.2f\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%d\t%.2f\n",ii * vpParam.INTERVAL + k, ii, vp_psnr, adaptLogic->frame_vp[ii*vpParam.INTERVAL +k][0], adaptLogic->frame_vp[ii*vpParam.INTERVAL +k][1], adaptLogic->est_vp_frame[ii*vpParam.INTERVAL + k][0], adaptLogic->est_vp_frame[ii*vpParam.INTERVAL + k][1], adaptLogic->estError[ii][0], adaptLogic->estError[ii][1], adaptLogic->decide_width[ii], adaptLogic->speed[ii][0]);
      }
      fflush(log_tile_ver);
      fflush(log_tile_ver_linear);
      fflush(log_dec);
      fflush(log_frame_psnr);
    }
  }else{//DASH
    fprintf(log_frame_psnr, "fid\tdecid\tver\tbitrate\test_vp_psnr\n");
    for(int ii=0; ii < dec_id; ii++){
      for(int k=0; k < vpParam.INTERVAL; k++){
        fprintf(log_frame_psnr, "%d\t%d\t%.2f\t%d\t%.2f\n", ii * vpParam.INTERVAL + k, ii,metadata->DASH_PSNR[ii][adaptLogic->selectDASHVer[ii]],adaptLogic->selectDASHVer[ii], metadata->DASH_BR[ii][adaptLogic->selectDASHVer[ii]]);
      }
    }
    fflush(log_frame_psnr);
    fclose(log_frame_psnr);
  }
}

