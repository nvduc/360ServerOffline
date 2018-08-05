#include "AdaptationLogic.h"
#include "Metadata.h"
#include "Viewport.h"
#include "GaussianFilter.h"
#include <math.h>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <vector>
#include "common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
// #include "ga-common.h"
// #include <stdlib.h>

#define TRUE 1
#define FALSE 0

int EST_ERR = 1;
int MOVE_LEFT;
// tile position structure
struct Pos {
  int i;
  int j;
};
// list of adjecent tiles of tile (i,j)
struct Pos TOP(int i, int j, int M, int N);
struct Pos DOWN(int i, int j, int M, int N);
struct Pos LEFT(int i, int j, int M, int N);
struct Pos RIGHT(int i, int j, int M, int N);
struct Pos TOP_LEFT(int i, int j, int M, int N);
struct Pos TOP_RIGHT(int i, int j, int M, int N);
struct Pos DOWN_LEFT(int i, int j, int M, int N);
struct Pos DOWN_RIGHT(int i, int j, int M, int N);



AdaptationLogic::AdaptationLogic(Metadata * meta, int INTERVAL_, int METHOD_, double margin_){
  int i;
  margin = margin_;
  METHOD = METHOD_;
  alpha_est_error = 0.2;
  metadata = meta;
  INTERVAL = INTERVAL_;
  //
  //
  estError = new double*[metadata->NO_SEG_FULL]; 
  speed = new double*[metadata->NO_SEG_FULL];
  for(i=0; i < metadata->NO_SEG_FULL; i++){
    estError[i] = new double[2];
    speed[i] = new double[2];
  }
  // initiate seg_play_time and frame_play_time
  seg_play_time = new int[metadata->NO_SEG_FULL];
  fr_play_time = new int[metadata->NO_SEG_FULL * INTERVAL];
  for(i=0; i < metadata->NO_SEG_FULL; i++){
    seg_play_time[i] = INTERVAL * metadata->SD * 1000;  
  }
  for(i=0; i < metadata->NO_SEG_FULL * INTERVAL; i++){
    fr_play_time[i] = i * metadata->SD * 1000;
  }
  // cout << "No_segment #2" << metadata->NO_SEG_FULL << endl;
  decide_time = new double[metadata->NO_SEG_FULL];
  fr_stall_time = new int[metadata->NO_SEG_FULL * INTERVAL];
  fr_rcv_time = new double[metadata->NO_SEG_FULL * INTERVAL];
  fr_inter_time = new double[metadata->NO_SEG_FULL * INTERVAL];
  fr_size = new int[metadata->NO_SEG_FULL * INTERVAL];
  frameviewport = new int[metadata->NO_SEG_FULL * INTERVAL];
  framepsnr = new double[metadata->NO_SEG_FULL * INTERVAL];
  //
  seg_end_time = new double[metadata->NO_SEG_FULL];
  seg_start_time = new double[metadata->NO_SEG_FULL];
  seg_thrp = new double[metadata->NO_SEG_FULL];
  dataThrp = new double[metadata->NO_SEG_FULL];
  est_thrp = new double[metadata->NO_SEG_FULL];
  cur_vp = new int*[metadata->NO_SEG_FULL];
  est_vp = new int*[metadata->NO_SEG_FULL];
  est_vp_last = new int[metadata->NO_SEG_FULL];
  seg_bitrate = new double[metadata->NO_SEG_FULL];
  real_vp_first = new int[metadata->NO_SEG_FULL];
  real_vp_last = new int[metadata->NO_SEG_FULL];
  RTT = new double[metadata->NO_SEG_FULL];
  est_vp_psnr_first = new double[metadata->NO_SEG_FULL];
  est_vp_psnr_last = new double[metadata->NO_SEG_FULL];
  est_error = new int[metadata->NO_SEG_FULL];
  tileSize = new double*[metadata->NO_SEG_FULL];
  tileVer = new int*[metadata->NO_SEG_FULL];
  for(i=0; i < metadata->NO_SEG_FULL; i++){
    tileSize[i] = new double[metadata->vp->No_tile];
    tileVer[i] = new int[metadata->vp->No_tile];
    cur_vp[i] = new int[2];
    est_vp[i] = new int[2];
  }
  erpSize = new int[metadata->NO_SEG_FULL];
  decide_width = new int[metadata->NO_SEG_FULL];
  //
  currEstError = 0;
  //
  est_thrp_act = new double[metadata->NO_SEG_FULL];
  //
  selectDASHVer = new int[metadata->NO_SEG_FULL];
  //
  for(i=0; i < metadata->NO_SEG_FULL; i++){
    decide_width[i] = 0;
  }
  select_method = new int[metadata->NO_SEG_FULL];
  est_vp_frame = new int*[metadata->NO_SEG_FULL * INTERVAL];
  frame_vp = new int*[metadata->NO_SEG_FULL * INTERVAL];
   for(i=0; i < metadata->NO_SEG_FULL * INTERVAL; i++){
    est_vp_frame[i] = new int[2];
    frame_vp[i] = new int[2];
  }

  calcTime = new int[metadata->NO_SEG_FULL];
  // load estimated head positions
  char buff[100];
  vector <double> v;
  int rows;
  int cols;
  int frame_id;
}
AdaptationLogic::~AdaptationLogic(){

}
int AdaptationLogic::getNextSegmentDASH(int index){
  int selectVer;
  estimator(index);
  selectVer = 0;

  //printf("est_thrp=%.2f\n", est_thrp[index]);
  //for(int i=0; i < metadata->NO_VER; i++)
    //printf("DASH_BR[%d][%d] = %.2f\n", index, i, metadata->DASH_BR[index][i]);

  while(selectVer < metadata->NO_VER && metadata->DASH_BR[index][selectVer] < est_thrp[index]) selectVer ++;

  selectVer = (selectVer > 0)?(selectVer-1):selectVer;

  selectDASHVer[index] = selectVer;

  return selectVer;
}
double* AdaptationLogic::getNextSegment(int index){
  struct timeval t_start, t_end, t_now;
  estimator(index);
  gettimeofday(&t_start, NULL);
  switch(METHOD){
  case 1:
    tileVer[index] = EXT_ALL(index, 0); // ROI
    break;
  case 2:
    tileVer[index] = EXT_ALL(index, 1); // EXT-1
    break;
  case 3:
    tileVer[index] = EXT_ALL(index, 2); // EXT-2
    break;
  case 4:
    tileVer[index] = EXT_ALL(index, 3); // EQUAL
    break;
  case 5:
    tileVer[index] = EXT_LEFT_RIGHT(index, 1); // LR-1
    break;
  case 6:
    tileVer[index] = EXT_LEFT_RIGHT(index, 2); // LR-2
    break;
  case 7:
    tileVer[index] = EXT_LEFT(index, 1); // L-1
    break;
  case 8:
    tileVer[index] = EXT_LEFT(index, 2); // L-2
    break;
  case 9:
    tileVer[index] = EXT_RIGHT(index, 1); // R-1
    break;
  case 10:
    tileVer[index] = EXT_RIGHT(index, 2); // R-2
    break;
  case 11:
    tileVer[index] = ISM(index); // ISM
    break;
  case 12:
    tileVer[index] = BellLab(index); // BellLab
    break;
  case 13:
    tileVer[index] = Proposed(index); // Adapt-area
    break;
  case 14:
    tileVer[index] = ROI_first(index); // ROI using first frame as the estimated value
    break;
  case 15:
    tileVer[index] = Proposed_v2(index); // Adaptive: High-Low
    break;
  case 16:
    tileVer[index] = Proposed_v3(index); // Adaptive: Low-High
    break;
  case 17:
    tileVer[index] = Proposed_v4(index); // Adaptive: Low-High-Low
    break;
  case 18:
    tileVer[index] = ISM_ext(index); // Full-search ISM, ext-width=2, 9 areas.
    break;
  case 19:
    tileVer[index] = proposed_3(index); // last ISM's implementation
    break;
  case 20:
    tileVer[index] = ISM_error(index);
    break;
  case 21:
    tileVer[index] = ISM_ext_v3(index); // Full-search ISM, ext-width=2, 9 areas: quick-1
    break;
  case 22:
    tileVer[index] = ISM_ext_v4(index); // Full-search ISM, ext-width=2, 9 areas: quick-2
    break;
    
  }
  gettimeofday(&t_end, NULL);
  // calculate tiles' size and segments' bitrate
  tileSize[index] = getTileSize(index, tileVer[index]);
  seg_bitrate[index] = getSegBitrate(index, tileVer[index]);
  calcTime[index] = tvdiff_us(&t_end, &t_start);
  // record decision making time
  //gettimeofday(&decTime, NULL);
  //printf("%ld.%06ld\n", decTime.tv_sec, decTime.tv_usec);
  return tileSize[index];
}

int* AdaptationLogic::BellLab(int index){
  int i,j, q;
  int rows = metadata->vp->No_tile_v;
  int cols = metadata->vp->No_tile_h;
  int tiles = rows * cols;
  double **vprob;
  //
  double used_bandwidth = 0;
  int *tileVer = new int[tiles];
  for(i=0; i < tiles; i++){ // allocate a minimum quality for all tiles
    tileVer[i] = 0;
    used_bandwidth += metadata->TILE_BR[index][i][tileVer[i]];
  }
  // first interval
  if(index==0)
    return tileVer;
  // generate gaussian filter kernel
  double sigma = (abs(speed[index][0]) / 0.033) / 180.0 * 3.14  * metadata->delay;
  if(sigma > 0){
    int filter_size = 2 * ceil(sigma * 2) + 1;
    double** kernel = filter(filter_size, sigma);
    printf("############ index:%d\n", index);
    printf("############ KERNEL:\n");
    for(i=0; i < filter_size; i++){
      for(j=0; j < filter_size; j++)
	printf("%.4f ", kernel[i][j]);
      printf("\n");
    }
    // compute tile visible probabilities
    int *vmask = getVisibleTile(cur_vp[index]);
    double** vmask_2D = new double*[rows];
    for(i=0; i < rows; i++){
      vmask_2D[i] = new double[cols];
      for(j=0; j < cols; j++)
	vmask_2D[i][j] = vmask[i * rows + j];
    }
    printf("############ VMASK:\n");
    for(i=0; i < rows; i++){
      for(j=0; j < cols; j++)
	printf("%.2f ", vmask_2D[i][j]);
      printf("\n");
    }
    vprob = applyFilter(vmask_2D, kernel, rows, filter_size); // visible probabilities of tiles
    printf("############ vprob:\n");
    for(i=0; i < rows; i++){
      for(j=0; j < cols; j++)
	printf("%.4f ", vprob[i][j]);
      printf("\n");
    }
  }else{
    int *vmask = getVisibleTile(cur_vp[index]);
    double** vmask_2D = new double*[rows];
    for(i=0; i < rows; i++){
      vmask_2D[i] = new double[cols];
      for(j=0; j < cols; j++)
	vmask_2D[i][j] = vmask[i * rows + j];
    }
    vprob = vmask_2D;
  }
  //	//
  double** A = iniTwoDimenArrayDouble(4, metadata->vp->No_tile * (metadata->NO_VER - 1));
  for(i=0; i < metadata->vp->No_tile; i++){
    printf("#[Tile #%d]: ", i);
    for(q=1; q < metadata->NO_VER; q++){
      A[0][i * (metadata->NO_VER - 1) + q - 1] = i;
      A[1][i * (metadata->NO_VER - 1) + q - 1] = q;
      A[2][i * (metadata->NO_VER - 1) + q - 1] = metadata->Uti[index][i][q] * vprob[i/rows][i%rows]/metadata->Cost[index][i][q];
      A[3][i * (metadata->NO_VER - 1) + q - 1] = metadata->Cost[index][i][q];
      printf("%.2f\t%.2f\t", A[2][i * (metadata->NO_VER - 1) + q - 1], A[3][i * (metadata->NO_VER - 1) + q - 1]);
    }
    printf("\n");
  }
  // sort A[2] in ascending order
  std::vector<double> array(A[2], A[2] + metadata->vp->No_tile * (metadata->NO_VER - 1));
  std::vector<int> indices = sort_index(array);
  //
  printf("sorted array:\n");//
  //    for (std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it){
  //       std::cout << *it << " " << array[*it] << "\n";
  //    }
  //
  //calculate the selected version for each tile
  for (std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it){
    if(used_bandwidth + A[3][*it] <= est_thrp[index] && tileVer[(int)A[0][*it]] == A[1][*it] - 1){
      tileVer[(int)A[0][*it]] = A[1][*it];
      used_bandwidth += A[3][*it];
    }
  }
  printf("############ TILE VERSION:\n");
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++)
      printf("%d ", tileVer[i * rows + j]);
    printf("\n");
  }
  printf("############ TILE VERSION:\n");
  return tileVer;
}
void AdaptationLogic::estimator(int index){

  double alpha = 0.8;
  int i;
  double tmp;
  int last_frame_id;
  int VP_EST_WIN = INTERVAL; /* Viewport estimation window in frames*/
  if(EST_ERR == 1){
  if(index <= metadata->BUFF/INTERVAL || last_frame_id < 2){ /* all estimated values are set to 0 for the first interval */
    cur_vp[index][0] = 0;
    cur_vp[index][1] = 0;
    // estimate viewport as the last updated viewport.
    est_vp[index][0] = 0;
    est_vp[index][1] = 0;
  for(i=0; i < INTERVAL; i++){
    est_vp_frame[index * INTERVAL + i][0] = 0;
    est_vp_frame[index * INTERVAL + i][1] = 0;
  }
  estError[index][0] = 0;
  estError[index][1] = 0;
  speed[index][0] = 0;
  speed[index][1] = 0;
  est_thrp_act[index] = 0;
  MOVE_LEFT = TRUE;
  }else{
    // record last frame id
    // lastFeedbackFrame[index] = last_frame_id;
    //printf("#[estimator] lastFeedbackFrame: %d\n", lastFeedbackFrame[index]);
    // get last reported feedbacks
    last_frame_id = index*INTERVAL - 3;
    cur_vp[index][0] = frame_vp[last_frame_id][0];
    cur_vp[index][1] = frame_vp[last_frame_id][1];
    /* calculate estimation errors */
    // use average of all received frames
    // estError[index][0] = 0;
    // for(i=0; i < last_frame_id; i++){
    //   tmp = -est_vp_frame[last_frame_id][0] + frame_vp[last_frame_id][0];
    //   if(tmp < -180)
    //     tmp += 360;
    //   else if(tmp > 180)
    //     tmp -= 360;
    //   estError[index][0] += tmp / last_frame_id;
    // }
    // use error of the first frame
    // estError[index][0] = - est_vp[index-1][0] + frame_vp[(index-1) * INTERVAL][0];
    // using error of the last received frame
    estError[index][0] = -est_vp_frame[last_frame_id][0] + frame_vp[last_frame_id][0];
    if(estError[index][0] < -180)
      estError[index][0] += 360;
    else if(estError[index][0] > 180)
      estError[index][0] -= 360;
    // theta
     // use average of all received frames
    // estError[index][1] = 0;
    // for(i=0; i < last_frame_id; i++){
    //   tmp = - est_vp_frame[last_frame_id][1] + frame_vp[last_frame_id][1];
    //   if(tmp < -90)
    //     tmp += 180;
    //   else if(tmp > 90)
    //     tmp -= 180;
    //   estError[index][1] += tmp / last_frame_id;
    // }
    // use error of the first frame
    // estError[index][1] = -est_vp[index-1][1] + frame_vp[(index-1) * INTERVAL][1];
    estError[index][1] = - est_vp_frame[last_frame_id][1] + frame_vp[last_frame_id][1];
    if(estError[index][1] < -90)
      estError[index][1] += 180;
    else if(estError[index][1] > 90)
      estError[index][1] -= 180;

    // estError[index][0] = 0;
    // estError[index][1] = 0;
    /* speed according to phi */
    //printf("## last_frame_id:%d (%d,%d)\n",last_frame_id,  frame_vp[last_frame_id][0], frame_vp[last_frame_id][1]);
    int delta_phi;
    //printf("##frame_vp[%d][0]=%d\n",last_frame_id, frame_vp[last_frame_id][0]);
    if(last_frame_id >= VP_EST_WIN){
     // printf("##frame-vp[%d][0] = %d\n", last_frame_id - VP_EST_WIN, frame_vp[last_frame_id-VP_EST_WIN][0]);
      delta_phi = frame_vp[last_frame_id][0] - frame_vp[last_frame_id-VP_EST_WIN][0];
    }
    else{
      //printf("##frame-vp[%d][0] = %d\n", 0, frame_vp[0][0]);
      delta_phi = frame_vp[last_frame_id][0] - frame_vp[0][0];
    }
    //printf("##phi=%d\n", delta_phi);
    if(delta_phi < -180)
      delta_phi += 360;
    else
      if(delta_phi > 180)
  delta_phi -= 360;
    if(last_frame_id == 0)
      speed[index][0] = 0;
    else
      speed[index][0] = delta_phi/(1.0 * ((last_frame_id >= VP_EST_WIN)?VP_EST_WIN:last_frame_id));
    // speed according to theta
    int delta_theta;
    if(last_frame_id >= VP_EST_WIN)
      delta_theta = frame_vp[last_frame_id][1] - frame_vp[last_frame_id-VP_EST_WIN][1];
    else
      delta_theta = frame_vp[last_frame_id][1] - frame_vp[0][1];
    if(delta_theta < -90)
      delta_theta += 180;
    else
      if(delta_theta > 90)
  delta_theta -= 180;
    if(last_frame_id == 0)
      speed[index][1] = 0;
    else
      speed[index][1] = delta_theta/(1.0 * ((last_frame_id >= VP_EST_WIN)?VP_EST_WIN:last_frame_id));

    // estimate viewport as the last updated viewport.
    //printf("#[Estimator]: index=%d (%d, %d) (%.2f, %.2f) (%d, %d)\n",index, cur_vp[index][0], cur_vp[index][1], speed[index][0], speed[index][1], frame_vp[last_frame_id][1]  ,frame_vp[last_frame_id - VP_EST_WIN][1]);    
      for(i=0; i < INTERVAL; i++){
        est_vp_frame[index * INTERVAL + i][0] = (int) (cur_vp[index][0] + speed[index][0] * (metadata->BUFF + i));
        est_vp_frame[index * INTERVAL + i][1] = (int) (cur_vp[index][1] + speed[index][1] * (metadata->BUFF + i));
         //
         while(est_vp_frame[index * INTERVAL + i][0] >= 180)
        est_vp_frame[index * INTERVAL + i][0] -= 360;
         while(est_vp_frame[index * INTERVAL + i][0] < -180)
        est_vp_frame[index * INTERVAL + i][0] += 360;
         //
   //  if(est_vp_frame[index * INTERVAL + i][1] > 180)
   //est_vp_frame[index * INTERVAL + i][1] = est_vp_frame[index * INTERVAL + i][1]%180;
         while(est_vp_frame[index * INTERVAL + i][1] >= 90)
           est_vp_frame[index * INTERVAL + i][1] -= 90;
        while(est_vp_frame[index * INTERVAL + i][1] <= -90)//
            est_vp_frame[index * INTERVAL + i][1] += 90;
         //printf("##frame[%d]=(%d, %d)\n", i, est_vp_frame[index * INTERVAL + i][0], est_vp_frame[index * INTERVAL + i][1]);
        // calculate estimator errors

      }
    // use viewport of the middle frame as the representative viewport of the interval
      /*
    est_vp[index][0] = est_vp_frame[index * INTERVAL + INTERVAL/2][0];
    est_vp[index][1] = est_vp_frame[index * INTERVAL + INTERVAL/2][1];
      */
    est_vp[index] = est_vp_frame[index * INTERVAL];
    // use last reported viewport position as the representative viewport of the interval
    est_thrp_act[index] = (1-alpha) * est_thrp_act[index -1] + alpha * seg_thrp[index]; // smoothing
    //
    if(cur_vp[index][0] - cur_vp[index-1][0] > 0)
      MOVE_LEFT = FALSE;
    else
      MOVE_LEFT = TRUE;
  }
  }else{
    estError[index][0] = 0;
    estError[index][1] = 0;
    est_vp[index] = est_vp_frame[index * INTERVAL];
    est_thrp_act[index] = (1-alpha) * est_thrp_act[index -1] + alpha * seg_thrp[index]; // smoothing
  }
  est_thrp[index] = (1-margin) * est_thrp_act[index];

}
int AdaptationLogic::calcViewport(int** vp_trace, double t_now) {
  int cur_vp = 0;
  int i=0;
  //		printf("[DUC_CODE_LOG][calcViewport] t_now=%.2f NO_HEAD_SAMPLE = %d\n", t_now, metadata->NO_HEAD_SAMPLE);
  while(i < metadata->NO_HEAD_SAMPLE && vp_trace[i][0] <= t_now) i++;
  if(i==metadata->NO_HEAD_SAMPLE)
    cur_vp = vp_trace[metadata->NO_HEAD_SAMPLE-1][1];
  else{
    // intepolate from two nearest positions [i-1] and [i]
    //			if(vp_trace[i][1] == 0 && vp_trace[i-1][1] > 180){
    //				cur_vp = (int) (vp_trace[i-1][1] + (360*1.0 - vp_trace[i-1][1])/(vp_trace[i][0] - vp_trace[i-1][0])*(t_now - vp_trace[i-1][0]));
    //			}else{
    //				cur_vp = (int) (vp_trace[i-1][1] + (vp_trace[i][1]*1.0 - vp_trace[i-1][1])/(vp_trace[i][0] - vp_trace[i-1][0])*(t_now - vp_trace[i-1][0]));
    //			}
    if(vp_trace[i-1][1] == 360){
      cur_vp = (int) (vp_trace[i][1] * 1.0 /(vp_trace[i][0] - vp_trace[i-1][0])*(t_now - vp_trace[i-1][0]));
    }else{
      cur_vp = (int) (vp_trace[i-1][1] + (vp_trace[i][1]*1.0 - vp_trace[i-1][1])/(vp_trace[i][0] - vp_trace[i-1][0])*(t_now - vp_trace[i-1][0]));
    }
  }
  return cur_vp;
}
double* AdaptationLogic::getTileSize(int index, int *tileVer){
  //	printf("[DUC_CODE_LOG][getTileSize] 1 \n");
  double* tileSize = new double[metadata->vp->No_tile];
  for(int i=0; i < metadata->vp->No_tile; i++){
    tileSize[i] = metadata->TILE_SIZE[index][i][tileVer[i]];
  }
  return tileSize;
}
double AdaptationLogic::getSegBitrate(int index, int *tileVer){
  double seg_bitrate = 0;
  for(int i=0; i < metadata->vp->No_tile; i++)
    seg_bitrate += metadata->TILE_BR[index][i][tileVer[i]];
  //printf("#[DUC]: TILE_BR[%d][0][%d] = %.2f seg_bitrate=%.2f\n",index,tileVer[0],metadata->TILE_BR[index][0][tileVer[0]], seg_bitrate);
  return seg_bitrate;	
}
void AdaptationLogic::updateFramePlayTime(int d, int index){
  for(int i=index; i < metadata->NO_SEG_FULL * INTERVAL; i++)
    fr_play_time[i] += d;
}
double AdaptationLogic::estimateViewportPSNR(int index, int* tileVer, int* est_vp) {
  int i,j;
  int rows = 8;
  int cols = 8;
  int tiles = 64;
  // determine visible tiles and corresponding tiles' portion in viewport
  int* vmask = getVisibleTile(est_vp);
  if(vmask == NULL){
    printf("vmask null\n");
    printf("vp: (%d, %d)\n", est_vp[0], est_vp[1]);
  }
  int* pixel = getVisibleTilePixel(est_vp);
  if(pixel == NULL){
    printf("(phi, theta) = (%d, %d)\n", est_vp[0], est_vp[1]);
    printf("pixel null\n");
  }
  // estimate viewport MSE as a weighted sum of tiles' MSE. Tiles' weight = % of tile in viewport
  double est_vp_mse = 0;
  for(int i=0; i < metadata->vp->No_tile; i++){
    // printf("%d %d %d %d\n", index, i, tileVer[i], pixel[i]);
    if(vmask[i] > 0){
      est_vp_mse += metadata->TILE_MSE[index][i][tileVer[i]] * pixel[i] * 1.0 / (metadata->vp->vp_W * metadata->vp->vp_H);
    }
  }
  double est_vp_psnr_first = 10 * log10((255*255)/est_vp_mse);
  // printf("%.2f\t%.2f\n", est_vp_mse, est_vp_psnr_first);
  return est_vp_psnr_first;
}
int* AdaptationLogic::getVisibleTile(int* cur_vp){
  int phi,theta;
  if(cur_vp[0] < 0)
    phi = cur_vp[0] + 360;
  else
    phi = cur_vp[0];
  // convert from [-90; 90] to [0; 180]
  theta = cur_vp[1] + 90;
  //  printf("#[getVisibleTile]: (%d, %d) -> (%d, %d)\n", cur_vp[0], cur_vp[1], phi, theta);
  return metadata->vmask[phi][theta];
}
int* AdaptationLogic::getVisibleTilePixel(int* cur_vp){
 int phi,theta;
  if(cur_vp[0] < 0)
    phi = cur_vp[0] + 360;
  else
    phi = cur_vp[0];
  // convert from [-90; 90] to [0; 180]
  theta = cur_vp[1] + 90;
  //  printf("#[getVisibleTile]: (%d, %d) -> (%d, %d)\n", cur_vp[0], cur_vp[1], phi, theta);
  return metadata->pixel[phi][theta];
}
int** AdaptationLogic::iniTwoDimenArrayInt(int rows, int colms){
  int** ret = new int*[rows];
  for(int i=0; i < rows; i++){
    ret[i] = new int[colms];
    for(int j=0; j < colms; j++)
      ret[i][j] = 0;
  }
  return ret;
}
double** AdaptationLogic::iniTwoDimenArrayDouble(int rows, int colms){
  double** ret = new double*[rows];
  for(int i=0; i < rows; i++){
    ret[i] = new double[colms];
  }
  return ret;
}
struct Pos TOP(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = -1;
  pos.j = -1;
  if(i==0) return pos;
  pos.i = i-1;  
  pos.j = j;
  //printf("#[TOP] (pos.i, pos.j) = (%d, %d)\n", pos.i, pos.j);
  return pos;
}
struct Pos DOWN(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = -1;
  pos.j = -1;
  if(i==M-1) return pos;
  pos.j = j;
  pos.i = i+1;
  return pos;
}
struct Pos LEFT(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = i;
  if(j==0)
    pos.j = N-1;
  else
    pos.j = j-1;
  return pos;
}
struct Pos RIGHT(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = i;
  if(j==N-1)
    pos.j = 0;
  else
    pos.j = j+1;
  return pos;
}
struct Pos TOP_LEFT(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = -1;
  pos.j = -1;
  if(i==0)
    return pos;
  pos.i = i-1;
  pos.j = (j==0)?(N-1):(j-1);
  return pos;
}
struct Pos TOP_RIGHT(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = -1;
  pos.j = -1;
  if(i==0)
    return pos;
  pos.i = i-1;
  pos.j = (j==N-1)?0:(j+1);
  return pos;
}
struct Pos DOWN_LEFT(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = -1;
  pos.j = -1;
  if(i==M-1)
    return pos;
  pos.i = i+1;
  pos.j = (j==0)?(N-1):(j-1);
  return pos;
}
struct Pos DOWN_RIGHT(int i, int j, int M, int N){
  struct Pos pos;
  pos.i = -1;
  pos.j = -1;
  if(i==M-1)
    return pos;
  pos.i = i+1;
  pos.j = (j==N-1)?0:(j+1);
  return pos;
}
int* AdaptationLogic::extVMask(int* vmask, int ext_width){
  int* ret = new int[metadata->vp->No_tile];
  int M = metadata->vp->No_tile_v;
  int N = metadata->vp->No_tile_h;
  int i,j;
  struct Pos pos;
  if(ext_width == 0) return vmask;
  if(ext_width >= 3){
    for(i=0; i < metadata->vp->No_tile; i++)
      ret[i] = 1;
    return ret;
  }
  // vmask after extension
  int** vmask_ext = iniTwoDimenArrayInt(metadata->vp->No_tile_v, metadata->vp->No_tile_h);
  // convert 'vmask' to rectangle form
  //printf("#[ExtendVisibleMask_v2]: vmask\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      vmask_ext[i][j] = vmask[i*M + j];
      //printf("%d ", vmask[i*M + j]);
    }
    //printf("\n");
  }
  //
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//TOP
	pos = TOP(i,j,M,N);
	//printf("#[ExtendVisibleMask_v2]: TOP(i,j)=(%d,%d)\n", pos.i,pos.j);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//DOWN
	pos = DOWN(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//TOP-LEFT
	pos = TOP_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//TOP-RIGHT
	pos = TOP_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//DOWN-LEFT
	pos = DOWN_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//DOWN-RIGHT
	pos = DOWN_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  if(ext_width == 1) return ret;
  // extension width = 2
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//TOP
	pos = TOP(i,j,M,N);
	//printf("#[ExtendVisibleMask_v2]: TOP(i,j)=(%d,%d)\n", pos.i,pos.j);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//DOWN
	pos = DOWN(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//TOP-LEFT
	pos = TOP_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//TOP-RIGHT
	pos = TOP_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//DOWN-LEFT
	pos = DOWN_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//DOWN-RIGHT
	pos = DOWN_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext_2\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  return ret;
  
}
int* AdaptationLogic::extVMask_LR(int* vmask, int ext_width){
  int* ret = new int[metadata->vp->No_tile];
  int M = metadata->vp->No_tile_v;
  int N = metadata->vp->No_tile_h;
  int i,j;
  struct Pos pos;
  if(ext_width == 0) return vmask;
  if(ext_width >= 3){
    for(i=0; i < metadata->vp->No_tile; i++)
      ret[i] = 1;
    return ret;
  }
  // vmask after extension
  int** vmask_ext = iniTwoDimenArrayInt(metadata->vp->No_tile_v, metadata->vp->No_tile_h);
  // convert 'vmask' to rectangle form
  //printf("#[ExtendVisibleMask_v2]: vmask\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      vmask_ext[i][j] = vmask[i*M + j];
      //printf("%d ", vmask[i*M + j]);
    }
    //printf("\n");
  }
  //
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  if(ext_width == 1) return ret;
  // extension width = 2
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext_2\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  return ret;
  
}
int* AdaptationLogic::extVMask_L(int* vmask, int ext_width){
  int* ret = new int[metadata->vp->No_tile];
  int M = metadata->vp->No_tile_v;
  int N = metadata->vp->No_tile_h;
  int i,j;
  struct Pos pos;
  if(ext_width == 0) return vmask;
  if(ext_width >= 3){
    for(i=0; i < metadata->vp->No_tile; i++)
      ret[i] = 1;
    return ret;
  }
  // vmask after extension
  int** vmask_ext = iniTwoDimenArrayInt(metadata->vp->No_tile_v, metadata->vp->No_tile_h);
  // convert 'vmask' to rectangle form
  //printf("#[ExtendVisibleMask_v2]: vmask\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      vmask_ext[i][j] = vmask[i*M + j];
      //printf("%d ", vmask[i*M + j]);
    }
    //printf("\n");
  }
  //
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  if(ext_width == 1) return ret;
  // extension width = 2
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext_2\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  return ret;
  
}
int* AdaptationLogic::extVMask_R(int* vmask, int ext_width){
  int* ret = new int[metadata->vp->No_tile];
  int M = metadata->vp->No_tile_v;
  int N = metadata->vp->No_tile_h;
  int i,j;
  struct Pos pos;
  if(ext_width == 0) return vmask;
  if(ext_width >= 3){
    for(i=0; i < metadata->vp->No_tile; i++)
      ret[i] = 1;
    return ret;
  }
  // vmask after extension
  int** vmask_ext = iniTwoDimenArrayInt(metadata->vp->No_tile_v, metadata->vp->No_tile_h);
  // convert 'vmask' to rectangle form
  printf("#[ExtendVisibleMask_v2]: vmask\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      vmask_ext[i][j] = vmask[i*M + j];
      printf("%d ", vmask[i*M + j]);
    }
    printf("\n");
  }
  //
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  printf("#[ExtendVisibleMask_v2]: vmask_ext\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      printf("%d ", vmask_ext[i][j]);
    }
    printf("\n"); 
  }
  if(ext_width == 1) return ret;
  // extension width = 2
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
  // 2->1
  printf("#[ExtendVisibleMask_v2]: vmask_ext_2\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 2)
	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      printf("%d ", vmask_ext[i][j]);
    }
    printf("\n"); 
  }
  return ret;
  
}
int* AdaptationLogic::ISM(int index, int ext_width){
  int i, j, ii, jj;
  int rows = metadata->vp->No_tile_v;
  int cols = metadata->vp->No_tile_h;
  int tiles = rows * cols;
  int *tileVer = NULL;
  int *vmask_ext = new int[tiles];
  int mean_ver = metadata->NO_VER;
  int mean_ver_2 = metadata->NO_VER;
  int vp_ver;
  int ext_1_ver;
  double BW = est_thrp[index];
  double remainBW;
  bool FLAG;
  int *selectTileVer = new int[tiles];
  double max_vp_psnr = 0;
  double vp_psnr;
  double vp_psnr_last;
  //
  int width = 0;
  //  est_vp[index] = est_vp_frame[INTERVAL * index];
  // get visible mask with estimated viewport
  printf("#[seg #%d]:\n", index);
  int *vmask = getVisibleTile(est_vp[index]);
  printf("#[ISM] seg #%d vmask:\n", index);
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      printf("%d ", vmask[i * rows + j]);
    }
    printf("\n");
  }
  printf("\n");
  for(j=1; j <= ext_width; j++){
    // extend the visible area by 'ext_width' tile => in all directions
    int* vmask_ext_tmp = extVMask(vmask, j);
    for(i=0; i < tiles; i++){
      if(j==1){
	if(vmask_ext_tmp[i] == 1){
	  if(vmask[i] == 0)
	    vmask_ext[i] += j+1;
	  else
	    vmask_ext[i] = 1;
	}else
	  vmask_ext[i] = 0;
      }else{
	if(vmask_ext_tmp[i] == 1){
	  if(vmask_ext[i] == 0)
	    vmask_ext[i] += j+1;
	}
      }
    }
  }
  vmask = vmask_ext;
  printf("#########vmask_adjusted:\n");
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      printf("%d ", vmask[i * rows + j]);
    }
    printf("\n");
  }
  printf("\n");
  // get the selected version when all tile are of equal quality
  tileVer = decideTileVer_ROI(index, vmask, est_thrp[index]);
  // calculate viewport-psnr given 'tileVer'
  int tmp_vp[2];
  // first frame & last frame
  vp_psnr = estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL]);
  vp_psnr_last = estimateViewportPSNR(index, tileVer, est_vp_frame[(index + 1) * INTERVAL - 1]);
  // update selection
  if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
    std::copy (tileVer, tileVer + tiles, selectTileVer);
    max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
  }
  printf("#########tileVer:\n");
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      printf("%d ", tileVer[i * rows + j]);
    }
    printf("\n");
  }
  printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
  //p
  if(ext_width == 1){
    // determine quality of visible part
    for(i=0; i < tiles; i++)
      if(vmask[i] > 0 && tileVer[i] < mean_ver)
	mean_ver = tileVer[i];
    // search for the best (viewport-version, extesion-version) pair
    vp_ver = mean_ver + 1;
    while(vp_ver < metadata->NO_VER){
      remainBW = BW;
      // assign quality for visible and non-visible tiles
      for(i=0; i < tiles; i++){
	if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
	else
	  tileVer[i] = 0; // minimum quality for non-visible tiles
	remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
      }
      //
      if(remainBW <= 0){
	// reduce vp tiles's version to meet the bandwidth constraint
	for(i=0; i < tiles; i++){
	  if(vmask[i] == 1){
	    tileVer[i] -= 1; // assume that all tiles have same priority
	    remainBW += metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
	    if(remainBW > 0)
	      break;
	  }
	}
	// calculate viewport-psnr given 'tileVer'
	int tmp_vp[2];
	// first frame & last frame
	vp_psnr = estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL]);
	vp_psnr_last = estimateViewportPSNR(index, tileVer, est_vp_frame[(index + 1) * INTERVAL - 1]);
	// update selection
	if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	  std::copy (tileVer, tileVer + tiles, selectTileVer);
	  max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
	}
	break; // not enough bandwidth
      }
      // assign remaining bandwidth for ext_1 tiles
      FLAG = false;
      while(remainBW > 0 && !FLAG){
	FLAG = true;
	for(i=0; i < metadata->vp->No_tile; i++){
	  if(vmask[i] == 2 && remainBW > 0){
	    if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
	      remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
	      tileVer[i] ++;
	      FLAG = false;
	    }
	  }
	}
      }
      // assign remaining bandwidth to non-visible tiles
      if(remainBW > 0){
	FLAG = false;
	while(remainBW > 0 && !FLAG){
	  FLAG = true;
	  for(i=0; i < metadata->vp->No_tile; i++){
	    if(vmask[i] == 0 && remainBW > 0){
	      if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		tileVer[i] ++;
		FLAG = false;
	      }
	    }
	  }
	}
      }
      // calculate viewport-psnr given 'tileVer'
      // first frame & last frame
      vp_psnr = estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL]);
      vp_psnr_last = estimateViewportPSNR(index, tileVer, est_vp_frame[(index + 1) * INTERVAL - 1]);
      // update selection
      if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	std::copy (tileVer, tileVer + tiles, selectTileVer);
	max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
      }
      //
      printf("#########tileVer: vp_ver=%d\n", vp_ver);
      for(i=0; i < rows; i++){
	for(j=0; j < cols; j++){
	  printf("%d ", tileVer[i * rows + j]);
	}
	printf("\n");
      }
      printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
      //
      vp_ver ++;
    }//endwhile
  }
  else{
    // determine quality of visible part
    for(i=0; i < tiles; i++)
      if(vmask[i] > 0 && tileVer[i] < mean_ver)
	mean_ver = tileVer[i];
    // search for the best (viewport-version, ext1-version, ext2-version) tube
    vp_ver = mean_ver + 1;
    while(vp_ver < metadata->NO_VER){
      remainBW = BW;
      // assign quality for visible and non-visible tiles
      for(i=0; i < tiles; i++){
	if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
	else
	  tileVer[i] = 0; // minimum quality for non-visible tiles
	remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
      }
      //
      if(remainBW <= 0) break;
      // determine quality if 'ext1' and 'ext2' are assigned same version
      // equally assign remaining bandwidth for 'ext_1' and 'ext_2'
      FLAG = false;
      while(remainBW > 0 && !FLAG){
	FLAG = true;
	for(i=0; i < metadata->vp->No_tile; i++){
	  if((vmask[i] == 2 || vmask[i] == 3) && remainBW > 0){
	    if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
	      remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
	      tileVer[i] ++;
	      FLAG = false;
	    }
	  }
	}
      }
      // find version
      for(i=0; i < tiles; i++)
	if((vmask[i] == 2 || vmask[i] == 3) && tileVer[i] < mean_ver_2)
	  mean_ver_2 = tileVer[i];
      // assign remaining bandwidth to non-visible tiles
      if(remainBW > 0){
	FLAG = false;
	while(remainBW > 0 && !FLAG){
	  FLAG = true;
	  for(i=0; i < metadata->vp->No_tile; i++){
	    if(vmask[i] == 0 && remainBW > 0){
	      if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		tileVer[i] ++;
		FLAG = false;
	      }
	    }
	  }
	}
      }
      //
      // calculate viewport-psnr given 'tileVer'
      // first frame & last frame
      vp_psnr = estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL]);
      vp_psnr_last = estimateViewportPSNR(index, tileVer, est_vp_frame[(index + 1) * INTERVAL - 1]);			// update selection
      if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	std::copy (tileVer, tileVer + tiles, selectTileVer);
	max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
      }
      printf("#########tileVer: vp_ver:%d\n", vp_ver);
      for(i=0; i < rows; i++){
	for(j=0; j < cols; j++){
	  printf("%d ", tileVer[i * rows + j]);
	}
	printf("\n");
      }
      printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
      //
      ext_1_ver = mean_ver_2 + 1;
      while(ext_1_ver <= vp_ver){
	remainBW = BW;
	// assign quality for visible and non-visible tiles
	for(i=0; i < tiles; i++){
	  if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
	  else
	    if(vmask[i] == 2) tileVer[i] = ext_1_ver; // assign quality for ext_1 tiles
	    else
	      tileVer[i] = 0;
	  remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
	}
	//
	if(remainBW <= 0) break;
	// assign remaining bandwidth for 'ext_2' tiles
	FLAG = false;
	while(remainBW > 0 && !FLAG){
	  FLAG = true;
	  for(i=0; i < metadata->vp->No_tile; i++){
	    if(vmask[i] == 3 && remainBW > 0){
	      if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		tileVer[i] ++;
		FLAG = false;
	      }
	    }
	  }
	}
	// assign remaining bandwidth to non-visible tiles
	if(remainBW > 0){
	  FLAG = false;
	  while(remainBW > 0 && !FLAG){
	    FLAG = true;
	    for(i=0; i < metadata->vp->No_tile; i++){
	      if(vmask[i] == 0 && remainBW > 0){
		if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		  remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		  tileVer[i] ++;
		  FLAG = false;
		}
	      }
	    }
	  }
	}
	// calculate viewport-psnr given 'tileVer'
	// first frame & last frame
	vp_psnr = estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL]);
	vp_psnr_last = estimateViewportPSNR(index, tileVer, est_vp_frame[(index + 1) * INTERVAL - 1]);
	// update selection
	if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	  std::copy (tileVer, tileVer + tiles, selectTileVer);
	  max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
	}
	//
	printf("#########tileVer: vp_ver:%d ext_1_ver:%d\n", vp_ver, ext_1_ver);
	for(i=0; i < rows; i++){
	  for(j=0; j < cols; j++){
	    printf("%d ", tileVer[i * rows + j]);
	  }
	  printf("\n");
	}
	printf("\nest_vp:%d\testError:%.2f\tvp_psnr:%.2f\t max_vp_psnr:%.2f\n",est_vp[index][0], estError[index][0], vp_psnr, max_vp_psnr);
	ext_1_ver ++;
      }
      vp_ver++;
    }
  }
  /*
  printf("#########selecteTileVer:\n");
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      printf("%d ", selectTileVer[i * rows + j]);
    }
    printf("\n");
  }
  */
  //
  return selectTileVer;
}
int* AdaptationLogic::ISM_error(int index, int ext_width){
  int i, j, ii, jj;
  int rows = metadata->vp->No_tile_v;
  int cols = metadata->vp->No_tile_h;
  int tiles = rows * cols;
  int *tileVer = NULL;
  int *vmask_ext = new int[tiles];
  int mean_ver = metadata->NO_VER;
  int mean_ver_2 = metadata->NO_VER;
  int vp_ver;
  int ext_1_ver;
  double BW = est_thrp[index];
  double remainBW;
  bool FLAG;
  int *selectTileVer = new int[tiles];
  double max_vp_psnr = 0;
  double vp_psnr;
  double vp_psnr_last;
  //
  int width = 0;
  //  est_vp[index] = est_vp_frame[INTERVAL * index];
  // get visible mask with estimated viewport
  printf("#[seg #%d]:\n", index);
  int *vmask = getVisibleTile(est_vp[index]);
  printf("#[ISM] seg #%d vmask:\n", index);
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      vmask_ext[i * rows + j] = 0;
      printf("%d ", vmask[i * rows + j]);
    }
    printf("\n");
  }
  printf("\n");
  if(index == 3){
    printf("#[ISM_error] start debugging\n");
    printf("#[ISM_error] estError=(%.2f, %.2f)\n", estError[index][0], estError[index][1]);
    // exit(1);
  }
  for(j=1; j <= ext_width; j++){
    // extend the visible area by 'ext_width' tile => in all directions
    int* vmask_ext_tmp = extVMask(vmask, j);
    for(i=0; i < tiles; i++){
      if(j==1){
	if(vmask_ext_tmp[i] == 1){
	  if(vmask[i] == 0)
	    vmask_ext[i] += j+1;
	  else
	    vmask_ext[i] = 1;
	}else
	  vmask_ext[i] = 0;
      }else{
	if(vmask_ext_tmp[i] == 1){

	  if(vmask_ext[i] == 0)
	    vmask_ext[i] += j+1;
	}
      }
    }
  }
  vmask = vmask_ext;
  printf("#########vmask_adjusted:\n");
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      printf("%d ", vmask[i * rows + j]);
    }
    printf("\n");
  }
  printf("\n");
  // get the selected version when all tile are of equal quality
  tileVer = decideTileVer_ROI(index, vmask, est_thrp[index]);
  // calculate viewport-psnr given 'tileVer'
  int* tmp_vp = (int*) malloc(2 * sizeof(int));
  // first frame & last frame
  tmp_vp[0] = est_vp_frame[index * INTERVAL][0];
  tmp_vp[1] = est_vp_frame[index * INTERVAL][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
  tmp_vp[0] = est_vp_frame[(index + 1) * INTERVAL - 1][0] + estError[index][0];
  tmp_vp[1] = est_vp_frame[(index + 1) * INTERVAL - 1][1] + estError[index][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
  // update selection
  if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
    std::copy (tileVer, tileVer + tiles, selectTileVer);
    max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
  }
  printf("#########tileVer:\n");
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      printf("%d ", tileVer[i * rows + j]);
    }
    printf("\n");
  }
  printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
  //p
  if(ext_width == 1){
    // determine quality of visible part
    for(i=0; i < tiles; i++)
      if(vmask[i] > 0 && tileVer[i] < mean_ver)
	mean_ver = tileVer[i];
    // search for the best (viewport-version, extesion-version) pair
    vp_ver = mean_ver + 1;
    while(vp_ver < metadata->NO_VER){
      remainBW = BW;
      // assign quality for visible and non-visible tiles
      for(i=0; i < tiles; i++){
	if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
	else
	  tileVer[i] = 0; // minimum quality for non-visible tiles
	remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
      }
      //
      if(remainBW <= 0){
	// reduce vp tiles's version to meet the bandwidth constraint
	for(i=0; i < tiles; i++){
	  if(vmask[i] == 1){
	    tileVer[i] -= 1; // assume that all tiles have same priority
	    remainBW += metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
	    if(remainBW > 0)
	      break;
	  }
	}
	// calculate viewport-psnr given 'tileVer'
	int tmp_vp[2];
	// first frame & last frame
	  tmp_vp[0] = est_vp_frame[index * INTERVAL][0];
	  tmp_vp[1] = est_vp_frame[index * INTERVAL][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
	  vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
	  tmp_vp[0] = est_vp_frame[(index + 1) * INTERVAL - 1][0] + estError[index][0];
	  tmp_vp[1] = est_vp_frame[(index + 1) * INTERVAL - 1][1] + estError[index][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
	  vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
	// update selection
	if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	  std::copy (tileVer, tileVer + tiles, selectTileVer);
	  max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
	}
	break; // not enough bandwidth
      }
      // assign remaining bandwidth for ext_1 tiles
      FLAG = false;
      while(remainBW > 0 && !FLAG){
	FLAG = true;
	for(i=0; i < metadata->vp->No_tile; i++){
	  if(vmask[i] == 2 && remainBW > 0){
	    if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
	      remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
	      tileVer[i] ++;
	      FLAG = false;
	    }
	  }
	}
      }
      // assign remaining bandwidth to non-visible tiles
      if(remainBW > 0){
	FLAG = false;
	while(remainBW > 0 && !FLAG){
	  FLAG = true;
	  for(i=0; i < metadata->vp->No_tile; i++){
	    if(vmask[i] == 0 && remainBW > 0){
	      if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		tileVer[i] ++;
		FLAG = false;
	      }
	    }
	  }
	}
      }
      // calculate viewport-psnr given 'tileVer'
      // first frame & last frame
  tmp_vp[0] = est_vp_frame[index * INTERVAL][0];
  tmp_vp[1] = est_vp_frame[index * INTERVAL][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
  tmp_vp[0] = est_vp_frame[(index + 1) * INTERVAL - 1][0] + estError[index][0];
  tmp_vp[1] = est_vp_frame[(index + 1) * INTERVAL - 1][1] + estError[index][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
      // update selection
      if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	std::copy (tileVer, tileVer + tiles, selectTileVer);
	max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
      }
      //
      printf("#########tileVer: vp_ver=%d\n", vp_ver);
      for(i=0; i < rows; i++){
	for(j=0; j < cols; j++){
	  printf("%d ", tileVer[i * rows + j]);
	}
	printf("\n");
      }
      printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
      //
      vp_ver ++;
    }//endwhile
  }
  else{
    // determine quality of visible part
    for(i=0; i < tiles; i++)
      if(vmask[i] > 0 && tileVer[i] < mean_ver)
	mean_ver = tileVer[i];
    // search for the best (viewport-version, ext1-version, ext2-version) tube
    vp_ver = mean_ver + 1;
    while(vp_ver < metadata->NO_VER){
      remainBW = BW;
      // assign quality for visible and non-visible tiles
      for(i=0; i < tiles; i++){
	if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
	else
	  tileVer[i] = 0; // minimum quality for non-visible tiles
	remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
      }
      //
      if(remainBW <= 0) break;
      // determine quality if 'ext1' and 'ext2' are assigned same version
      // equally assign remaining bandwidth for 'ext_1' and 'ext_2'
      FLAG = false;
      while(remainBW > 0 && !FLAG){
	FLAG = true;
	for(i=0; i < metadata->vp->No_tile; i++){
	  if((vmask[i] == 2 || vmask[i] == 3) && remainBW > 0){
	    if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
	      remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
	      tileVer[i] ++;
	      FLAG = false;
	    }
	  }
	}
      }
      // find version
      for(i=0; i < tiles; i++)
	if((vmask[i] == 2 || vmask[i] == 3) && tileVer[i] < mean_ver_2)
	  mean_ver_2 = tileVer[i];
      // assign remaining bandwidth to non-visible tiles
      if(remainBW > 0){
	FLAG = false;
	while(remainBW > 0 && !FLAG){
	  FLAG = true;
	  for(i=0; i < metadata->vp->No_tile; i++){
	    if(vmask[i] == 0 && remainBW > 0){
	      if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		tileVer[i] ++;
		FLAG = false;
	      }
	    }
	  }
	}
      }
      //
      // calculate viewport-psnr given 'tileVer'
      // first frame & last frame
  tmp_vp[0] = est_vp_frame[index * INTERVAL][0];
  tmp_vp[1] = est_vp_frame[index * INTERVAL][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
  tmp_vp[0] = est_vp_frame[(index + 1) * INTERVAL - 1][0] + estError[index][0];
  tmp_vp[1] = est_vp_frame[(index + 1) * INTERVAL - 1][1] + estError[index][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
	// update selection
      if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	std::copy (tileVer, tileVer + tiles, selectTileVer);
	max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
      }
      printf("#########tileVer: vp_ver:%d\n", vp_ver);
      for(i=0; i < rows; i++){
	for(j=0; j < cols; j++){
	  printf("%d ", tileVer[i * rows + j]);
	}
	printf("\n");
      }
      printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
      //
      ext_1_ver = mean_ver_2 + 1;
      while(ext_1_ver <= vp_ver){
	remainBW = BW;
	// assign quality for visible and non-visible tiles
	for(i=0; i < tiles; i++){
	  if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
	  else
	    if(vmask[i] == 2) tileVer[i] = ext_1_ver; // assign quality for ext_1 tiles
	    else
	      tileVer[i] = 0;
	  remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
	}
	//
	if(remainBW <= 0) break;
	// assign remaining bandwidth for 'ext_2' tiles
	FLAG = false;
	while(remainBW > 0 && !FLAG){
	  FLAG = true;
	  for(i=0; i < metadata->vp->No_tile; i++){
	    if(vmask[i] == 3 && remainBW > 0){
	      if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		tileVer[i] ++;
		FLAG = false;
	      }
	    }
	  }
	}
	// assign remaining bandwidth to non-visible tiles
	if(remainBW > 0){
	  FLAG = false;
	  while(remainBW > 0 && !FLAG){
	    FLAG = true;
	    for(i=0; i < metadata->vp->No_tile; i++){
	      if(vmask[i] == 0 && remainBW > 0){
		if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
		  remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
		  tileVer[i] ++;
		  FLAG = false;
		}
	      }
	    }
	  }
	}
	// calculate viewport-psnr given 'tileVer'
	// first frame & last frame
	tmp_vp[0] = est_vp_frame[index * INTERVAL][0];
	tmp_vp[1] = est_vp_frame[index * INTERVAL][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
	vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
	tmp_vp[0] = est_vp_frame[(index + 1) * INTERVAL - 1][0] + estError[index][0];
	tmp_vp[1] = est_vp_frame[(index + 1) * INTERVAL - 1][1] + estError[index][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
	vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
	// update selection
	if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
	  std::copy (tileVer, tileVer + tiles, selectTileVer);
	  max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
	}
	//
	printf("#########tileVer: vp_ver:%d ext_1_ver:%d\n", vp_ver, ext_1_ver);
	for(i=0; i < rows; i++){
	  for(j=0; j < cols; j++){
	    printf("%d ", tileVer[i * rows + j]);
	  }
	  printf("\n");
	}
	printf("\nest_vp:%d\testError:%.2f\tvp_psnr:%.2f\t max_vp_psnr:%.2f\n",est_vp[index][0], estError[index][0], vp_psnr, max_vp_psnr);
	ext_1_ver ++;
      }
      vp_ver++;
    }
  }
  printf("#########selecteTileVer:\n");
  for(i=0; i < rows; i++){
    for(j=0; j < cols; j++){
      printf("%d ", selectTileVer[i * rows + j]);
    }
    printf("\n");
  }
  //
  return selectTileVer;
}

template <typename T>
std::vector<int> AdaptationLogic::sort_index(std::vector<T> const& values) {
    std::vector<int> indices(values.size());
    std::iota(begin(indices), end(indices), static_cast<int>(0));
    std::sort(
        begin(indices), end(indices),
        [&](int a, int b) { return values[a] > values[b]; }
    );
    return indices;
}
int* AdaptationLogic::ISM(int index){
  int width;
  int**tileVer = new int*[2];
  int* selectTileVer = NULL;
  double max_psnr = 0;
  double vp_psnr;
  for(width=1; width <= 2; width ++){
    tileVer[width-1] = ISM(index, width);
    printf("est_vp: (%d, %d)\n", est_vp[index][0], est_vp[index][1]);
    vp_psnr = estimateViewportPSNR(index, tileVer[width-1], est_vp_frame[index * INTERVAL]) + estimateViewportPSNR(index, tileVer[width-1], est_vp_frame[(index+1) * INTERVAL -1]);
    //vp_psnr = estimateViewportPSNR(index, tileVer[width-1], est_vp_frame[index * INTERVAL]);
    if(vp_psnr > max_psnr){
      selectTileVer = tileVer[width-1];
      max_psnr = vp_psnr;
      decide_width[index] = width;
    }
  }
  return selectTileVer;
}
int* AdaptationLogic::ISM_error(int index){
  int width;
  int**tileVer = new int*[2];
  int* selectTileVer = NULL;
  double max_psnr = 0;
  double vp_psnr;
  double vp_psnr_last;
  int* tmp_vp = (int*) malloc(2 * sizeof(int));
  for(width=1; width <= 2; width ++){
    tileVer[width-1] = ISM_error(index, width);
    printf("est_vp: (%d, %d)\n", est_vp[index][0], est_vp[index][1]);
  tmp_vp[0] = est_vp_frame[index * INTERVAL][0] + estError[index][0];
  tmp_vp[1] = est_vp_frame[index * INTERVAL][1] + estError[index][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr = estimateViewportPSNR(index, tileVer[width-1], tmp_vp);
  tmp_vp[0] = est_vp_frame[(index + 1) * INTERVAL - 1][0] + estError[index][0];
  tmp_vp[1] = est_vp_frame[(index + 1) * INTERVAL - 1][1] + estError[index][1];
  if(tmp_vp[0] >= 180)
    tmp_vp[0] -= 360;
  if(tmp_vp[0] < -180)
    tmp_vp[0] += 360;
  //
  if(tmp_vp[1] >= 90)
    tmp_vp[1] -= 180;
  if(tmp_vp[1] <= -90)//
    tmp_vp[1] += 180;
  vp_psnr_last = estimateViewportPSNR(index, tileVer[width-1], tmp_vp);
  //
  printf("index=%d width=%d: %.2f\n",index, width, vp_psnr+vp_psnr_last);
  if((vp_psnr + vp_psnr_last) > max_psnr){
      selectTileVer = tileVer[width-1];
      max_psnr = vp_psnr + vp_psnr_last;
      decide_width[index] = width;
    }
  }
  printf("index=%d select: max_psnr=%.2f w=%d\n", index, max_psnr, decide_width[index]);
  return selectTileVer;
}
int* AdaptationLogic::extVmask_ext(int* vmask,int ext_width, int NUM_PART){
    int* ret = new int[metadata->vp->No_tile];
  int M = metadata->vp->No_tile_v;
  int N = metadata->vp->No_tile_h;
  int i,j;
  struct Pos pos;
  if(ext_width == 0) return vmask;
  if(ext_width >= 3){
    for(i=0; i < metadata->vp->No_tile; i++)
      ret[i] = 1;
    return ret;
  }
  // vmask after extension
  int** vmask_ext = iniTwoDimenArrayInt(metadata->vp->No_tile_v, metadata->vp->No_tile_h);
  // convert 'vmask' to rectangle form
  //printf("#[ExtendVisibleMask_v2]: vmask\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      vmask_ext[i][j] = vmask[i*M + j];
      //printf("%d ", vmask[i*M + j]);
    }
    //printf("\n");
  }
  //
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//TOP
	pos = TOP(i,j,M,N);
	//printf("#[ExtendVisibleMask_v2]: TOP(i,j)=(%d,%d)\n", pos.i,pos.j);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 2;
      }
    }
  }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//DOWN
	pos = DOWN(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 3;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 4;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 5;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//TOP-LEFT
	pos = TOP_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 4;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//TOP-RIGHT
	pos = TOP_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 5;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//DOWN-LEFT
	pos = DOWN_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 4;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] == 1){
	//DOWN-RIGHT
	pos = DOWN_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 5;
      }
    }
 }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      // if(vmask_ext[i][j] == 2)
      // 	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  if(ext_width == 1) return ret;
  // extension width = 2
 //
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//printf("#[ExtendVisibleMask_v2]: (i,j)=(%d,%d)\n", i,j);
	//TOP
	pos = TOP(i,j,M,N);
	//printf("#[ExtendVisibleMask_v2]: TOP(i,j)=(%d,%d)\n", pos.i,pos.j);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 6;
      }
    }
  }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//DOWN
	pos = DOWN(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 7;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//LEFT
	pos = LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 8;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//RIGHT
	pos = RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 9;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//TOP-LEFT
	pos = TOP_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 8;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//TOP-RIGHT
	pos = TOP_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 9;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//DOWN-LEFT
	pos = DOWN_LEFT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 8;
      }
    }
 }
for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      if(vmask_ext[i][j] >= 1 && vmask_ext[i][j] <= 5){
	//DOWN-RIGHT
	pos = DOWN_RIGHT(i,j,M,N);
	if(pos.i != -1 && vmask_ext[pos.i][pos.j] == 0)
	  vmask_ext[pos.i][pos.j] = 9;
      }
    }
 }
  // 2->1
  //printf("#[ExtendVisibleMask_v2]: vmask_ext_2\n");
  for(i=0; i < M; i++){
    for(j=0; j < N; j++){
      // if(vmask_ext[i][j] == 2)
      // 	vmask_ext[i][j] = 1;
      ret[i*N + j]= vmask_ext[i][j];
      //printf("%d ", vmask_ext[i][j]);
    }
    //printf("\n"); 
  }
  return ret;
}
int* AdaptationLogic::ISM_ext(int index){
  int NUM_PART = 9;
  int i[100], j, ii;
  int EXT_W = 2;
  double remainBW, tmpsum, max_vpPSNR=0, vpPSNR;
  int N = metadata->vp->No_tile;
  int NUM_VER = metadata->NO_VER;
  int* tileVer = new int[N];
  int* ret = new int[N];
  int* vmask = getVisibleTile(est_vp[index]);
  int* adapt_area = extVmask_ext(vmask, EXT_W, NUM_PART);
  if(index == 0){
    for(j=0; j < N; j++)
      ret[j] = 0;
    return ret;
  }
  //
  /*
  printf("#[ISM_ext]: index=%d est_thrp=%.2f(kbps)\n", index, est_thrp[index]);
  for(j=0; j < N; j++){
    printf("%d ", adapt_area[j]);
    if((j+1) % 8 == 0) printf("\n");
  }
  */
  // allocate minimum version for background tiles
  remainBW = est_thrp[index];
  for(j=0; j < N; j++){
    if(adapt_area[j] == 0){
      tileVer[j] = 0;
      remainBW -= metadata->TILE_BR[index][j][0];
    }
  }
  // loop to find optimal versions for other areas
  switch(NUM_PART){
  case 9:
    for(i[0] = 0; i[0] < NUM_VER; i[0]++){//1
      //
      // tmpsum = 0;      
      // for(j=0; j < N; j++){
      // 	if(adapt_area[j] == 1){
      // 	  tmpsum += metadata->TILE_BR[index][j][i[0]];
      // 	}
      // }
      // if(tmpsum >= remainBW) continue;
      //
      for(i[1] = 0; i[1] < NUM_VER; i[1]++){//2
	//
	// for(j=0; j < N; j++){
	//   if(adapt_area[j] == 2){
	//     tmpsum += metadata->TILE_BR[index][j][i[1]];
	//   }
	// }
	// if(tmpsum >= remainBW) continue;
	//
	for(i[2] = 0; i[2] < NUM_VER; i[2]++){//2
	  //
	  // for(j=0; j < N; j++){
	  //   if(adapt_area[j] == 3){
	  //     tmpsum += metadata->TILE_BR[index][j][i[2]];
	  //   }
	  // }
	  // if(tmpsum >= remainBW) continue;
	  //
	  for(i[3] = 0; i[3] < NUM_VER; i[3]++){//4
	    //
	    // for(j=0; j < N; j++){
	    //   if(adapt_area[j] == 4){
	    // 	tmpsum += metadata->TILE_BR[index][j][i[3]];
	    //   }
	    // }
	    // if(tmpsum >= remainBW) continue;
	    //
	    for(i[4] = 0; i[4] < NUM_VER; i[4]++){//5
	      //
	      // for(j=0; j < N; j++){
	      // 	if(adapt_area[j] == 5){
	      // 	  tmpsum += metadata->TILE_BR[index][j][i[4]];
	      // 	}
	      // }
	      // if(tmpsum >= remainBW) continue;
	      //
	      for(i[5] = 0; i[5] < NUM_VER; i[5]++){//6
		//
		// for(j=0; j < N; j++){
		//   if(adapt_area[j] == 6){
		//     tmpsum += metadata->TILE_BR[index][j][i[5]];
		//   }
		// }
		// if(tmpsum >= remainBW) continue;
		//
		for(i[6] = 0; i[6] < NUM_VER; i[6]++){//7
		  //
		  // for(j=0; j < N; j++){
		  //   if(adapt_area[j] == 7){
		  //     tmpsum += metadata->TILE_BR[index][j][i[6]];
		  //   }
		  // }
		  // if(tmpsum >= remainBW) continue;
		  //
		  for(i[7] = 0; i[7] < NUM_VER; i[7]++){//8
		    //
		    // for(j=0; j < N; j++){
		    //   if(adapt_area[j] == 8){
		    // 	tmpsum += metadata->TILE_BR[index][j][i[7]];
		    //   }
		    // }
		    // if(tmpsum >= remainBW) continue;
		    //
		    for(i[8] = 0; i[8] < NUM_VER; i[8]++){
		      //
		       // for(j=0; j < N; j++){
		       // 	if(adapt_area[j] == 9){
		       // 	  tmpsum += metadata->TILE_BR[index][j][i[8]];
		       // 	}
		       // }
		       // if(tmpsum >= remainBW) continue;
		       // assign version for each area
		      tmpsum = 0;
		      for(j=0; j < N; j++){
			switch(adapt_area[j]){
			case 1:
			  tileVer[j] = i[0];
			  break;
			case 2:
			  tileVer[j] = i[1];
			  break;
			case 3:
			  tileVer[j] = i[2];
			  break;
			case 4:
			  tileVer[j] = i[3];
			  break;
			case 5:
			  tileVer[j] = i[4];
			  break;
			case 6:
			  tileVer[j] = i[5];
			  break;
			case 7:
			  tileVer[j] = i[6];
			  break;
			case 8:
			  tileVer[j] = i[7];
			  break;
			case 9:
			  tileVer[j] = i[8];
			  break;
			}
			tmpsum += metadata->TILE_BR[index][j][tileVer[j]];
		      }
		      //
		      if(tmpsum <=remainBW){
			//
			printf("#Candidate:\n");
			for(ii=0; ii < N; ii++){
			  printf("%d ", tileVer[ii]);
			  if((ii+1) % 8 == 0) printf("\n");
			}
			// compute viewport PSNR
			vpPSNR =  estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL]) +  estimateViewportPSNR(index, tileVer, est_vp_frame[(index + 1) * INTERVAL]);
			if(vpPSNR > max_vpPSNR){
			  for(j=0; j < N; j++)
			    ret[j] = tileVer[j];
			  max_vpPSNR = vpPSNR;
			}
			printf("#vpPSNR: %.2f max_vpPSNR: %.2f\n", vpPSNR, max_vpPSNR);
		      }
		      //
		    }//9
		  }//8
		}//7
	      }//6
	    }//5
	  }//4
	}//3
      }//2
    }//1
    break;
  }
  /*
  printf("#select_version:\n");
  for(ii=0; ii < N; ii++){
    printf("%d ", ret[ii]);
    if((ii+1) % 8 == 0) printf("\n");
  }
  */
  return ret;
}
int* AdaptationLogic::EXT_ALL(int index, int ext_w){
  	int i,j;
	int select_ver;
	int* tile = new int[metadata->vp->No_tile];
	int* vmask = getVisibleTile(est_vp[index]);
	int* vmask_ext = extVMask(vmask, ext_w);
	tile = decideTileVer_ROI(index, vmask_ext, est_thrp[index]);
	return tile;
}
int* AdaptationLogic::EXT_LEFT_RIGHT(int index, int ext_w){
	int i,j;
	int select_ver;
	int* tile = new int[metadata->vp->No_tile];
	int* vmask = getVisibleTile(est_vp[index]);
	int* vmask_ext = extVMask_LR(vmask, ext_w);
	tile = decideTileVer_ROI(index, vmask_ext, est_thrp[index]);
	return tile;
}
int* AdaptationLogic::EXT_LEFT(int index, int ext_w){
	int i,j;
	int select_ver;
	int* tile = new int[metadata->vp->No_tile];
	int* vmask = getVisibleTile(est_vp[index]);
	int* vmask_ext = extVMask_L(vmask, ext_w);
	tile = decideTileVer_ROI(index, vmask_ext, est_thrp[index]);
	return tile;
}
int* AdaptationLogic::EXT_RIGHT(int index, int ext_w){
	int i,j;
	int select_ver;
	int* tile = new int[metadata->vp->No_tile];
	int* vmask = getVisibleTile(est_vp[index]);
	int* vmask_ext = extVMask_R(vmask, ext_w);
	tile = decideTileVer_ROI(index, vmask_ext, est_thrp[index]);
	return tile;
}
int* AdaptationLogic::Proposed(int index){
  int* adapt_area = new int[metadata->vp->No_tile];
  int* vmask;
  int* tile_ver = new int[metadata->vp->No_tile];
  int i,j;
  // and determine the adapt area
  for(j=0; j < metadata->vp->No_tile; j++)
    adapt_area[j] = 0;
  //
  for(i=0; i < INTERVAL; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1)
	adapt_area[j] = 1;
    }
  }
  printf("#[EXT_Adapt_Area_try2]: adapt_area\n");
  for(i=0; i < metadata->vp->No_tile_v; i++){
    for(j=0; j < metadata->vp->No_tile_h; j++){
      printf("%d ", adapt_area[i*metadata->vp->No_tile_h + j]);
    }
    printf("\n");
  }
  // Extend the adapt are (optional)
  
  // Determine quality levels for tiles (lowest for background, EQUAL-based for the adapt area)
  tile_ver = decideTileVer_ROI(index, adapt_area, est_thrp[index]);
  return tile_ver;
}
int* AdaptationLogic::decideTileVer_ROI(int index, int* vmask_ext, double est_thrp){
  int i,j;
  int select_ver;
  int* tile = new int[metadata->vp->No_tile];
  double sum;
  double delta_R;
  if(index == 0){
    for(i=0; i < metadata->vp->No_tile; i++){
      tile[i] = 0;
    }
  }else{
    // allocate the minimum version for tiles not in T_h
    sum = 0;
    for(i=0; i < metadata->vp->No_tile; i++){
      if(vmask_ext[i] == 0){
	tile[i] = 0;
	sum += metadata->TILE_BR[index][i][tile[i]];
      }
    }
    delta_R = est_thrp - sum;
    // Apply equal to the remaining tiles
    for(j=0; j < metadata->NO_VER; j++){
      sum = 0;
      for(i=0; i < metadata->vp->No_tile; i++){
	if(vmask_ext[i] > 0){
	  sum += metadata->TILE_BR[index][i][j];
	}
      }
      if(sum > delta_R)
	break;
    }
    //
    sum = 0;
    for(i=0; i < metadata->vp->No_tile; i++){
      if(vmask_ext[i] > 0){
	tile[i] = j - 1;
	sum += metadata->TILE_BR[index][i][tile[i]];
      }
    }
    delta_R -= sum;
    //
    bool FLAG = true;
    while(delta_R > 0){
      FLAG = true;
      for(i=0; i < metadata->vp->No_tile; i++){
	if(vmask_ext[i] == 1){
	  if(tile[i] < metadata->NO_VER -1 && (metadata->TILE_BR[index][i][tile[i] + 1] - metadata->TILE_BR[index][i][tile[i]]) <= delta_R){
	    delta_R -= (metadata->TILE_BR[index][i][tile[i] + 1] - metadata->TILE_BR[index][i][tile[i]]);
	    tile[i] += 1;
	    FLAG = false;
	  }
	}
      }
      if(FLAG) break; // break if no improvement is done in the last round.
    }
   while(delta_R > 0){
       FLAG = true;
       for(i=0; i < metadata->vp->No_tile; i++){
    	if(vmask_ext[i] == 0){
    	  if(tile[i] < metadata->NO_VER -1 && (metadata->TILE_BR[index][i][tile[i] + 1] - metadata->TILE_BR[index][i][tile[i]]) <= delta_R){
    	    delta_R -= (metadata->TILE_BR[index][i][tile[i] + 1] - metadata->TILE_BR[index][i][tile[i]]);
    	    tile[i] += 1;
    	    FLAG = false;
    	  }
    	}
       }
       if(FLAG) break; // break if no improvement is done in the last round.
     }
  }
  return tile;
}
int* AdaptationLogic::ROI_first(int index){
  // use the first frame's viewport as the representative viewport for the interval
  est_vp[index] = est_vp_frame[index * INTERVAL]; 
  return EXT_ALL(index, 0);
}
int* AdaptationLogic::Proposed_v2(int index){
  int* adapt_area = new int[metadata->vp->No_tile];
  int* vmask;
  int* tile_ver = new int[metadata->vp->No_tile];
  int i,j;
  int K = INTERVAL/2;
  // and determine the adapt area
  for(j=0; j < metadata->vp->No_tile; j++)
    adapt_area[j] = 0;
  // mask visible tiles of first 'K' frames with prio of 1
  for(i=0; i < K; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1 && adapt_area[j] == 0)
	adapt_area[j] = 1;
    }
  }
  // mask visible tiles of remain 'INTERVAL - K' frames with prio of 2
  for(i=K; i < INTERVAL; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1 && adapt_area[j] == 0)
	adapt_area[j] = 2;
    }
  }
  printf("#[Proposed_v2]: index=%d adapt_area\n", index);
  for(i=0; i < metadata->vp->No_tile_v; i++){
    for(j=0; j < metadata->vp->No_tile_h; j++){
      printf("%d ", adapt_area[i*metadata->vp->No_tile_h + j]);
    }
    printf("\n");
  }
  // Extend the adapt are (optional)
  
  // Determine quality levels for tiles (lower priority --> higher quality)
  // search for the best quality level of each area 
  tile_ver = decideTileVer_EXT(index, adapt_area, est_thrp[index]);
  return tile_ver;
}
int* AdaptationLogic::Proposed_v3(int index){
  int* adapt_area = new int[metadata->vp->No_tile];
  int* vmask;
  int* tile_ver = new int[metadata->vp->No_tile];
  int i,j;
  int K = INTERVAL/2;
  // and determine the adapt area
  for(j=0; j < metadata->vp->No_tile; j++)
    adapt_area[j] = 0;
  // mask visible tiles of first 'K' frames with prio of 2
  for(i=0; i < K; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1 && adapt_area[j] == 0)
	adapt_area[j] = 2;
    }
  }
  // mask visible tiles of remain 'INTERVAL - K' frames with prio of 1
  for(i=K; i < INTERVAL; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1 && adapt_area[j] == 0)
	adapt_area[j] = 1;
    }
  }
  printf("#[]: adapt_area\n");
  for(i=0; i < metadata->vp->No_tile_v; i++){
    for(j=0; j < metadata->vp->No_tile_h; j++){
      printf("%d ", adapt_area[i*metadata->vp->No_tile_h + j]);
    }
    printf("\n");
  }
  // Extend the adapt are (optional)
  
  // Determine quality levels for tiles (lower priority --> higher quality)
  // search for the best quality level of each area 
  tile_ver = decideTileVer_EXT(index, adapt_area, est_thrp[index]);
  return tile_ver;
}
int* AdaptationLogic::Proposed_v4(int index){
  int* adapt_area = new int[metadata->vp->No_tile];
  int* vmask;
  int* tile_ver = new int[metadata->vp->No_tile];
  int i,j;
  int K = INTERVAL/4;
  // and determine the adapt area
  for(j=0; j < metadata->vp->No_tile; j++)
    adapt_area[j] = 0;
  
  // mask visible tiles of remain frames with prio of 1
  for(i=K; i < 3*K; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1 && adapt_area[j] == 0)
	adapt_area[j] = 1;
    }
  }
  // mask visible tiles of [0-N/4] & [3N/4,N] frames with prio of 2
  for(i=0; i < K; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1 && adapt_area[j] == 0)
	adapt_area[j] = 2;
    }
  }
  for(i=3*K; i < INTERVAL; i++){
    vmask = getVisibleTile(est_vp_frame[index * INTERVAL + i]);
    for(j=0; j < metadata->vp->No_tile; j++){
      if(vmask[j] == 1 && adapt_area[j] == 0)
	adapt_area[j] = 2;
    }
  }

  printf("#[EXT_Adapt_Area_try2]: adapt_area\n");
  for(i=0; i < metadata->vp->No_tile_v; i++){
    for(j=0; j < metadata->vp->No_tile_h; j++){
      printf("%d ", adapt_area[i*metadata->vp->No_tile_h + j]);
    }
    printf("\n");
  }
  // Extend the adapt are (optional)
  
  // Determine quality levels for tiles (lower priority --> higher quality)
  // search for the best quality level of each area 
  tile_ver = decideTileVer_EXT(index, adapt_area, est_thrp[index]);
  return tile_ver;
}
int* AdaptationLogic::decideTileVer_EXT(int index, int* adapt_area, double est_thrp){
  int i,j;
  int N = metadata->vp->No_tile;
  int H = metadata->vp->No_tile_h;
  int V = metadata->vp->No_tile_v;
  int* tileVer = new int[N];
  int* tileVer2 = new int[N];
  int* ret = new int[N];
  double maxPSNR = 0, vpPSNR;
  double usedBW = 0;
  double remainBW = 0;
  double BW;
  int BREAK_ALL;
  int L1_Qua; /* quality of layer 1 */
  /* Allocate minimum quality for all tiles */
  for(i=0; i < N; i++){
    tileVer[i] = 0;
    tileVer2[i] = 0;
    usedBW += metadata->TILE_BR[index][i][tileVer[i]];
  }
  if(index == 0)
    return tileVer;
  /* For each quality level of layer 1*/
  remainBW = est_thrp - usedBW;
  for(L1_Qua = 2; L1_Qua < metadata->NO_VER; L1_Qua ++){
    printf("#[decideTileVer_EXT][index=%d] L1_Qua = %d:\n",index, L1_Qua);
    printf("remainBW=%.2f(kbps)\n", remainBW);
    //
    BW = remainBW;
    memcpy(tileVer, tileVer2, N * sizeof(tileVer[0]));
    //
    for(i=0; i < H; i++){
      for(j=0; j < V; j++){
	printf("%d ", tileVer[i*H + j]);
      }
      printf("\t");
      for(j=0; j < V; j++){
	printf("%d ", adapt_area[i*H + j]);
      }
      printf("\n");
    }
    printf("\n");
    /* select version for L1 tiles */
    tileVer = selectTileVersion(index, tileVer, adapt_area, 1, L1_Qua, &BW);
    printf("Layer #1:\n");
    for(i=0; i < H; i++){
      for(j=0; j < V; j++){
	printf("%d ", tileVer[i*H + j]);
      }
      printf("\t");
      for(j=0; j < V; j++){
	printf("%d ", adapt_area[i*H + j]);
      }
      printf("\n");
    }
    printf("remainBW=%.2f(kbps)\n", BW);
    printf("\n");
    /* select version for L2 tiles */
    printf("Layer #2:\n");
    tileVer = selectTileVersion(index, tileVer, adapt_area, 2, &BW);
    for(i=0; i < H; i++){
      for(j=0; j < V; j++){
	printf("%d ", tileVer[i*H + j]);
      }
      printf("\t");
      for(j=0; j < V; j++){
	printf("%d ", adapt_area[i*H + j]);
      }
      printf("\n");
    }
    printf("remainBW=%.2f(kbps)\n", BW);
    printf("\n");
    //
    while(BW > 0){
      BREAK_ALL = 1;
      for(i=0; i < N; i++){
	if(adapt_area[i] == 1){
	  if(tileVer[i] <  metadata->NO_VER-1 && (metadata->TILE_BR[index][i][tileVer[i] + 1] - metadata->TILE_BR[index][i][tileVer[i]]) < BW){
	    BW -= (metadata->TILE_BR[index][i][tileVer[i] + 1] - metadata->TILE_BR[index][i][tileVer[i]]);
	    tileVer[i] ++;
	    BREAK_ALL = 0;
	  }
	}
      }
      //
      for(i=0; i < N; i++){
	if(adapt_area[i] == 2){
	  if(tileVer[i] <  metadata->NO_VER-1 && (metadata->TILE_BR[index][i][tileVer[i] + 1] - metadata->TILE_BR[index][i][tileVer[i]]) < BW){
	    BW -= (metadata->TILE_BR[index][i][tileVer[i] + 1] - metadata->TILE_BR[index][i][tileVer[i]]);
	    tileVer[i] ++;
	    BREAK_ALL = 0;
	  }
	}
      }
      //
      for(i=0; i < N; i++){
	if(adapt_area[i] == 0){
	  if(tileVer[i] < metadata->NO_VER-1 && (metadata->TILE_BR[index][i][tileVer[i] + 1] - metadata->TILE_BR[index][i][tileVer[i]]) < BW){
	    BW -= (metadata->TILE_BR[index][i][tileVer[i] + 1] - metadata->TILE_BR[index][i][tileVer[i]]);
	    tileVer[i] ++;
	    BREAK_ALL = 0;
	  }
	}
      }
      if(BREAK_ALL == 1) 
	break;
    }
    /* calculate and update optimal tile version */
    vpPSNR = 0;
    for(i=0; i < INTERVAL; i++)
      vpPSNR += 1.0/INTERVAL * estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL + i]);
    printf("vpPSNR:%.2f\tmaxPSNR:%.2f\n", vpPSNR, maxPSNR);
    if(vpPSNR > maxPSNR){
      maxPSNR = vpPSNR;
      for(i=0; i < N; i++)
	ret[i] = tileVer[i];
    }
    printf("vpPSNR:%.2f\tmaxPSNR:%.2f\n", vpPSNR, maxPSNR);
  }
  for(i=0; i < H; i++){
    for(j=0; j < V; j++){
      printf("%d ", ret[i*H + j]);
    }
    printf("\n");
  }
  printf("remainBW=%.2f(kbps)\n", BW);
  printf("\n");
 
  return ret;
}
int* AdaptationLogic::selectTileVersion(int index, int* tileVer, int* adapt_area, int layer, int Q, double* BW){
  int i,j;
  int N = metadata->vp->No_tile;
  int BREAK_ALL;
  int* ret = new int[N];
  printf("size=%d(bytes)\n", N * sizeof(tileVer[0]));
  memcpy(ret, tileVer, N * sizeof(tileVer[0]));
  //
  for(i=0; i < N; i++){
    if(adapt_area[i] == layer){
      ret[i] = Q;
      *BW -= (metadata->TILE_BR[index][i][ret[i]] - metadata->TILE_BR[index][i][0]);
    }
  }
  printf("#[selectTileVersion]: remainBW=%.2f(kbps)\n", *BW);
  if(*BW < 0){
    BREAK_ALL = 0;
    while(1){
      for(i=0; i < N; i++){
	if(adapt_area[i] == layer){
	  ret[i] -= 1;
	  *BW += (metadata->TILE_BR[index][i][ret[i] + 1] - metadata->TILE_BR[index][i][ret[i]]);
	  if(*BW > 0){
	    BREAK_ALL = 1;
	    break;
	  }
	}
      }
      if(BREAK_ALL == 1)
	break;
    }
  }
  return ret;
}
int* AdaptationLogic::selectTileVersion(int index, int* tileVer, int* adapt_area, int layer, double* BW){
  int i;
  int N = metadata->vp->No_tile;
  int BREAK_ALL;
  int* ret = new int[N];
  printf("size=%d(bytes)\n", N * sizeof(tileVer[0]));
  memcpy(ret, tileVer, N * sizeof(tileVer[0]));
  //
  while(1){
    BREAK_ALL = 1;
    for(i=0; i < N; i++){
      if(adapt_area[i] == layer){
	if(ret[i] < metadata->NO_VER-1 && metadata->TILE_BR[index][i][ret[i] + 1] - metadata->TILE_BR[index][i][ret[i]] < *BW){
	  *BW -= (metadata->TILE_BR[index][i][ret[i] + 1] - metadata->TILE_BR[index][i][ret[i]]);
	  ret[i] ++;
	  BREAK_ALL = 0;
	}
      }
    }
    if(BREAK_ALL == 1) break;
  }
  return ret;
}
int* AdaptationLogic::proposed_3(int index, int ext_width){
	int i, j, ii, jj;
	int rows = metadata->vp->No_tile_v;
	int cols = metadata->vp->No_tile_h;
	int tiles = rows * cols;
	int *tileVer = NULL;
	int *vmask_ext = new int[tiles];
	int mean_ver = metadata->NO_VER;
	int mean_ver_2 = metadata->NO_VER;
	int vp_ver;
	int ext_1_ver;
	double BW = est_thrp[index];
	double remainBW;
	bool FLAG;
	int *selectTileVer = new int[tiles];
	double max_vp_psnr = 0;
	double vp_psnr;
	double vp_psnr_last;
	//
	int width = 0;
	// get visible mask with estimated viewport
	printf("#[seg #%d]:\n", index);
	int *vmask = getVisibleTile(est_vp[index]);
	printf("#########vmask:\n");
	for(i=0; i < rows; i++){
		for(j=0; j < cols; j++){
			printf("%d ", vmask[i * rows + j]);
		}
		printf("\n");
	}
	printf("\n");
	for(j=1; j <= ext_width; j++){
		// extend the visible area by 'ext_width' tile => in all directions
		int* vmask_ext_tmp = extendVisibleMask(vmask, j);
			for(i=0; i < tiles; i++){
			if(j==1){
				if(vmask_ext_tmp[i] == 1){
					if(vmask[i] == 0)
						vmask_ext[i] += j+1;
					else
						vmask_ext[i] = 1;
				}else
					vmask_ext[i] = 0;
			}else{
				if(vmask_ext_tmp[i] == 1){
					if(vmask_ext[i] == 0)
						vmask_ext[i] += j+1;
				}
			}
		}
	}
	vmask = vmask_ext;
	printf("#########vmask_adjusted:\n");
	for(i=0; i < rows; i++){
		for(j=0; j < cols; j++){
			printf("%d ", vmask[i * rows + j]);
		}
		printf("\n");
	}
	printf("\n");
	// get the selected version when all tile are of equal quality
	tileVer = directRoIBG(index, vmask);
// calculate viewport-psnr given 'tileVer'
	// calculate viewport-psnr given 'tileVer'
	int tmp_vp[2];
	tmp_vp[0] = est_vp[index][0] + estError[index][0];
	tmp_vp[1] = est_vp[index][1] + estError[index][1];
	vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
	//
	tmp_vp[0] = est_vp[index][0] + estError[index][0] + speed[index][0] * INTERVAL;
	tmp_vp[1] = est_vp[index][1] + estError[index][1] + speed[index][1] * INTERVAL;
	vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
	// update selection
	if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
		std::copy (tileVer, tileVer + tiles, selectTileVer);
		max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
	}
	printf("#########tileVer:\n");
	for(i=0; i < rows; i++){
		for(j=0; j < cols; j++){
			printf("%d ", tileVer[i * rows + j]);
		}
		printf("\n");
	}
	printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
	//
	if(ext_width == 1){
		// determine quality of visible part
		for(i=0; i < tiles; i++)
			if(vmask[i] > 0 && tileVer[i] < mean_ver)
				mean_ver = tileVer[i];
		// search for the best (viewport-version, extesion-version) pair
		vp_ver = mean_ver + 1;
		while(vp_ver < metadata->NO_VER){
			remainBW = BW;
			// assign quality for visible and non-visible tiles
			for(i=0; i < tiles; i++){
				if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
				else
					tileVer[i] = 0; // minimum quality for non-visible tiles
				remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
			}
			//
			if(remainBW <= 0){
				// reduce vp tiles's version to meet the bandwidth constraint
				for(i=0; i < tiles; i++){
					if(vmask[i] == 1){
						tileVer[i] -= 1; // assume that all tiles have same priority
						remainBW += metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
						if(remainBW > 0)
							break;
					}
				}
	// calculate viewport-psnr given 'tileVer'
	int tmp_vp[2];
	tmp_vp[0] = est_vp[index][0] + estError[index][0];
	tmp_vp[1] = est_vp[index][1] + estError[index][1];
	vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
	//
	tmp_vp[0] = est_vp[index][0] + estError[index][0] + speed[index][0] * INTERVAL;
	tmp_vp[1] = est_vp[index][1] + estError[index][1] + speed[index][1] * INTERVAL;
	vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
				// update selection
				if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
					std::copy (tileVer, tileVer + tiles, selectTileVer);
					max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
				}
				break; // not enough bandwidth
			}
			// assign remaining bandwidth for ext_1 tiles
			FLAG = false;
			while(remainBW > 0 && !FLAG){
				FLAG = true;
				for(i=0; i < metadata->vp->No_tile; i++){
					if(vmask[i] == 2 && remainBW > 0){
						if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
							remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
							tileVer[i] ++;
							FLAG = false;
						}
					}
				}
			}
			// assign remaining bandwidth to non-visible tiles
			if(remainBW > 0){
				FLAG = false;
				while(remainBW > 0 && !FLAG){
					FLAG = true;
					for(i=0; i < metadata->vp->No_tile; i++){
						if(vmask[i] == 0 && remainBW > 0){
							if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
								remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
								tileVer[i] ++;
								FLAG = false;
							}
						}
					}
				}
			}
	// calculate viewport-psnr given 'tileVer'
	int tmp_vp[2];
	tmp_vp[0] = est_vp[index][0] + estError[index][0];
	tmp_vp[1] = est_vp[index][1] + estError[index][1];
	vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
	//
	tmp_vp[0] = est_vp[index][0] + estError[index][0] + speed[index][0] * INTERVAL;
	tmp_vp[1] = est_vp[index][1] + estError[index][1] + speed[index][1] * INTERVAL;
	vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
			// update selection
			if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
				std::copy (tileVer, tileVer + tiles, selectTileVer);
				max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
			}
			//
			printf("#########tileVer: vp_ver=%d\n", vp_ver);
			for(i=0; i < rows; i++){
				for(j=0; j < cols; j++){
					printf("%d ", tileVer[i * rows + j]);
				}
				printf("\n");
			}
			printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
			//
			vp_ver ++;
		}//endwhile
	}
	else{
		// determine quality of visible part
		for(i=0; i < tiles; i++)
			if(vmask[i] > 0 && tileVer[i] < mean_ver)
				mean_ver = tileVer[i];
		// search for the best (viewport-version, ext1-version, ext2-version) tube
		vp_ver = mean_ver + 1;
		while(vp_ver < metadata->NO_VER){
			remainBW = BW;
			// assign quality for visible and non-visible tiles
			for(i=0; i < tiles; i++){
				if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
				else
					tileVer[i] = 0; // minimum quality for non-visible tiles
				remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
			}
			//
			if(remainBW <= 0) break;
			// determine quality if 'ext1' and 'ext2' are assigned same version
			// equally assign remaining bandwidth for 'ext_1' and 'ext_2'
			FLAG = false;
			while(remainBW > 0 && !FLAG){
				FLAG = true;
				for(i=0; i < metadata->vp->No_tile; i++){
					if((vmask[i] == 2 || vmask[i] == 3) && remainBW > 0){
						if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
							remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
							tileVer[i] ++;
							FLAG = false;
						}
					}
				}
			}
			// find version
			for(i=0; i < tiles; i++)
				if((vmask[i] == 2 || vmask[i] == 3) && tileVer[i] < mean_ver_2)
					mean_ver_2 = tileVer[i];
			// assign remaining bandwidth to non-visible tiles
			if(remainBW > 0){
				FLAG = false;
				while(remainBW > 0 && !FLAG){
					FLAG = true;
					for(i=0; i < metadata->vp->No_tile; i++){
						if(vmask[i] == 0 && remainBW > 0){
							if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
								remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
								tileVer[i] ++;
								FLAG = false;
							}
						}
					}
				}
			}
			//
	// calculate viewport-psnr given 'tileVer'
	int tmp_vp[2];
	tmp_vp[0] = est_vp[index][0] + estError[index][0];
	tmp_vp[1] = est_vp[index][1] + estError[index][1];
	vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
	//
	tmp_vp[0] = est_vp[index][0] + estError[index][0] + speed[index][0] * INTERVAL;
	tmp_vp[1] = est_vp[index][1] + estError[index][1] + speed[index][1] * INTERVAL;
	vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
			// update selection
			if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
				std::copy (tileVer, tileVer + tiles, selectTileVer);
				max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
			}
			printf("#########tileVer: vp_ver:%d\n", vp_ver);
			for(i=0; i < rows; i++){
				for(j=0; j < cols; j++){
					printf("%d ", tileVer[i * rows + j]);
				}
				printf("\n");
			}
			printf("\nvp_psnr:%.2f\t max_vp_psnr:%.2f\n", vp_psnr, max_vp_psnr);
			//
			ext_1_ver = mean_ver_2 + 1;
			while(ext_1_ver <= vp_ver){
				remainBW = BW;
				// assign quality for visible and non-visible tiles
				for(i=0; i < tiles; i++){
					if(vmask[i] == 1) tileVer[i] = vp_ver; // assign a quality of 'vp_ver' for visible tiles
					else
					if(vmask[i] == 2) tileVer[i] = ext_1_ver; // assign quality for ext_1 tiles
					else
						tileVer[i] = 0;
					remainBW -= metadata->TILE_BR[index][i][tileVer[i]]; // update remaining bandwidth
				}
				//
				if(remainBW <= 0) break;
				// assign remaining bandwidth for 'ext_2' tiles
				FLAG = false;
				while(remainBW > 0 && !FLAG){
					FLAG = true;
					for(i=0; i < metadata->vp->No_tile; i++){
						if(vmask[i] == 3 && remainBW > 0){
							if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
								remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
								tileVer[i] ++;
								FLAG = false;
							}
						}
					}
				}
				// assign remaining bandwidth to non-visible tiles
				if(remainBW > 0){
					FLAG = false;
					while(remainBW > 0 && !FLAG){
						FLAG = true;
						for(i=0; i < metadata->vp->No_tile; i++){
							if(vmask[i] == 0 && remainBW > 0){
								if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBW){
									remainBW -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
									tileVer[i] ++;
									FLAG = false;
								}
							}
						}
					}
				}
	// calculate viewport-psnr given 'tileVer'
	int tmp_vp[2];
	tmp_vp[0] = est_vp[index][0] + estError[index][0];
	tmp_vp[1] = est_vp[index][1] + estError[index][1];
	vp_psnr = estimateViewportPSNR(index, tileVer, tmp_vp);
	//
	tmp_vp[0] = est_vp[index][0] + estError[index][0] + speed[index][0] * INTERVAL;
	tmp_vp[1] = est_vp[index][1] + estError[index][1] + speed[index][1] * INTERVAL;
	vp_psnr_last = estimateViewportPSNR(index, tileVer, tmp_vp);
				// update selection
				if((vp_psnr + vp_psnr_last)/2 > max_vp_psnr){
					std::copy (tileVer, tileVer + tiles, selectTileVer);
					max_vp_psnr = (vp_psnr + vp_psnr_last)/2;
				}
				//
				printf("#########tileVer: vp_ver:%d ext_1_ver:%d\n", vp_ver, ext_1_ver);
				for(i=0; i < rows; i++){
					for(j=0; j < cols; j++){
						printf("%d ", tileVer[i * rows + j]);
					}
					printf("\n");
				}
				printf("\nest_vp:%d\testError:%.2f\tvp_psnr:%.2f\t max_vp_psnr:%.2f\n",est_vp[index][0], estError[index][0], vp_psnr, max_vp_psnr);
				ext_1_ver ++;
			}
			vp_ver++;
		}
	}
	printf("#########selecteTileVer:\n");
	for(i=0; i < rows; i++){
		for(j=0; j < cols; j++){
			printf("%d ", selectTileVer[i * rows + j]);
		}
		printf("\n");
	}
	//
	return selectTileVer;
}
int* AdaptationLogic::proposed_3(int index){
	int width;
	int**tileVer = new int*[2];
	int* selectTileVer = NULL;
	double max_psnr = 0;
	double vp_psnr;
	for(width=1; width <= 2; width ++){
		tileVer[width-1] = proposed_3(index, width);
			int tmp_vp[2];
			tmp_vp[0] = est_vp[index][0] + estError[index][0];
			tmp_vp[1] = est_vp[index][1] + estError[index][1];
			vp_psnr = estimateViewportPSNR(index, tileVer[width-1], tmp_vp);
		if(vp_psnr > max_psnr){
			selectTileVer = tileVer[width-1];
			max_psnr = vp_psnr;
			decide_width[index] = width;
		}
	}
	return selectTileVer;
}
int* AdaptationLogic::extendVisibleMask(int* vmask, int ext_width){
		int** ext_vmask = iniTwoDimenArrayInt(metadata->vp->No_tile_v, metadata->vp->No_tile_h);
		int* ret = new int[metadata->vp->No_tile];
		int i,j;
		// convert 'vmask' to rectangle
		for(i=0; i < metadata->vp->No_tile_v; i++){
			for(j=0; j < metadata->vp->No_tile_h; j++){
				ext_vmask[i][j] = vmask[i*metadata->vp->No_tile_h + j];
//				printf("%d ", ext_vmask[i][j]);
			}
//			printf("\n");
		}
//		printf("\n");
//		int** ext_vmask_tmp = iniTwoDimenArrayInt(metadata->vp->No_tile_v,metadata->vp->No_tile_h);
		// extend visible mask in four directions by a width of 'wxt_width'
		if(ext_width == 0){
			ret = vmask;
		}else{
			if(ext_width >= 4){
				for(i=0; i < metadata->vp->No_tile; i++)
					ret[i] = 1;
			}else{
				int** A = iniTwoDimenArrayInt(3, metadata->vp->No_tile_h);
//				printf("-------------A\n");
//				for(i=0; i < 3; i++){
//					for(j=0; j < metadata->vp->No_tile_h; j++){
//						printf("%d ", A[i][j]);
//					}
//					printf("\n");
//				}
//				printf("\n");
//				// compute visible area boundaries:
				// vertically
				int bcol = -1; // begin column
				int ecol = -1; // end column
				for(i=0; i < metadata->vp->No_tile_h; i++){
					for(j=0; j < metadata->vp->No_tile_v; j++){
						//					System.out.printf("(%d, %d)\n", i, j);
						if(A[0][i] == 0 && ext_vmask[j][i] == 1){
							//						System.out.printf("#1(%d, %d)\n", j, i);
							A[0][i] = 1;
							A[1][i] = j;
						}
						if(A[0][i] == 1 && ext_vmask[j][i] == 0){
							//						System.out.printf("#2(%d, %d)\n", j, i);
							A[2][i] = j-1;
							break;
						}
					}
				}
				// horizontally
				for(i=0; i < metadata->vp->No_tile_h; i++){
					if(bcol == -1 && A[0][i] == 1){
						bcol = i;
						ecol = i;
						continue;
					}
					if(bcol != -1 && A[0][i] == 1 && A[0][i-1] == 1)
						ecol = i;
					if(bcol != -1 && A[0][i] == 1 && A[0][i-1] != 1){
						bcol = i;
						break;
					}
				}
				// working here: 
//				printf("%d->%d %d->%d\n", bcol, ecol, A[1][bcol], A[2][bcol]);
				// expand to left
				int pcol; // previous column
				int ncol; // next column
				if(bcol > ext_width){
					pcol = bcol - ext_width;
				}else{
					pcol = metadata->vp->No_tile_h - (ext_width - bcol);
				}
//				printf("pcol:%d\n", pcol);
				if(pcol < bcol){
					for(i=A[1][bcol]; i <= A[2][bcol]; i++)
						for(j=pcol; j <= bcol-1; j++)
							ext_vmask[i][j] = 1;
				}else{
					for(i=A[1][bcol]; i <= A[2][bcol]; i++)
						for(j=0; j < metadata->vp->No_tile_h; j++)
							if(j <= bcol-1 || j >= pcol)
								ext_vmask[i][j] = 1;
				}
				// expand to right
				if(ecol + ext_width < metadata->vp->No_tile_h){
					ncol = ecol + ext_width;
				}else{
					ncol = ecol + ext_width - metadata->vp->No_tile_h;
				}
//				printf("ncol:%d\n", ncol);
				if(ncol > ecol){
					for(i=A[1][ecol]; i <= A[2][ecol]; i++)
						for(j=ecol+1; j <= ncol; j++)
							ext_vmask[i][j] = 1;
				}else{
					for(i=A[1][ecol]; i <= A[2][ecol]; i++)
						for(j=0; j < metadata->vp->No_tile_h; j++)
							if(j <= ncol || j >= ecol+1)
								ext_vmask[i][j] = 1;
				}
				// up and down
				int trow = A[1][bcol] - ext_width; // top row
				if(trow < 0) trow = 0;
				int brow = A[2][bcol] + ext_width; // bottom row
				if(brow >= metadata->vp->No_tile_v)
					brow = metadata->vp->No_tile_v - 1;
				int tmp_3 = A[1][bcol] - 1;
				int tmp_4 = A[2][bcol] + 1;
//				System.out.printf("trow:%d tmp_3:%d\nbrow:%d tmp_4:%d\n pcol:%d ncol:%d\n", trow, tmp_3, brow, tmp_4, pcol, ncol);
				//
				if(pcol < ncol){
					for(i=0; i < metadata->vp->No_tile_v; i++){
						//					System.out.printf("********i:%d t_row: %d\n", i, trow);
						if((i >= trow && i <= tmp_3) || (i >= tmp_4 && i <= brow)){
							for(j=pcol; j <= ncol; j++){
								//							System.out.printf("i:%d j:%d\n", i, j);
								ext_vmask[i][j] = 1;
							}
						}
					}
				}else{
					//				System.out.printf("case #2!!\n");
					for(i=0; i < metadata->vp->No_tile_v; i++){
						if ((i >= trow && i <= tmp_3) || (i >= tmp_4 && i <= brow)){
							for(j=0; j < metadata->vp->No_tile_h; j++){
								if(j >= pcol || j <= ncol){
									//								System.out.printf("i:%d j:%d\n", i, j);
									ext_vmask[i][j] = 1;
								}
							}
						}
					}
				}
				// convert 'rectangle' to 1-dimension array
				for(i=0; i < metadata->vp->No_tile_h; i++){
					for(j=0; j < metadata->vp->No_tile_v; j++){
						ret[i*metadata->vp->No_tile_h + j]= ext_vmask[i][j];
					}
				}
			}
		}
		return ret;
}
int* AdaptationLogic::directRoIBG(int index, int* vmask){

		// determine visible tiles at current viewport
		int* tileVer = new int[metadata->vp->No_tile];
		double allocatedBandwidth = 0;
		double remainBandwidth = est_thrp[index];
		double total_br;
		int i,j;
		bool FLAG = false;
		//System.out.printf("-est_bw:%.2f\n", est_bw);
		// decide tile's versions
		// assign minimum quality for all tiles
		for(i=0; i < metadata->vp->No_tile; i++){
				tileVer[i] = 0;
				allocatedBandwidth += metadata->TILE_BR[index][i][tileVer[i]];
		}
		remainBandwidth -= allocatedBandwidth;
//		printf("bw:%.2f\tallocated:%.2f\tremain:%.2f\n", est_thrp[index], allocatedBandwidth, remainBandwidth);
		//System.out.printf("-allocatedBandwidth:%.2f\n-remainBW:%.2f\n",allocatedBandwidth, remainBandwidth);
		if(remainBandwidth <= 0){
			return tileVer;
		}
		// find the next higher version for visible tiles
		for(i=1; i < metadata->NO_VER; i++){ // for each quality version
			// compute total bitrate of the visible tiles
			total_br = 0;
			for(j=0; j < metadata->vp->No_tile; j++){
				if(vmask[j] > 0)
					total_br += (metadata->TILE_BR[index][j][i] - metadata->TILE_BR[index][j][tileVer[j]]);
			}
			if(total_br > remainBandwidth) break;
		}
		// update the selected version for visible tiles 
		for(j=0; j < metadata->vp->No_tile; j++){
			if(vmask[j] > 0){
				remainBandwidth -= (metadata->TILE_BR[index][j][i-1] - metadata->TILE_BR[index][j][tileVer[j]]);
				tileVer[j] = i-1;
			}
		}
//		printf("version:%d\tremain:%.2f\n", i, remainBandwidth);
		//System.out.printf("-remainBW:%.2f tileVer: %d\n", remainBandwidth, i-1);
		// allocate remain bw if any to visible tiles
		FLAG = false;
		while(remainBandwidth > 0 && !FLAG){
			FLAG = true;
			for(i=0; i < metadata->vp->No_tile; i++){
				if(vmask[i] > 0 && remainBandwidth > 0){
					if(tileVer[i] < metadata->NO_VER - 1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBandwidth){
						remainBandwidth -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
						tileVer[i] ++;
						FLAG = false;
					}
				}
			}
		}
		//System.out.printf("-remainBW:%.2f\n", remainBandwidth);
		// allocate remain bw if any to invisible tiles
		FLAG = false;
		while(remainBandwidth > 0 && !FLAG){
			FLAG = true;
			for(i=0; i < metadata->vp->No_tile; i++){
				if(vmask[i] == 0 && remainBandwidth > 0){
					if(tileVer[i] == 0){
						if(tileVer[i] < metadata->NO_VER -1 && (metadata->TILE_BR[index][i][tileVer[i]+2] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBandwidth){
							remainBandwidth -= metadata->TILE_BR[index][i][tileVer[i]+2] - metadata->TILE_BR[index][i][tileVer[i]];
							tileVer[i] += 2;
							FLAG = false;
						}
					}else
					{
						if(tileVer[i] < metadata->NO_VER -1 && (metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]]) < remainBandwidth){
							remainBandwidth -= metadata->TILE_BR[index][i][tileVer[i]+1] - metadata->TILE_BR[index][i][tileVer[i]];
							tileVer[i] ++;
							FLAG = false;
						}
					}
				}
			}
		}
		//System.out.printf("-remainBW:%.2f\n", remainBandwidth);
		return tileVer;
	
}
int* AdaptationLogic::ISM_ext_v3(int index){
  int NUM_PART = 9;
  int i[100], j, ii;
  int EXT_W = 2;
  double remainBW,remainBW2, max_vpPSNR = 0, vpPSNR, max_remainBW;
  double tmpsum[100];
  int tmp_vp[2];
  int N = metadata->vp->No_tile;
  int NUM_VER = metadata->NO_VER;
  int* tileVer = (int*) malloc(N * sizeof(int));
  int* ret = (int*) malloc(N * sizeof(int));;
  est_vp[index] = est_vp_frame[INTERVAL * index];
  printf("(%d, %d)\n", est_vp[index][0], est_vp[index][1]);
  int* vmask = getVisibleTile(est_vp[index]);
  if(vmask == NULL){
    printf("(%d, %d)\n", est_vp[index][0], est_vp[index][1]);
    exit(1);
  }
  int* adapt_area = extVmask_ext(vmask, EXT_W, NUM_PART);
  int count = 0, count2=0;
  double sum;
  int WITH_BREAK = 1;
  if(index < 2){
    for(j=0; j < N; j++)
      ret[j] = 0;
    return ret;
  }
  //
  /*
  printf("#[ISM_ext]: index=%d est_thrp=%.2f(kbps)\n", index, est_thrp[index]);
  for(j=0; j < N; j++){
    printf("%d ", adapt_area[j]);
    if((j+1) % 8 == 0) printf("\n");
  }
  */
  // select the lowest version for background tiles
  remainBW = est_thrp[index];
  for(j=0; j < N; j++){
    if(adapt_area[j] == 0){
      tileVer[j] = 0;
      remainBW -= metadata->TILE_BR[index][j][0];
    }
  }
  // loop to find optimal versions of tiles in other areas
  for(i[0] = 0; i[0] < NUM_VER; i[0]++){//1
    //
    tmpsum[0] = 0;      
    for(j=0; j < N; j++){
      if(adapt_area[j] == 1){
	tmpsum[0] += metadata->TILE_BR[index][j][i[0]];
      }
    }
    if(tmpsum[0] >= remainBW) break;
    //
    for(i[1] = 0; i[1] < NUM_VER; i[1]++){//2
      //
      tmpsum[1] = 0;
      for(j=0; j < N; j++){
	if(adapt_area[j] == 2){
	  tmpsum[1] += metadata->TILE_BR[index][j][i[1]];
	}
      }
      if(tmpsum[0] + tmpsum[1] >= remainBW) break;
      if(i[1] > i[0]) break;
      //
      for(i[2] = 0; i[2] < NUM_VER; i[2]++){//3
	//
	tmpsum[2] = 0;
	for(j=0; j < N; j++){
	  if(adapt_area[j] == 3){
	    tmpsum[2] += metadata->TILE_BR[index][j][i[2]];
	  }
	}
	if(tmpsum[0] + tmpsum[1] + tmpsum[2] >= remainBW) break;
	if(i[2] > i[0]) break;
	//
	for(i[3] = 0; i[3] < NUM_VER; i[3]++){//4
	  //
	  tmpsum[3] = 0;
	  for(j=0; j < N; j++){
	    if(adapt_area[j] == 4){
	      tmpsum[3] += metadata->TILE_BR[index][j][i[3]];
	    }
	  }
	  if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] >= remainBW) break;
	  if(i[3] > i[0]) break;
	  //
	  for(i[4] = 0; i[4] < NUM_VER; i[4]++){//5
	    //
	    tmpsum[4] = 0;
	    for(j=0; j < N; j++){
	      if(adapt_area[j] == 5){
		tmpsum[4] += metadata->TILE_BR[index][j][i[4]];
	      }
	    }
	    if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] >= remainBW) break;
	    if(i[4] > i[0]) break;
	    //
	    for(i[5] = 0; i[5] < NUM_VER; i[5]++){//6
	      //
	      tmpsum[5] = 0;
	      for(j=0; j < N; j++){
		if(adapt_area[j] == 6){
		  tmpsum[5] += metadata->TILE_BR[index][j][i[5]];
		}
	      }
	      if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] >= remainBW) break;
	      if(i[5] > i[0] || i[5] > i[1] || i[5] > i[2] || i[5] > i[3] || i[5] > i[4]) break;
	      //if(i[5] > i[0] || i[5] > i[1]) break;
	      //
	      for(i[6] = 0; i[6] < NUM_VER; i[6]++){//7
		//
		tmpsum[6] = 0;
		for(j=0; j < N; j++){
		  if(adapt_area[j] == 7){
		    tmpsum[6] += metadata->TILE_BR[index][j][i[6]];
		  }
		}
		if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] + tmpsum[6] >= remainBW) break;
		if(i[6] > i[0] || i[6] > i[1] || i[6] > i[2] || i[6] > i[3] || i[6] > i[4]) break;
		//if(i[6] > i[0] || i[6] > i[2]) break;
		//
		for(i[7] = 0; i[7] < NUM_VER; i[7]++){//8
		  //
		  tmpsum[7] = 0;
		  for(j=0; j < N; j++){
		    if(adapt_area[j] == 8){
		      tmpsum[7] += metadata->TILE_BR[index][j][i[7]];
		    }
		  }
		  if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] + tmpsum[6] + tmpsum[7] >= remainBW) break;
		  if(i[7] > i[0] || i[7] > i[1] || i[7] > i[2] || i[7] > i[3] || i[7] > i[4]) break;
		  //if(i[7] > i[0] || i[7] > i[3]) break;
		  //
		  for(i[8] = 0; i[8] < NUM_VER; i[8]++){
		    //
		    tmpsum[8] = 0;
		    for(j=0; j < N; j++){
		      if(adapt_area[j] == 9){
			tmpsum[8] += metadata->TILE_BR[index][j][i[8]];
		      }
		    }
		    if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] + tmpsum[6] + tmpsum[7] + tmpsum[8] >= remainBW) break;
		    if(i[8] > i[0] || i[8] > i[1] || i[8] > i[2] || i[8] > i[3] || i[8] > i[4]) break;
		    //if(i[8] > i[0] || i[8] > i[4]) break;
		    count2++;
		    // assign version for each area
		    sum = 0;
		    for(j=0; j < N; j++){
		      if(adapt_area[j] > 0){
			tileVer[j] = i[adapt_area[j]-1];
			sum += metadata->TILE_BR[index][j][tileVer[j]];
		      }
		    }
		    //
		    if(sum <= remainBW){
		      // using remaining bandwidth (if any) to increase tile versions
		      remainBW2 = remainBW - sum;
		      int BREAK_FLAG;
		      int area_id;
		      while(remainBW2 > 0){
			BREAK_FLAG = 1;
			//for(area_id = 5; area_id >= 2; area_id --){
			for(ii=N-1; ii >= 0; ii--){
			  if(adapt_area[ii] == 1  && tileVer[ii] < NUM_VER -1 && (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]) < remainBW2){
			    remainBW2 = remainBW2 - (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]);
			    tileVer[ii] += 1;
			    BREAK_FLAG = 0;
			  }
			}
			//}
			if(BREAK_FLAG == 1)
			  break;
		      }
		      while(remainBW2 > 0){
			BREAK_FLAG = 1;
			//for(area_id = 5; area_id >= 2; area_id --){
			  for(ii=0; ii < N; ii++){
			    if(adapt_area[ii] >=2 && adapt_area[ii] <=5 && tileVer[ii] < NUM_VER -1 && (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]) < remainBW2){
			      remainBW2 = remainBW2 - (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]);
			      tileVer[ii] += 1;
			      BREAK_FLAG = 0;
			    }
			  }
			  //}
			if(BREAK_FLAG == 1)
			  break;
		      }
		      while(remainBW2 > 0){
			BREAK_FLAG = 1;
			for(area_id = 6; area_id <= 9; area_id ++){
			  for(ii=0; ii < N; ii++){
			    if(adapt_area[ii] == area_id && tileVer[ii] < NUM_VER -1 && (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]) < remainBW2){
			      remainBW2 = remainBW2 - (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]);
			      tileVer[ii] += 1;
			      BREAK_FLAG = 0;
			    }
			  }
			}
			if(BREAK_FLAG == 1)
			  break;
		      }
		      // compute viewport PSNR
		      vpPSNR = 0;
		      for(j=0; j < INTERVAL; j++){
            tmp_vp[0] = est_vp_frame[index * INTERVAL + j][0] + estError[index][0] * j * 1.0 / (INTERVAL -1);
            tmp_vp[1] = est_vp_frame[index * INTERVAL + j][1] + estError[index][1] * j * 1.0 / (INTERVAL -1);
            if(tmp_vp[0] > 180)
              tmp_vp[0] -= 360;
            if(tmp_vp[0] <= -180)
              tmp_vp[0] += 360;
            if(tmp_vp[1] > 90)
              tmp_vp[1] = 90;
            if(tmp_vp[1] < -90)
              tmp_vp[1] = -90;
            vpPSNR += 1.0 / INTERVAL * estimateViewportPSNR(index, tileVer, tmp_vp);
          }
		      if(vpPSNR > max_vpPSNR){
			//for(j=0; j < N; j++)
			//  ret[j] = tileVer[j];
			std::copy (tileVer, tileVer + N, ret);
			max_vpPSNR = vpPSNR;
			max_remainBW = remainBW2;
			if(index==3 && max_vpPSNR >= 42.39){
			  double total_br = 0;
			  //printf("[DUC]: remainBW=%.2f\n", remainBW2);
			  for(j=0; j < N; j++){
			    total_br += metadata->TILE_BR[index][j][ret[j]];
			    //printf("%d ", tileVer[j]);
			    //if((j+1) % 8 == 0) printf("\n");
			  }
			  printf("[DUC]: total_br=%.2f remainBW=%.2f\n", total_br, est_thrp[index] - total_br);
			}
		      }
		      count++;
		    }
		    //
		  }//9
		}//8
	      }//7
	    }//6
	  }//5
	}//4
      }//3
    }//2
  }//1
  //printf("# select_version: index=%d vpPSNR=%.2f count=%d count2=%d remainBW=%.2f\n",index, max_vpPSNR, count, count2, max_remainBW);
  double tmp_bitrate = 0;
  for(ii=0; ii < N; ii++){
    tmp_bitrate += metadata->TILE_BR[index][ii][ret[ii]];
    //printf("%d ", ret[ii]);
    //if((ii+1) % 8 == 0) printf("\n");
  }
  //printf("# bitrate=%.2f remainBW=%.2f\n", tmp_bitrate, est_thrp[index] - tmp_bitrate);
  return ret;
}
int* AdaptationLogic::ISM_ext_v4(int index){
  int NUM_PART = 9;
  int i[100], j, ii;
  int EXT_W = 2;
  double remainBW,remainBW2, max_vpPSNR = 0, vpPSNR, max_remainBW;
  double tmpsum[100];
  int N = metadata->vp->No_tile;
  int NUM_VER = metadata->NO_VER;
  int* tileVer = (int*) malloc(N * sizeof(int));
  int* ret = (int*) malloc(N * sizeof(int));;
  est_vp[index] = est_vp_frame[INTERVAL * index];
  printf("(%d, %d)\n", est_vp[index][0], est_vp[index][1]);
  int* vmask = getVisibleTile(est_vp[index]);
  if(vmask == NULL){
    printf("(%d, %d)\n", est_vp[index][0], est_vp[index][1]);
    exit(1);
  }
  int* adapt_area = extVmask_ext(vmask, EXT_W, NUM_PART);
  int count = 0, count2=0;
  double sum;
  int WITH_BREAK = 1;
  if(index < 2){
    for(j=0; j < N; j++)
      ret[j] = 0;
    return ret;
  }
  //
  printf("#[ISM_ext]: index=%d est_thrp=%.2f(kbps)\n", index, est_thrp[index]);
  for(j=0; j < N; j++){
    printf("%d ", adapt_area[j]);
    if((j+1) % 8 == 0) printf("\n");
  }
  // select the lowest version for background tiles
  remainBW = est_thrp[index];
  for(j=0; j < N; j++){
    if(adapt_area[j] == 0){
      tileVer[j] = 0;
      remainBW -= metadata->TILE_BR[index][j][0];
    }
  }
  // loop to find optimal versions of tiles in other areas
  for(i[0] = 0; i[0] < NUM_VER; i[0]++){//1
    //
    tmpsum[0] = 0;      
    for(j=0; j < N; j++){
      if(adapt_area[j] == 1){
	tmpsum[0] += metadata->TILE_BR[index][j][i[0]];
      }
    }
    if(tmpsum[0] >= remainBW) break;
    //
    for(i[1] = 0; i[1] < NUM_VER; i[1]++){//2
      //
      tmpsum[1] = 0;
      for(j=0; j < N; j++){
	if(adapt_area[j] == 2){
	  tmpsum[1] += metadata->TILE_BR[index][j][i[1]];
	}
      }
      if(tmpsum[0] + tmpsum[1] >= remainBW) break;
      if(i[1] > i[0]) break;
      //
      for(i[2] = 0; i[2] < NUM_VER; i[2]++){//3
	//
	tmpsum[2] = 0;
	for(j=0; j < N; j++){
	  if(adapt_area[j] == 3){
	    tmpsum[2] += metadata->TILE_BR[index][j][i[2]];
	  }
	}
	if(tmpsum[0] + tmpsum[1] + tmpsum[2] >= remainBW) break;
	if(i[2] > i[0]) break;
	//
	for(i[3] = 0; i[3] < NUM_VER; i[3]++){//4
	  //
	  tmpsum[3] = 0;
	  for(j=0; j < N; j++){
	    if(adapt_area[j] == 4){
	      tmpsum[3] += metadata->TILE_BR[index][j][i[3]];
	    }
	  }
	  if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] >= remainBW) break;
	  if(i[3] > i[0]) break;
	  //
	  for(i[4] = 0; i[4] < NUM_VER; i[4]++){//5
	    //
	    tmpsum[4] = 0;
	    for(j=0; j < N; j++){
	      if(adapt_area[j] == 5){
		tmpsum[4] += metadata->TILE_BR[index][j][i[4]];
	      }
	    }
	    if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] >= remainBW) break;
	    if(i[4] > i[0]) break;
	    //
	    for(i[5] = 0; i[5] < NUM_VER; i[5]++){//6
	      //
	      tmpsum[5] = 0;
	      for(j=0; j < N; j++){
		if(adapt_area[j] == 6){
		  tmpsum[5] += metadata->TILE_BR[index][j][i[5]];
		}
	      }
	      if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] >= remainBW) break;
	      //if(i[5] > i[0] || i[5] > i[1] || i[5] > i[2] || i[5] > i[3] || i[5] > i[4]) break;
	      if(i[5] > i[0] || i[5] > i[1]) break;
	      //
	      for(i[6] = 0; i[6] < NUM_VER; i[6]++){//7
		//
		tmpsum[6] = 0;
		for(j=0; j < N; j++){
		  if(adapt_area[j] == 7){
		    tmpsum[6] += metadata->TILE_BR[index][j][i[6]];
		  }
		}
		if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] + tmpsum[6] >= remainBW) break;
		//if(i[6] > i[0] || i[6] > i[1] || i[6] > i[2] || i[6] > i[3] || i[6] > i[4]) break;
		if(i[6] > i[0] || i[6] > i[2]) break;
		//
		for(i[7] = 0; i[7] < NUM_VER; i[7]++){//8
		  //
		  tmpsum[7] = 0;
		  for(j=0; j < N; j++){
		    if(adapt_area[j] == 8){
		      tmpsum[7] += metadata->TILE_BR[index][j][i[7]];
		    }
		  }
		  if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] + tmpsum[6] + tmpsum[7] >= remainBW) break;
		  //if(i[7] > i[0] || i[7] > i[1] || i[7] > i[2] || i[7] > i[3] || i[7] > i[4]) break;
		  if(i[7] > i[0] || i[7] > i[3]) break;
		  //
		  for(i[8] = 0; i[8] < NUM_VER; i[8]++){
		    //
		    tmpsum[8] = 0;
		    for(j=0; j < N; j++){
		      if(adapt_area[j] == 9){
			tmpsum[8] += metadata->TILE_BR[index][j][i[8]];
		      }
		    }
		    if(tmpsum[0] + tmpsum[1] + tmpsum[2] + tmpsum[3] + tmpsum[4] + tmpsum[5] + tmpsum[6] + tmpsum[7] + tmpsum[8] >= remainBW) break;
		    if(i[8] > i[0] || i[8] > i[1] || i[8] > i[2] || i[8] > i[3] || i[8] > i[4]) break;
		    //if(i[8] > i[0] || i[8] > i[4]) break;
		    count2++;
		    // assign version for each area
		    sum = 0;
		    for(j=0; j < N; j++){
		      if(adapt_area[j] > 0){
			tileVer[j] = i[adapt_area[j]-1];
			sum += metadata->TILE_BR[index][j][tileVer[j]];
		      }
		    }
		    //
		    if(sum <= remainBW){
		      // using remaining bandwidth (if any) to increase tile versions
		      remainBW2 = remainBW - sum;
		      int BREAK_FLAG;
		      int area_id;
		      while(remainBW2 > 0){
			BREAK_FLAG = 1;
			//for(area_id = 5; area_id >= 2; area_id --){
			for(ii=N-1; ii >= 0; ii--){
			  if(adapt_area[ii] == 1  && tileVer[ii] < NUM_VER -1 && (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]) < remainBW2){
			    remainBW2 = remainBW2 - (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]);
			    tileVer[ii] += 1;
			    BREAK_FLAG = 0;
			  }
			}
			//}
			if(BREAK_FLAG == 1)
			  break;
		      }
		      while(remainBW2 > 0){
			BREAK_FLAG = 1;
			//for(area_id = 5; area_id >= 2; area_id --){
			  for(ii=0; ii < N; ii++){
			    if(adapt_area[ii] >=2 && adapt_area[ii] <=5 && tileVer[ii] < NUM_VER -1 && (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]) < remainBW2){
			      remainBW2 = remainBW2 - (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]);
			      tileVer[ii] += 1;
			      BREAK_FLAG = 0;
			    }
			  }
			  //}
			if(BREAK_FLAG == 1)
			  break;
		      }
		      while(remainBW2 > 0){
			BREAK_FLAG = 1;
			for(area_id = 6; area_id <= 9; area_id ++){
			  for(ii=0; ii < N; ii++){
			    if(adapt_area[ii] == area_id && tileVer[ii] < NUM_VER -1 && (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]) < remainBW2){
			      remainBW2 = remainBW2 - (metadata->TILE_BR[index][ii][tileVer[ii] +1] - metadata->TILE_BR[index][ii][tileVer[ii]]);
			      tileVer[ii] += 1;
			      BREAK_FLAG = 0;
			    }
			  }
			}
			if(BREAK_FLAG == 1)
			  break;
		      }
		      // compute viewport PSNR
		      vpPSNR = 0;
		      for(j=0; j < INTERVAL; j++)
			vpPSNR += 1.0 / INTERVAL * estimateViewportPSNR(index, tileVer, est_vp_frame[index * INTERVAL + j]);
		      if(vpPSNR > max_vpPSNR){
			//for(j=0; j < N; j++)
			//  ret[j] = tileVer[j];
			std::copy (tileVer, tileVer + N, ret);
			max_vpPSNR = vpPSNR;
			max_remainBW = remainBW2;
			if(index==3 && max_vpPSNR >= 42.39){
			  double total_br = 0;
			  printf("[DUC]: remainBW=%.2f\n", remainBW2);
			  for(j=0; j < N; j++){
			    total_br += metadata->TILE_BR[index][j][ret[j]];
			    printf("%d ", tileVer[j]);
			    if((j+1) % 8 == 0) printf("\n");
			  }
			  printf("[DUC]: total_br=%.2f remainBW=%.2f\n", total_br, est_thrp[index] - total_br);
			}
		      }
		      count++;
		    }
		    //
		  }//9
		}//8
	      }//7
	    }//6
	  }//5
	}//4
      }//3
    }//2
  }//1
  printf("# select_version: index=%d vpPSNR=%.2f count=%d count2=%d remainBW=%.2f\n",index, max_vpPSNR, count, count2, max_remainBW);
  double tmp_bitrate = 0;
  for(ii=0; ii < N; ii++){
    tmp_bitrate += metadata->TILE_BR[index][ii][ret[ii]];
    printf("%d ", ret[ii]);
    if((ii+1) % 8 == 0) printf("\n");
  }
  printf("# bitrate=%.2f remainBW=%.2f\n", tmp_bitrate, est_thrp[index] - tmp_bitrate);
  return ret;
}

