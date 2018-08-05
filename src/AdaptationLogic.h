#ifndef AL_H
#define AL_H
#include "Metadata.h"
#include "Viewport.h"
class AdaptationLogic
{
public:
  int VP_EST_METHOD; //1: linear regression; 2: last position
  Metadata *metadata;
  double margin; // safety margin
  int METHOD;
  int INTERVAL; // adaptation interval
  double alpha_est_error;
  //
  int* fr_size;
  int* fr_play_time;
  int* fr_stall_time;
  double* fr_inter_time;
  double* fr_rcv_time;
  int* frameviewport;
  double* framepsnr;
  //
  int* seg_play_time;
  int* seg_stall_time;
  double* decide_time;
  //
  double* seg_end_time;
  double* seg_start_time;
  double* seg_thrp;
  double* est_thrp;
  double* est_thrp_act;
  int** cur_vp;
  int** est_vp;
  double* seg_bitrate;
  int* real_vp_first;
  int* real_vp_last;
  double** tileSize;
  int** tileVer;
  double* RTT;
  double* dataThrp;
  double* est_vp_psnr_first;
  double* est_vp_psnr_last;
  int* est_vp_last;
  int* est_error;
  //
  int* erpSize;
  //
  double** estError;
  int currEstError;
  int* decide_width;
  double** speed;
  //
  int* selectDASHVer;
  //
  int* select_method;
  // frames' estimated viewport positions
  int** est_vp_frame;
  int** frame_vp; // actual viewport position at each frame
  // processing time
  int* calcTime;
  AdaptationLogic(Metadata *metadata, int INTERVAL, int METHOD, double margin);
  ~AdaptationLogic();
  double* getNextSegment(int index);
  int getNextSegmentDASH(int index);
  int* getVisibleTile(int* cur_vp);
  int* getVisibleTilePixel(int* est_vp);
  double getSegBitrate(int index, int* tileVer);
  void estimator(int index);
  int calcViewport(int** vp_trace, double t_now);
  double* getTileSize(int index, int* tileVer);
  void updateFramePlayTime(int d, int index);
  double estimateViewportPSNR(int index, int* tileVer, int* vp);	//
  int* extVMask(int* vmask, int ext_width); /* extend the viewport by 'ext_width' tiles and divide into 'ext_width + 1' areas*/
  int* extVmask_ext(int* vmask, int ext_width, int num_part);// extend the viewport by 'ext_width' tiles and divide the adapt_area into 'num_part' areas.
  int* extVMask_LR(int* vmask, int ext_width);
  int* extVMask_L(int* vmask, int ext_width);
  int* extVMask_R(int* vmask, int ext_width);

  int** iniTwoDimenArrayInt(int rows, int cols);
  double** iniTwoDimenArrayDouble(int rows, int cols);
  //
  template <typename T>
    std::vector<int> sort_index(std::vector<T> const& values);
  int* BellLab(int index);
  int* ISM(int index, int ext_width);
  int* ISM(int index);
  int* ISM_error(int index, int ext_width);
  int* ISM_error(int index);
  int* ISM_ext(int index);
  int* ISM_ext_v3(int index);
  int* ISM_ext_v4(int index);
  int* EXT_ALL(int index, int ext_width);
  int* EXT_LEFT_RIGHT(int index, int ext_width);
  int* EXT_LEFT(int index, int ext_width);
  int* EXT_RIGHT(int index, int ext_width);
  int* Proposed(int index);
  int* decideTileVer_ROI(int index, int* adapt_area, double est_thrp);
  int* ROI_first(int index);
  int* Proposed_v2(int index);
  int* Proposed_v3(int index);
  int* Proposed_v4(int index);
  /*ISM-last-implementation*/
  int* proposed_3(int index);
  int* proposed_3(int index, int ext_width);
  int* extendVisibleMask(int* vmask, int ext_width);
  int* directRoIBG(int index, int* vmask);
  //
  int* decideTileVer_EXT(int index, int* adapt_area, double est_thrp);
  int* selectTileVersion(int index, int* tileVer, int* adapt_area, int layer, double* BW);
  int* selectTileVersion(int index, int* tileVer, int*adapt_area, int layer, int Q, double* BW);
  int* EQUAL(int index);

};
#endif
