#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include "AdaptationLogic.h"
#include "Viewport.h"
#include "Metadata.h"
#include "GaussianFilter.h"
#include "ViewportParam.h"
#define M_PI 3.1415
using namespace std;
ViewportParam read_viewport_info(char* );
//
int main(int argc, char* argv[]){
  if(argc != 5){
    printf("usage: ./getVTile index  phi theta version.txt\n");
    exit(1);
  }
  ViewportParam vpParam;
  AdaptationLogic *adaptLogic;
  Viewport *viewport;
  Metadata *metadata;
  //
  vpParam = read_viewport_info("config_copy/Viewport.cfg");
  viewport = new Viewport(vpParam.Fh, vpParam.Fv, vpParam.vp_W, vpParam.vp_H, vpParam.erp_W, vpParam.erp_H, vpParam.tile_W, vpParam.tile_H);
  metadata = new Metadata(viewport, vpParam.SD, vpParam.SESS_DUR, vpParam.NO_VER, vpParam.FPS, vpParam.NO_SEG, vpParam.BUFFSIZE, vpParam.NO_SEG_FULL, vpParam.head_pos_trace, vpParam.INTERVAL, vpParam.BUFFSIZE);
  metadata->speed = vpParam.speed;
  metadata->delay = vpParam.delay;
  adaptLogic = new AdaptationLogic(metadata, vpParam.INTERVAL, vpParam.METHOD, vpParam.margin);
  //
  int* vp = (int*) malloc(2 * sizeof(int));
  int row = 8;
  int col = 8;
  int index = atoi(argv[1]);
  vp[0] = atoi(argv[2]);
  vp[1] = atoi(argv[3]);
  int* vmask = adaptLogic->getVisibleTile(vp);
  FILE* fp = fopen(argv[4], "r");
  if(fp == NULL){
    printf("Cannot open file!\n");
    exit(2);
  }
  int* tileVer = (int*) malloc(row * col * sizeof(int));
  int x;
  int i=0;
  while(fscanf(fp, "%d ", &x) != EOF){
    tileVer[i++] = x;
  }
  double vppsnr = adaptLogic->estimateViewportPSNR(index, tileVer, vp);
  for(int i=0; i < row; i++){
    for(int j=0; j < col; j++){
      printf("%d ", vmask[i*row + j]);
    }
    printf("\t");
    for(int j=0; j < col; j++){
      printf("%d ", tileVer[i*row + j]);
    }
    printf("\n");
  }
  printf("(phi,theta) = (%d, %d)\t", vp[0], vp[1]);
  printf("vpPSNR=%.2f (dB)\n", vppsnr);
  return 0;
}
ViewportParam read_viewport_info(char* filename){
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
			if(key.compare("head_pos_trace") != 0)
				val = std::stod(val_str);
			cout << key << ":" << val << endl;
		}
		if(key.compare("Fh")==0) vpParam.Fh = val * M_PI / 180;
		if(key.compare("Fv")==0) vpParam.Fv= val * M_PI / 180;
		if(key.compare("vp_W")==0) vpParam.vp_W = (int) val;
		if(key.compare("vp_H")==0) vpParam.vp_H = (int) val;
		if(key.compare("erp_W")==0) vpParam.erp_W = (int) val;
		if(key.compare("erp_H")==0) vpParam.erp_H = (int) val;
		if(key.compare("tile_W")==0) vpParam.tile_W = (int) val;
		if(key.compare("tile_H")==0) vpParam.tile_H = (int) val;
		if(key.compare("SD")==0) vpParam.SD = val;
		if(key.compare("SESS_DUR")==0) vpParam.SESS_DUR = (int) val;
		if(key.compare("NO_VER")==0) vpParam.NO_VER = (int) val;
		if(key.compare("FPS")==0) vpParam.FPS = (int) val;
		if(key.compare("NO_SEG")==0) vpParam.NO_SEG = (int) val;
		if(key.compare("BUFFSIZE")==0) vpParam.BUFFSIZE = val;
		if(key.compare("NO_SEG_FULL")==0) vpParam.NO_SEG_FULL = (int) val;
		if(key.compare("INTERVAL")==0) vpParam.INTERVAL = (int) val;
		if(key.compare("METHOD")==0) vpParam.METHOD = (int) val;
		if(key.compare("margin")==0) vpParam.margin = val;
		if(key.compare("speed") == 0) vpParam.speed = val;
		if(key.compare("delay") == 0) vpParam.delay = val;
		if(key.compare("NO_FRAME") == 0) vpParam.NO_FRAME = (int) val;
		if(key.compare("NO_FRAME_ORIGIN") == 0) vpParam.NO_FRAME_ORIGIN = (int) val;
		if(key.compare("MAX_BW") == 0) vpParam.MAX_BW = val;
		if(key.compare("head_pos_trace") == 0) {
			vpParam.head_pos_trace = val_str.c_str();
			cout << vpParam.head_pos_trace << endl;
		}
	}
	vpParam.NO_SEG = vpParam.NO_FRAME_ORIGIN / vpParam.INTERVAL;
	vpParam.NO_SEG_FULL = vpParam.NO_FRAME / vpParam.INTERVAL;
	return vpParam;
}
