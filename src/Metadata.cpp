#include "Metadata.h"
#include "Viewport.h"
#include <stdio.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
using namespace std;

Metadata::Metadata(Viewport* vp_, double SD_, int SESS_DUR_, int NO_VER_, int FPS_,int NO_SEG_, double BUFFSIZE_, int NO_SEG_FULL_, const char* head_pos_data, int INTERVAL_, int BUFF_){
	int i,j;
	char filename[100];
	vp = vp_;
	SD = SD_;
	SESS_DUR = SESS_DUR_;
	NO_VER = NO_VER_;
	FPS = FPS_;
	NO_SEG = NO_SEG_;
	BUFFSIZE = BUFFSIZE_;
	NO_SEG_FULL = NO_SEG_FULL_;
	INTERVAL = INTERVAL_;
	BUFF = BUFF_;
	char const* head_trace = head_pos_data;
	// load data from file
	cout << "$$$$ Load tile info" << endl;
	loadTileInfo();
	//
	cout << "$$$$ Load head position trace: " << head_pos_data << endl;
	// sprintf(filename, "data/head_pos_variable_trace_3.txt");
//	sprintf(filename, "data/head_pos_speed_120_6s.txt");
//	sprintf(filename, "data/head_trace/cs_Duc.txt");
//	sprintf(filename, "data/head_trace/cs_Huyen.txt");
	sprintf(filename, "data/head_trace/cs_hoanganh.txt");
//	sprintf(filename, "data/head_trace/sin_speed_60_60s.txt");
//	sprintf(filename, "data/head_trace/sin_speed_60_60s_trace_2.txt");
//	sprintf(filename, "data/head_trace/sin_speed_90_60s.txt");
//	sprintf(filename, "data/head_trace/constant_speed_60_60.txt");
//	sprintf(filename, "data/head_trace/sin_speed_75_60s.txt");
//	sprintf(filename, "data/head_trace_1.txt");
//	sprintf(filename, "data/head_trace_2.txt");
//	loadHeadTrace(head_trace);
	loadHeadTrace(filename);
	//
	cout << "$$$$ Load visible mask" << endl;
	sprintf(filename, "data/visible_mask_8x8_FoV_90_full.txt");
	loadVisibleMask(filename);
	//
	cout << "$$$$ Load inputs finished !!!" << endl;
}

Metadata::~Metadata(){

}
void Metadata::import_matrix_from_txt_file(const char* filename_X, vector <double>& v, int& rows, int& cols){
	ifstream file_X;
    string line;
    // erase all current elements
    v.erase(v.begin(), v.end());
    cout << "open text file: " << filename_X << endl;
    file_X.open(filename_X);
    if (file_X.is_open())
    {
        int i=0;
        getline(file_X, line);
        
        
        cols = ReadNumbers( line, v );
        rows = 1;
        // cout << "cols:" << cols << endl;
		while(getline(file_X, line) != 0){
			ReadNumbers(line, v);
			rows ++;
		}        
        
        // for ( i=1;i<32767;i++){
        //     if ( getline(file_X, line) == 0 ) break;
        //     ReadNumbers( line, v );
            
        // }
        
        // rows=i;
        // // cout << "rows :" << rows << endl;
        // if(rows >32766) cout<< "N must be smaller than MAX_INT";
        
        file_X.close();
    }
    else{
        cout << "file open failed";
    }
    
    // cout << "v:" << endl;
    // for (int i=0;i<rows;i++){
        // for (int j=0;j<cols;j++){
            // cout << v[i*cols+j] << "\t" ;
        // }
        // cout << endl;
    // }
}
int Metadata::ReadNumbers( const string & s, vector <double> & v ){
	istringstream is( s );
    double n;
    while( is >> n ) {
        v.push_back( n );
    }
    return v.size();
}
void Metadata::loadTileInfo(){
	int tid = 0;
	int i,j,k;
	int seg_id;
	cout << NO_SEG << " "<<vp->No_tile<<" "<<NO_VER<<endl;
	// initiate variables
	TILE_BR = new double**[NO_SEG_FULL];
	TILE_MSE = new double**[NO_SEG_FULL];
	TILE_PSNR = new double**[NO_SEG_FULL];
	TILE_SIZE = new double**[NO_SEG_FULL];
	Uti = new double**[NO_SEG_FULL];
	Cost = new double**[NO_SEG_FULL];
	// DASH-related infommation
	DASH_BR = new double*[NO_SEG_FULL];
	DASH_MSE = new double*[NO_SEG_FULL];
	DASH_PSNR = new double*[NO_SEG_FULL];
	for(i=0; i < NO_SEG_FULL; i++){
		TILE_BR[i] = new double*[vp->No_tile];
		TILE_MSE[i] = new double*[vp->No_tile];
		TILE_PSNR[i] = new double*[vp->No_tile];
		TILE_SIZE[i] = new double*[vp->No_tile];
		Uti[i] = new double*[vp->No_tile];
		Cost[i] = new double*[vp->No_tile];
		for(j=0; j < vp->No_tile; j++){
			TILE_BR[i][j] = new double[NO_VER];
			TILE_MSE[i][j] = new double[NO_VER];
			TILE_PSNR[i][j] = new double[NO_VER];
			TILE_SIZE[i][j] = new double[NO_VER];
			Uti[i][j] = new double[NO_VER];
			Cost[i][j] = new double[NO_VER];
		}
	}
	VER_BR = new double*[NO_SEG_FULL];
	for(j=0; j < NO_SEG_FULL; j++){
		VER_BR[j] = new double[NO_VER];
		DASH_BR[j] = new double[NO_VER];
		DASH_MSE[j] = new double[NO_VER];
		DASH_PSNR[j] = new double[NO_VER];
	}
	//
//	char tile_info_path[] = "data/tile_info/";
//	char tile_info_path[] = "data/tile_info_cs_16x16_300fr/gop=32/";
//	char tile_info_path[200] = "data/tile_info_cs_16x16_300fr_9_ver/gop=32/";
	// char tile_info_path[200] = "data/tile_info_cs_8x8_300fr_9_ver_new/gop=32/";
	// char tile_info_path[200] = "data/tile_info_cs_8x8_300fr_1s/gop=4/";
	// char tile_info_path[200] = "data/tile_info_cs_8x8_300fr_1s/gop=4/";
	// char tile_info_path[200] = "data/tile_info_yakitori_erp_8x8_9_ver_900fr/";
	char tile_info_path[200];
	//	sprintf(tile_info_path, "data/tile_info_cs_8x8_1792fr_7_ver/%dframe/", INTERVAL);
	sprintf(tile_info_path, "data/Yakitori_900fr_7_ver/tile_8x8/%dframe/", INTERVAL);
	printf("#[DUC]: Loading tile info from file\n");
	string s;
	char buff[100];
	vector <double> v;
	int rows;
	int cols;
	// Load tiles' info
	for(tid=0; tid < vp->No_tile; tid++){
		sprintf(buff, "%stile_%d.txt", tile_info_path, tid);
		s = buff;
		cout << s << endl;
		import_matrix_from_txt_file(buff, v, rows, cols);
		for(i=0; i < NO_SEG; i++){
			for(j=0; j < NO_VER; j++){
				// cout << i << " " << j << " " << tid <<endl;
				TILE_BR[i][tid][j] = v[i * cols + 3*j];
				TILE_PSNR[i][tid][j] = v[i * cols + 3*j + 1];
				TILE_MSE[i][tid][j] = v[i * cols + 3*j + 2];
				TILE_SIZE[i][tid][j] = TILE_BR[i][tid][j] * SD * INTERVAL;
				VER_BR[i][j] += TILE_BR[i][tid][j];
				//
				if(j==0){
					Uti[i][tid][j] = TILE_MSE[i][tid][j] - 65025;
					Cost[i][tid][j] = TILE_BR[i][tid][j];
				}else{
					Uti[i][tid][j] = TILE_MSE[i][tid][j-1] - TILE_MSE[i][tid][j];
					Cost[i][tid][j] = TILE_BR[i][tid][j] - TILE_BR[i][tid][j-1];
				}
			}
		}

		// repeat video sequence
		cout << "NO_SEG_FULL: " << NO_SEG_FULL << " NO_SEG:" << NO_SEG <<endl;
		if(NO_SEG < NO_SEG_FULL){
			for(i=NO_SEG; i < NO_SEG_FULL; i++){
				// cout << "i:" << i << endl;
				for(k=1; k <= NO_VER; k++){
					//System.out.printf("k=%d\n", k);
					TILE_BR[i][tid][k-1] = TILE_BR[i%NO_SEG][tid][k-1];
					TILE_PSNR[i][tid][k-1] = TILE_PSNR[i%NO_SEG][tid][k-1];
					TILE_MSE[i][tid][k-1] = TILE_MSE[i%NO_SEG][tid][k-1];
					TILE_SIZE[i][tid][k-1] = TILE_SIZE[i%NO_SEG][tid][k-1];
					// cout << TILE_BR[i][tid][k-1] << " " << TILE_PSNR[i][tid][k-1] << " " << TILE_MSE[i][tid][k-1] << endl; 
					VER_BR[i][k-1] = VER_BR[i%NO_SEG][k-1]; 
					//
					Uti[i][tid][k-1] = Uti[i%NO_SEG][tid][k-1];
					Cost[i][tid][k-1] = Cost[i%NO_SEG][tid][k-1];
				}
			}
		}
	}
	// // load DASH segments info
	sprintf(buff,"data/Yakitori_900fr_7_ver/non-tile/DASH_seg_%dframe.txt", INTERVAL);
	import_matrix_from_txt_file(buff, v, rows, cols);
	for(i=0; i < NO_SEG; i++){
		for(j=0; j < NO_VER; j++){
			DASH_BR[i][j] = v[i * cols + 3*j];
			DASH_PSNR[i][j] = v[i * cols + 3*j + 1];
			DASH_MSE[i][j] = v[i * cols + 3*j + 2];
		}
	}
	printf("DASH_BR[0][0]=%.2f\tDASH_BR[5][4]=%.2f\n", DASH_BR[0][0], DASH_BR[5][4]);
	// repeat until the end of the streaming session
	if(NO_SEG < NO_SEG_FULL){
		for(i=NO_SEG; i < NO_SEG_FULL; i++){
			for(j=0; j < NO_VER; j++){
				DASH_BR[i][j] = DASH_BR[i%NO_SEG][j];
				DASH_PSNR[i][j] = DASH_PSNR[i%NO_SEG][j];
				DASH_MSE[i][j] = DASH_MSE[i%NO_SEG][j];
			}
		}
	}
	// compute and record version PSNR
	FILE *f_psnr = NULL;
	FILE *f_br = NULL;
	VER_PSNR = new double*[NO_SEG];
	for(j=0; j < NO_SEG; j++){
		VER_PSNR[j] = new double[NO_VER];
	}
	for(i=0; i < NO_SEG; i++){
		sprintf(buff, "data/ver_info/seg_%d_psnr.txt", i);
		f_psnr = fopen(buff,"w");
		sprintf(buff, "data/ver_info/seg_%d_bitrate.txt", i);
		f_br = fopen(buff,"w");
		for(k=0; k < NO_VER; k++){
			fprintf(f_psnr, "-------Ver #%d:\n", k);
			fprintf(f_br, "-------Ver #%d:\n", k);
			VER_PSNR[i][k] = 0;
			for(tid=0; tid < vp->No_tile; tid++){
				VER_PSNR[i][k] += TILE_PSNR[i][tid][k] / vp->No_tile;
				fprintf(f_psnr, "%.2f\t", TILE_PSNR[i][tid][k]);
				fprintf(f_br, "%.2f\t", TILE_BR[i][tid][k]);
				if((tid + 1) % vp->No_tile_h == 0){
					fprintf(f_psnr, "\n");
					fprintf(f_br, "\n");
				}
			}
		}
		fclose(f_psnr);
		fclose(f_br);
	}
	// compute tile bitrate per version
	for(k=0; k < NO_VER; k++){
		double ver_br = 0;
		for(tid = 0; tid < vp->No_tile; tid ++){
			for(i=0; i < NO_SEG; i++){
				ver_br += TILE_BR[i][tid][k];
			}
		}
		printf("ver #%d: %.2f\n", k, ver_br/(vp->No_tile * NO_SEG));
	}
	printf("TILE_BR[1][1][1]=%.2f\n", TILE_BR[1][1][1]);
}
void Metadata::loadHeadTrace(const char* filename){
	cout << "Opening head trace: " << filename << endl;
	int i,j,k;
	vector <double> v;
	int rows;
	int cols;
	import_matrix_from_txt_file(filename, v, rows, cols);
	NO_HEAD_SAMPLE = rows;
	head_pos = new int*[rows];
	for(i=0; i < rows; i++)
		head_pos[i] = new int[cols];
	for(i=0; i < rows; i++){
		for(j=0; j < cols; j++){
			head_pos[i][j] = v[i * cols + j];
		}
	}
}
void Metadata::loadVisibleMask(char* filename){
	int i,j,k;
	vector <double> v;
	int rows;
	int cols;
	int phi_num = 360;
	int theta_num = 181;
	import_matrix_from_txt_file(filename, v, rows, cols);
	NO_VMASK_SAMPLE = rows;
	vmask = new int**[phi_num];
	pixel = new int**[phi_num];
	for(i=0; i < phi_num; i++){
		vmask[i] = new int*[theta_num];
		pixel[i] = new int*[theta_num];
		for(j=0; j < theta_num; j++){
			vmask[i][j] = new int[vp->No_tile];
			pixel[i][j] = new int[vp->No_tile];
		}
	}
	for(i=0; i < NO_VMASK_SAMPLE; i++){
		printf("%d %d\n", int(v[i*cols]), int(v[i*cols + 1]) + 90);
		for(j=0; j < vp->No_tile; j++){
			pixel[int(v[i*cols])][int(v[i*cols + 1]) + 90][j] = v[i * cols + 2*j + 3];
			if(pixel[int(v[i*cols])][int(v[i*cols + 1]) + 90][j] > 0)
				vmask[int(v[i*cols])][int(v[i*cols + 1]) + 90][j] = 1;
			else
				vmask[int(v[i*cols])][int(v[i*cols + 1]) + 90][j] = 0;
		}
	}
}
