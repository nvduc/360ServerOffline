#include "Viewport.h"
#include <stdio.h>
#include <cmath>
#define _USE_MATH_DEFINES

Viewport::Viewport(double Fh_, double Fv_, int vp_W_, int vp_H_, int erp_W_, int erp_H_, int tile_W_, int tile_H_)
{
	int i,j,tid;
	Fh = Fh_;
	Fv = Fv_;
	vp_W = vp_W_;
	vp_H = vp_H_;
	erp_W = erp_W_;
	erp_H = erp_H_;
	tile_H = tile_H_;
	tile_W = tile_W_;
	No_tile_h = erp_W_/tile_W_;
	No_tile_v = erp_H_/tile_H_;
	No_tile = No_tile_h * No_tile_v;
	LB_tile_W = new int[No_tile];
	HB_tile_W = new int[No_tile];
	LB_tile_H = new int[No_tile];
	HB_tile_H = new int[No_tile];
	for(j=0; j < No_tile_v; j++){
		for(i=0; i < No_tile_h; i++){
			tid = j * No_tile_h + i;
			LB_tile_W[tid] = i * tile_W;
			HB_tile_W[tid] = (i+1) * tile_W;
			LB_tile_H[tid] = j * tile_H;
			HB_tile_H[tid] = (j+1) * tile_H;
		}
	}
}
Viewport::~Viewport(){
}
void Viewport::calcVisibleMask(double phi, double theta, int** vmask, int** pixel){
		// printf("Check #0\n");
		int i, j, m_, n_, t;
		double u,v,x,y,z,x_,y_,z_,X,Y,Z,phi_, theta_,u_,v_;
		int m_temp;
		// printf("Check #1: %d %d\n", vp_H, vp_W);
		int **M;
		int **N;
		//
		M = new int*[vp_H];
		N = new int*[vp_H];
		for(i=0; i < vp_W; i++){
			M[i] = new int[vp_W];
			N[i] = new int[vp_W];
		}
		//	
		// printf("Check #2\n");
		// Map viewport to ERP
		for(m_=0; m_ < vp_W; m_++){
			for(n_=0; n_ < vp_H; n_++){
				// printf("%d %d\n", m_, n_);
		        u = (m_+0.5) * 2 * tan(Fh/2)/vp_W;
		        v = (n_+0.5) * 2 * tan(Fv/2)/vp_H;
		        x = u - tan(Fh/2);
		        y = -v + tan(Fv/2);
		        z = 1;
		        x_ = x/sqrt(x*x + y*y + z*z);
		        y_ = y/sqrt(x*x + y*y + z*z);
		        z_ = z/sqrt(x*x + y*y + z*z);
		        double R[3][3] = {{cos(phi + M_PI/2),-sin(phi + M_PI/2)*sin(theta),sin(phi+M_PI/2) * cos(theta)},{0, cos(theta), sin(theta)},{-sin(phi + M_PI/2),-cos(phi + M_PI/2) * sin(theta), cos(phi + M_PI/2) * cos(theta)}};
		        X = R[0][0] * x_ + R[0][1] * y_ + R[0][2] * z_;
		        Y = R[1][0] * x_ + R[1][1] * y_ + R[1][2] * z_;
		        Z = R[2][0] * x_ + R[2][1] * y_ + R[2][2] * z_;
		        phi_ = atan(-Z/X);
		        theta_ = asin(Y/sqrt(X*X + Y*Y + Z*Z));
		        if(X < 0)
		            phi_=M_PI+phi_;
		        u_ = phi_/(2*M_PI) + 0.5;
		        v_ = 0.5 - theta_ / M_PI;
		        m_temp =  (int) round(u_ * erp_W - 0.5);
		        if(m_temp > erp_W)
		            M[n_][m_] = m_temp - erp_W;
		        else
		        	M[n_][m_]  = m_temp;
		        N[n_][m_]  = (int) round(v_ * erp_H - 0.5);
			}
		}
		// compute visible tiles
				for(m_=0; m_ < vp_W; m_++){
			    	for(n_=0; n_ < vp_H; n_++){
			        	for(i=0; i < No_tile_v; i++){
			        		for(j=0; j < No_tile_h; j++){
				            	t = i * No_tile_h + j;
			        			if(N[n_][m_] < HB_tile_H[t] && N[n_][m_]>= LB_tile_H[t] && M[n_][m_] < HB_tile_W[t] && M[n_][m_] >= LB_tile_W[t]){
				               		vmask[i][j] = 1;
				                    pixel[i][j] = pixel[i][j] + 1;
				                    break; 
				            	}
			        		}
			        	}
			    	}
				}
}