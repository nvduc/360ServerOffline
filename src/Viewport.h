#ifndef VIEWPORT_H
#define VIEWPORT_H

class Viewport
{
public:
	double Fh;				// Horizontal FoV (rad)
	double Fv;				// Vertical FoV (rad)
	int vp_W;				// Viewport's width (Number of pixels)
	int vp_H;				// Viewport's height (Number of pixels)
	int erp_W;				// ERP's width
	int erp_H;				// ERP's height
	int tile_W;				// tile's width
	int tile_H;				// tile's height
	int No_tile_h;			// Number of vertical tiles
	int No_tile_v;			// Number of horizontal tiles
	int No_tile;			// Number of tiles
	int* LB_tile_W;		// tiles's horizontal low boundaries
	int* HB_tile_W;		// tiles's horizontal high boundaries
	int* LB_tile_H;		// tiles's vertical low boundaries
	int* HB_tile_H;		// tiles's vertical high boundaries
	Viewport(double Fh_, double Fv_, int vp_W_, int vp_H_, int erp_W_, int erp_H_, int tile_W_, int tile_H_);
	~Viewport();
	void calcVisibleMask(double phi, double theta, int** vmask, int** pixel);
	
};
#endif
