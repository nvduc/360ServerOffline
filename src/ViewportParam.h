typedef struct ViewportParam{
  double Fh;				// Horizontal FoV (rad)
  double Fv;				// Vertical FoV (rad)
  int vp_W;				// Viewport's width (Number of pixels)
  int vp_H;				// Viewport's height (Number of pixels)
  int erp_W;				// ERP's width
  int erp_H;				// ERP's height
  int tile_W;				// tile's width
  int tile_H;				// tile's height
  double SD; 				// segment duration in secs
  int SESS_DUR; 			// session duration in secs
  int NO_VER;				// Number of available versions
  int FPS;				// Frame per second
  int NO_SEG;				// Number of segment from trace data
  int BUFFSIZE; 			// buffer size as number of frames
  int NO_SEG_FULL;		// Numer of segments in the sessions
  int INTERVAL;			// adaptation interval in frames
  int METHOD;				// adaptation method
  double margin;			// bandwidth estimation margin
  const char* head_pos_trace;  // head position trace
  double speed;			// head movement speed
  double delay; 			// network delay
  int NO_FRAME;			// total number of frames in a session.
  int NO_FRAME_ORIGIN;	// total number of frames in the considered video.
  double MAX_BW;
};
