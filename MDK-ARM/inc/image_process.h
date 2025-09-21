#ifndef __image_process_h
#define __image_process_h
#include "global.h"
#define Straight_path 1
#define Big_bend 1
#define Small_bend 1
#define Island_loop 1
#define Crossroads 1
#define End 1
#define scan_start 19
#define scan_end 40
#define div_detect_row  15
typedef struct Image
{    
	uint16_t output_image[120][188];
    uint16_t original_image[120][188];
    uint8_t x_points[120];
    uint8_t y_points[120];
    uint8_t x_leftedge[120];
    uint8_t x_rightedge[120];
    uint8_t Type_flag;
} Image;
extern struct Image image_buf;


void image_edge_detection_optimized(Image *image , uint16_t width, uint16_t height);
void fit_line(Image *image, int width, float *slope, float *intercept);
void image_process_init(Image *image);
#endif
