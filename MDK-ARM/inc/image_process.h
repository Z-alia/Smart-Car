#ifndef __image_process_h
#define __image_process_h
#include "global.h"
typedef struct Image
{    
	uint16_t output_image[120][188];
    uint16_t original_image[120][188];
    uint8_t x_points[120];
    uint8_t y_points[120];
} Image;
extern struct Image image_buf;


void image_edge_detection_optimized(Image *image , uint16_t width, uint16_t height);
void fit_line(Image *image, int width, float *slope, float *intercept);
void image_process_init(Image *image);
#endif
