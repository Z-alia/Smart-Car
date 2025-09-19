#ifndef __image_process_h__
#define __image_process_h__
#include "global.h"
typedef struct Image
{    
	uint16_t output_image[120][180];
    uint16_t original_image[120][180];
    uint8_t x_points[120];
    uint8_t y_points[120];
} Image;

void image_edge_detection_optimized(Image *image , int width, int height);
void fit_line(Image *image, int width, float *slope, float *intercept);
void image_process_init(Image *image);
#endif
