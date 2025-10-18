/*
 * Binarization.h
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_BINARIZATION_H_
#define CODE_CAMERA_PROCESS_BINARIZATION_H_
#include "main.h"
#include "type_def.h"

extern uint8 Grayscale[120][188];	//灰度图
extern uint8 imo[120][188]; 		//处理后的图像


void Global_Binarization();
void Adaptive_Binarization(int S, int T);
void img_otsu_exposure_adjust();


#endif /* CODE_CAMERA_PROCESS_BINARIZATION_H_ */
