/*
 * circle.h
 *
 *  Created on: 2023Äê6ÔÂ21ÈÕ
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_CIRCLE_H_
#define CODE_CAMERA_PROCESS_CIRCLE_H_
#include "global.h"
void ring_identification();
void left_ring_line_scan();
void right_ring_identification();

void left_ring_first_angle();
void left_ring_confirm();
void left_ring_begin_turn();
void left_ring_circular_arc();
void left_ring_second_angle();
void left_ring_in_loop();
void left_ring_prepare_out();
void left_ring_out_loop_turn();
void left_ring_out_angle();
void left_ring_out_loop();
void left_ring_straight_out_angle();
void left_ring_complete_out();

void right_ring_first_angle();
void right_ring_confirm();
void right_ring_begin_turn();
void right_ring_circular_arc();
void right_ring_second_angle();
void right_ring_in_loop();
void right_ring_prepare_out();
void right_ring_out_angle();
void right_ring_out_loop_turn();
void right_ring_out_loop();
void right_ring_straight_out_angle();
void right_ring_complete_out();

void find_angle_left_down(int*angle_x,int*angle_y);
void find_angle_right_down(int*angle_x,int*angle_y);

#endif /* CODE_CAMERA_PROCESS_CIRCLE_H_ */
