#include "scan_line.h"
#include <stdint.h>

uint8_t left_straight = 0;
uint8_t right_straight = 0;

// 判断三点是否近似共线（面积小于阈值）
static uint8_t is_points_straight(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, float area_threshold)
{
    // 向量叉积求三角形面积
    int16_t area = (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1);
    area=(area>=0)?area:-area;
    return area < area_threshold;
}

// 等距取三个左边缘点和三个右边缘点，判断是否为直线
void check_line_straight(uint8_t start, uint8_t end, float area_threshold)
{
    uint8_t step = (end - start) / 2;
    uint8_t idx1 = start;
    uint8_t idx2 = start + step;
    uint8_t idx3 = end;
    // 左边缘点
    int16_t x1 = lineinfo[idx1].left;
    int16_t y1 = lineinfo[idx1].y;
    int16_t x2 = lineinfo[idx2].left;
    int16_t y2 = lineinfo[idx2].y;
    int16_t x3 = lineinfo[idx3].left;
    int16_t y3 = lineinfo[idx3].y;
    left_straight = is_points_straight(x1, y1, x2, y2, x3, y3, area_threshold);
    // 右边缘点
    x1 = lineinfo[idx1].right;
    y1 = lineinfo[idx1].y;
    x2 = lineinfo[idx2].right;
    y2 = lineinfo[idx2].y;
    x3 = lineinfo[idx3].right;
    y3 = lineinfo[idx3].y;
    right_straight = is_points_straight(x1, y1, x2, y2, x3, y3, area_threshold);
}
