/*
 * port.h
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */

#ifndef CODE_GUI_PORT_H_
#define CODE_GUI_PORT_H_
#include "GUI.h"
/*********flash相关宏定义*****************/
#define EEPROM_SAVE_SECTOR 11    //参数储存的页码
#define EEPROM_BACKUP_SECTOR 10  //备用参数储存的页码
#define EEPROM_BACKBACKUP_SECTOR 9 //二级备用数据存储页码
#define EEPROM2_SAVE_SECTOR 8    //参数储存的页码（第二页）
#define EEPROM2_BACKUP_SECTOR 7  //备用参数储存的页码（第二页）
/********************************************/

/**********gpio相关宏定义***********/
//该定义兼容拨轮按钮，上拨所对引脚相当于KEY1_PIN，下拨所对引脚相当于KEY2_PIN，按下所对引脚相当于KEY3_PIN
#define KEY1_PIN 1//gpio_get_level(P10_6)
#define KEY2_PIN 1//gpio_get_level(P10_2)          //这几个引脚要改成自己的板子所对应的引脚，形式也要与自己的库一致
#define KEY3_PIN 1//gpio_get_level(P10_5)
#define KEY4_PIN 1//gpio_get_level(P22_2)

#define PRESS_PIN gpio_get_level(P15_8)
#define CW_PIN    gpio_get_level(P15_6)
#define CCW_PIN   gpio_get_level(P15_7)

//IO口电平读取函数
#define GUI_gpio_get_level(gpio_pin)   gpio_get_level(gpio_pin)    //改成自己的库中对应读取引脚的函数
/*********************************************/

/***********延迟函数的接口函数****/
#define GUI_delay_ms(nms)   system_delay_ms(nms)                   //改成自己的库中的延迟函数
/***********************************/


//函数声明，可以不用动
void GUI_flash_erase_page(u32 page);
void GUI_flash_read_page( u32 page_num, u32 *buf, u16 len);
void GUI_flash_write_page(u32 page_num, const u32 *buf, u16 len);

void GUI_display_string(u16 x,u16 y,const char dat[]);
void GUI_draw_point(uint16 x, uint16 y, const uint16 color);
void GUI_draw_line(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);
void key_scan();
void ec11_scan();




#endif /* CODE_GUI_PORT_H_ */
