#ifndef __LCD_FONTS_H
#define __LCD_FONTS_H

#include <stdint.h>



typedef struct _pFont
{    
	 const uint8_t 		*pTable;  		// 字模数据指针
	 uint16_t 			Width; 		  // 字体宽度，单位：像素
	 uint16_t 			Height; 			// 字体高度，单位：像素
	 uint16_t 			Sizes; 			// 字模总大小（字节数）
	 uint16_t			Table_Rows;		// 字模表的行数（部分字体可能为二维表）
} pFONT;



/*------------------------------------ ASCII字体 ---------------------------------------------*/

extern pFONT ASCII_Font20; 	// 20x10 ASCII字体

#endif 
 
