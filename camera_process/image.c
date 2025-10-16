//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "Binarization.h"
#include "lcd_spi_200.h"
#include "morph_binary_bitpacked.h"
/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2022年9月8日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2022年9月8日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
	if (x > y)             return y;
	else if (x < -y)       return -y;
	else                return x;
}

//二值化后bin_image用Grayscale取代

/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2022年9月8日
备    注：
example：  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
	uint16 i = 0,l_found = 0,r_found = 0;
	//清零
	start_point_l[0] = 0;//x
	start_point_l[1] = 0;//y

	start_point_r[0] = 0;//x
	start_point_r[1] = 0;//y

		//从中间往左边，先找起点
	for (i = image_w / 2; i > border_min; i--)
	{
		start_point_l[0] = i;//x
		start_point_l[1] = start_row;//y
		if (Grayscale[start_row][i] == 255 && Grayscale[start_row][i - 1] == 0)
		{
			//printf("找到左边起点image[%d][%d]\n", start_row,i);
			l_found = 1;
			break;
		}
	}

	for (i = image_w / 2; i < border_max; i++)
	{
		start_point_r[0] = i;//x
		start_point_r[1] = start_row;//y
		if (Grayscale[start_row][i] == 255 && Grayscale[start_row][i + 1] == 0)
		{
			//printf("找到右边起点image[%d][%d]\n",start_row, i);
			r_found = 1;
			break;
		}
	}

	if(l_found&&r_found)return 1;
	else {
		//printf("未找到起点\n");
		return 0;
	} 
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
							uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r			：最多需要循环的次数
(*image)[image_w]		：需要进行找点的图像数组，必须是二值图,填入数组名称即可
					   特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic				：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic				：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x				：左边起点横坐标
l_start_y				：左边起点纵坐标
r_start_x				：右边起点横坐标
r_start_y				：右边起点纵坐标
hightest				：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
	search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
				start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */
#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

	uint8 i = 0, j = 0;

	//左边变量
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;//统计左边
	//定义八个邻域
	static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针

	//右边变量
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };//中心坐标点
	uint8 index_r = 0;//索引下标
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//统计右边
	//定义八个邻域
	static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//开启邻域循环
	while (break_flag--)
	{

		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i + 1) & 7][0];
				temp_l[index_l][1] = search_filds_l[(i + 1) & 7][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i);//记录生长方向
			}
		}

		// 决策逻辑移到循环外部
		if (index_l)
		{
			//更新坐标点
			center_point_l[0] = temp_l[0][0];//x
			center_point_l[1] = temp_l[0][1];//y
			for (j = 0; j < index_l; j++)
			{
				if (center_point_l[1] > temp_l[j][1])
				{
					center_point_l[0] = temp_l[j][0];//x
					center_point_l[1] = temp_l[j][1];//y
				}
			}
		}
		if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
			&& points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
			||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
				&& points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
		{
			//printf("三次进入同一个点，退出\n");
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1]) < 2
			)
		{
			//printf("\n左右相遇退出\n");	
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			//printf("\n如果左边比右边高了，左边等待右边\n");	
			continue;//如果左边比右边高了，左边等待右边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			//printf("\n左边开始向下了，等待右边，等待中... \n");
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零，后使用
			temp_r[i][1] = 0;//先清零，后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i + 1) & 7][0];
				temp_r[index_r][1] = search_filds_r[(i + 1) & 7][1];
				index_r++;//索引加一
				dir_r[r_data_statics - 1] = (i);//记录生长方向
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
		}

		// 决策逻辑移到循环外部
		if (index_r)
		{
			//更新坐标点
			center_point_r[0] = temp_r[0][0];//x
			center_point_r[1] = temp_r[0][1];//y
			for (j = 0; j < index_r; j++)
			{
				if (center_point_r[1] > temp_r[j][1])
				{
					center_point_r[0] = temp_r[j][0];//x
					center_point_r[1] = temp_r[j][1];//y
				}
			}
		}


	}


	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L	：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[image_h];//左线数组
uint8 r_border[image_h];//右线数组
uint8 center_line[image_h];//中线数组
void get_left(uint16 total_L)
{
	uint16 j;
	//初始化左边界为最小值
	for (j = 0; j < image_h; j++)
	{
		l_border[j] = border_min;
	}

	// 遍历所有找到的点，更新l_border数组
	for (j = 0; j < total_L; j++)
	{
		uint16 row = points_l[j][1];
		uint16 col = points_l[j][0];
		if (row < image_h) // 确保行号在范围内
		{
			// 如果当前行的边界还是初始值，或者找到了一个更左边的点
			if (l_border[row] == border_min || col < l_border[row])
			{
				l_border[row] = col;
			}
		}
	}
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
	uint16 j;
	//初始化右边界为最大值
	for (j = 0; j < image_h; j++)
	{
		r_border[j] = border_max;
	}

	// 遍历所有找到的点，更新r_border数组
	for (j = 0; j < total_R; j++)
	{
		uint16 row = points_r[j][1];
		uint16 col = points_r[j][0];
		if (row < image_h) // 确保行号在范围内
		{
			// 如果当前行的边界还是初始值，或者找到了一个更右边的点
			if (r_border[row] == border_max || col > r_border[row])
			{
				r_border[row] = col;
			}
		}
	}
}

/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[image_w]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{

	uint8 i = 0;
	for (i = 0; i < image_h; i++)
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][image_w - 1] = 0;
		image[i][image_w - 2] = 0;

	}
	for (i = 0; i < image_w; i++)
	{
		image[0][i] = 0;
		image[1][i] = 0;
		//image[image_h-1][i] = 0;

	}
}

//绘制边界线
void draw_edge()
{
	int row=0,colum;
    for(row=0;row<120;row++)
    {
        for(colum=0;colum<image_w;colum++)
        {
            if(Grayscale[row][colum]==0)
			{
                imo[row][colum]=0;
			}
            else
			{
                imo[row][colum]=255;
			}
        }

    }
	for(row=0;row<120;row++)
    {
		imo[row][l_border[row]]=1;
		imo[row][r_border[row]]=2;
		imo[row][center_line[row]]=3;
	}
}


/** 
* @brief 最小二乘法
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
*  @see CTest		Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast=0.0f;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
*  @see CTest		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
	uint16 i, num = 0;
	uint16 xsum = 0, ysum = 0;
	float y_average, x_average;

	num = 0;
	xsum = 0;
	ysum = 0;
	y_average = 0;
	x_average = 0;
	for (i = start; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		num++;
	}

	//计算各个平均数
	if (num)
	{
		x_average = (float)xsum / (float)num;
		y_average = (float)ysum / (float)num;

	}

	/*计算斜率*/
	*slope_rate = Slope_Calculate(start, end, border);//斜率
	*intercept = y_average - (*slope_rate)*x_average;//截距
}

/** 
* @brief 十字补线函数
* @param uint8(*image)[image_w]		输入二值图像
* @param uint8 *l_border			输入左边界首地址
* @param uint8 *r_border			输入右边界首地址
* @param uint16 total_num_l			输入左边循环总次数
* @param uint16 total_num_r			输入右边循环总次数
* @param uint16 *dir_l				输入左边生长方向首地址
* @param uint16 *dir_r				输入右边生长方向首地址
* @param uint16(*points_l)[2]		输入左边轮廓首地址
* @param uint16(*points_r)[2]		输入右边轮廓首地址
*  @see CTest		cross_fill(image,l_border, r_border, data_statics_l, data_statics_r, dir_l, dir_r, points_l, points_r);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
										 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
	uint16 i;
	uint8 break_num_l = 0;
	uint8 break_num_r = 0;
	uint8 start, end;
	float slope_l_rate = 0, intercept_l = 0;
	//出十字
	for (i = 1; i + 7 < total_num_l; i++)
	{
		if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
		{
			break_num_l = points_l[i][1];//传递y坐标
			//printf("brea_knum-L:%d\n", break_num_l);
			//printf("I:%d\n", i);
			//printf("十字标志位：1\n");
			break;
		}
	}
	for (i = 1; i + 7 < total_num_r; i++)
	{
		if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
		{
			break_num_r = points_r[i][1];//传递y坐标
			//printf("brea_knum-R:%d\n", break_num_r);
			//printf("I:%d\n", i);
			//printf("十字标志位：1\n");
			break;
		}
	}
	if (break_num_l&&break_num_r&&image[image_h - 1][4] && image[image_h - 1][image_w - 4])//两边生长方向都符合条件
	{
		//计算斜率
		start = break_num_l - 15;
		start = limit_a_b(start, 0, image_h-1);
		end = break_num_l - 5;
		calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
		//printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
		for (i = break_num_l - 5; i < image_h - 1; i++)
		{
			l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
			l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
		}

		//计算斜率
		start = break_num_r - 15;//起点
		start = limit_a_b(start, 0, image_h-1);//限幅
		end = break_num_r - 5;//终点
		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
		//printf("slope_l_rate:%d\nintercept_l:%d\n", slope_l_rate, intercept_l);
		for (i = break_num_r - 5; i < image_h - 1; i++)
		{
			r_border[i] = slope_l_rate * (i)+intercept_l;
			r_border[i] = limit_a_b(r_border[i], border_min, border_max);
		}


	}

}


/*
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_process();
 */
void image_process(void)
{
uint16 i;
uint8 Hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小

//滤波（形态学处理）
morph_clean_u8_binary_adapter(Grayscale[0], image_w, image_h, imo[0]);
image_draw_rectan(imo);//填黑框
//清零
data_stastics_l = 0;
data_stastics_r = 0;
if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
{
	//printf("正在开始八领域\n");
	search_l_r((uint16)USE_num, Grayscale, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
	//printf("八邻域已结束\n");
	// 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
	get_left(data_stastics_l);
	get_right(data_stastics_r);
	//处理函数放这里 不要放到if外面
    cross_fill(Grayscale, l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);//十字补线
}
    //求中线
	for (i = Hightest; i < image_h-1; i++)
	{
		center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
	}
    //显示边线
	draw_edge();
    //显示图像
	show_ov2640_image_int8(0, 0, imo[0], image_w, image_h, image_w, image_h);			




}


