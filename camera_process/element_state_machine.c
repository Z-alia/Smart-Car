/**
 * @file element_state_machine.c
 * @brief 简洁元素识别状态机实现
 * @note 只关注watch标志位，使用switch结构
 */

#include "element_state_machine.h"
#include "Element_recognition.h"  // 引入watch结构体
#include "integral.h"
//#include "nav_recorder_all_in_one.h"

// ==================== 全局变量 ====================

ElementState_t g_element_state = STATE_NORMAL;

// ==================== 私有函数 ====================

/**
 * @brief 处理正常巡线状态
 */
static void handle_normal_state(void)
{
	left_ring_first_angle();
    // 检测环岛入环条件
    if (watch.InLoop == 1) {
        g_element_state = STATE_LOOP_ENTRY;
		
        return;
    }
    
    // 检测十字路口
    if (watch.cross_flag) {
        g_element_state = STATE_CROSS;
        return;
    }
    
    // 检测斑马线
    if (watch.zebra_flag) {
        g_element_state = STATE_ZEBRA;
        return;
    }
    
    // 检测黑色障碍物
    if (watch.black_obstacle_flag) {
        g_element_state = STATE_OBSTACLE_BLACK;
        return;
    }
    
    // 检测红色障碍物
    if (watch.Red_obstacle_flag) {
        g_element_state = STATE_OBSTACLE_RED;
        return;
    }
    
    // 检测上坡（假设watch中有ramp_flag标志位）
    // 注意：需要在Element_recognition.h中添加 uint8_t ramp_flag;
    // if (watch.ramp_flag) {
    //     g_element_state = STATE_RAMP;
    //     return;
    // }
    
    // 其他情况保持正常状态
}

/**
 * @brief 处理环岛入环状态
 */
static void handle_loop_entry_state(void)
{
		left_ring_circular_arc();
		left_ring_second_angle();
		left_ring_begin_turn();
		left_ring_prepare_out();
		left_ring_out_angle();
		left_ring_out_loop_turn();
		left_ring_out_loop();
		left_ring_straight_out_angle();
		left_ring_complete_out();
    if(watch.InLoop==2)
	  {
		  distance_integral.integeral_flag=1;
		  if(distance_integral.integeral_data>300)//150  
		  {
			  watch.InLoop=10;
			  clear_distant_integeral();
			  //distance_integral.integeral_flag=0;
		  }
	}
	  if(watch.InLoop==10)
	  {
		 distance_integral.integeral_flag=1;
		  if(distance_integral.integeral_data>400)//150  
		  {
			  watch.InLoop=4;
			  clear_distant_integeral();
			  //distance_integral.integeral_flag=0;
		  } 
	  }
//	  if(watch.InLoop==4)
//	  {
//		 distance_integral.integeral_flag=1;
//		  if(distance_integral.integeral_data>300)//150  
//		  {
//			  watch.InLoop=11;
//			  clear_distant_integeral();
//			  //distance_integral.integeral_flag=0;
//		  } 
//	  }
	  left_ring_linefix();
	   if (watch.InLoop==11) {
		   
        g_element_state = STATE_LOOP_EXIT;
        return;
    }
}




/**
 * @brief 处理十字路口状态
 */
static void handle_cross_state(void)
{
    // 十字路口通过完成
    if (!watch.cross_flag) {
        g_element_state = STATE_NORMAL;
        return;
    }
}

/**
 * @brief 处理斑马线状态
 */
static void handle_zebra_state(void)
{
    // 斑马线通过完成
    if (!watch.zebra_flag) {
        g_element_state = STATE_NORMAL;
        return;
    }
}

/**
 * @brief 处理黑色障碍物状态
 */
static void handle_obstacle_black_state(void)
{
	
    
}

/**
 * @brief 处理红色障碍物状态
 */
static void handle_obstacle_red_state(void)
{
    
    //开始复现
    // 障碍物处理完成
    if (0) {
        g_element_state = STATE_NORMAL;
        return;
    }
}

/**
 * @brief 处理上坡状态
 */
static void handle_ramp_state(void)
{
    // 上坡完成条件（根据实际情况调整）
    // 方案1：基于标志位清零（需要在watch中添加 uint8_t ramp_flag）
    // if (!watch.ramp_flag) {
    //     g_element_state = STATE_NORMAL;
    //     return;
    // }
    
    // 方案2：基于距离积分（已经爬升一定距离）
    if (distance_integral.integeral_data > 500) {  // 爬升500单位距离后恢复
        clear_distant_integeral();
        g_element_state = STATE_NORMAL;
        return;
    }
    
    // 方案3：基于时间（上坡持续一定时间后恢复）
    // static uint32_t ramp_start_time = 0;
    // if (ramp_start_time == 0) {
    //     ramp_start_time = HAL_GetTick();
    // }
    // if (HAL_GetTick() - ramp_start_time > 3000) {  // 3秒后恢复正常
    //     ramp_start_time = 0;
    //     g_element_state = STATE_NORMAL;
    //     return;
    // }
}

// ==================== 公共接口实现 ====================

/**
 * @brief 初始化元素识别状态机
 */
void element_state_machine_init(void)
{
    g_element_state = STATE_NORMAL;
}

/**
 * @brief 状态机主循环（switch结构）
 */
void element_state_machine_update(void)
{
    switch (g_element_state)
    {
        case STATE_NORMAL:
            // 正常巡线，检测各种元素
            handle_normal_state();
            break;
            
        case STATE_LOOP_ENTRY:
            // 环岛
            handle_loop_entry_state();
            break;
            
        case STATE_CROSS:
            // 十字路口
            handle_cross_state();
            break;
            
        case STATE_ZEBRA:
            // 斑马线
            handle_zebra_state();
            break;
            
        case STATE_OBSTACLE_BLACK:
            // 黑色障碍物
            handle_obstacle_black_state();
            break;
            
        case STATE_OBSTACLE_RED:
            // 红色障碍物
            handle_obstacle_red_state();
            break;
            
        case STATE_RAMP:
            // 上坡
            handle_ramp_state();
            break;
            
        default:
            // 未知状态，重置到正常
            g_element_state = STATE_NORMAL;
            break;
    }
}

/**
 * @brief 获取当前状态
 */
ElementState_t element_state_machine_get_state(void)
{
    return g_element_state;
}

/**
 * @brief 手动重置状态机
 */
void element_state_machine_reset(void)
{
    g_element_state = STATE_NORMAL;
}
