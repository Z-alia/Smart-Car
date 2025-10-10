#ifndef soft_iic_h_
#define soft_iic_h_

#include "main.h"

//typedef struct
//{
//    uint32_t              scl_pin;                                                
//    uint32_t              sda_pin;                                                
//    uint8_t               addr;                                                   
//    uint32_t              delay;                                                  
//}soft_iic_info_struct;
typedef struct
{
    GPIO_TypeDef* scl_port;// 用于记录对应的引脚组
    uint16_t      scl_pin;// 用于记录对应的引脚编号
    GPIO_TypeDef* sda_port;// 用于记录对应的引脚组
    uint16_t      sda_pin;// 用于记录对应的引脚编号
    uint8_t       addr;// 器件地址 七位地址模式
    uint32_t      delay;// 模拟 IIC 软延时时长
} soft_iic_info_struct;

void        soft_iic_write_8bit             (soft_iic_info_struct *soft_iic_obj, const uint8_t data);
void        soft_iic_write_8bit_array       (soft_iic_info_struct *soft_iic_obj, const uint8_t *data, uint32_t len);

void        soft_iic_write_16bit            (soft_iic_info_struct *soft_iic_obj, const uint16_t data);
void        soft_iic_write_16bit_array      (soft_iic_info_struct *soft_iic_obj, const uint16_t *data, uint32_t len);

void        soft_iic_write_8bit_register    (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t data);
void        soft_iic_write_8bit_registers   (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, const uint8_t *data, uint32_t len);

void        soft_iic_write_16bit_register   (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name, const uint16_t data);
void        soft_iic_write_16bit_registers  (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name, const uint16_t *data, uint32_t len);

uint8_t       soft_iic_read_8bit              (soft_iic_info_struct *soft_iic_obj);
void        soft_iic_read_8bit_array        (soft_iic_info_struct *soft_iic_obj, uint8_t *data, uint32_t len);

uint16_t      soft_iic_read_16bit             (soft_iic_info_struct *soft_iic_obj);
void        soft_iic_read_16bit_array       (soft_iic_info_struct *soft_iic_obj, uint16_t *data, uint32_t len);

uint8_t       soft_iic_read_8bit_register     (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name);
void        soft_iic_read_8bit_registers    (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, uint8_t *data, uint32_t len);

uint16_t      soft_iic_read_16bit_register    (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name);
void        soft_iic_read_16bit_registers   (soft_iic_info_struct *soft_iic_obj, const uint16_t register_name, uint16_t *data, uint32_t len);

void        soft_iic_transfer_8bit_array    (soft_iic_info_struct *soft_iic_obj, const uint8_t *write_data, uint32_t write_len, uint8_t *read_data, uint32_t read_len);
void        soft_iic_transfer_16bit_array   (soft_iic_info_struct *soft_iic_obj, const uint16_t *write_data, uint32_t write_len, uint16_t *read_data, uint32_t read_len);

void        soft_iic_sccb_write_register    (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name, uint8_t data);
uint8_t       soft_iic_sccb_read_register     (soft_iic_info_struct *soft_iic_obj, const uint8_t register_name);

void soft_iic_init(soft_iic_info_struct *obj,
                   GPIO_TypeDef* scl_port, uint32_t scl_pin,
                   GPIO_TypeDef* sda_port, uint32_t sda_pin,
                   uint8_t addr, uint32_t delay);

#endif

