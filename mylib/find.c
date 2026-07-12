#include "ti_msp_dl_config.h"
#include "mylib/find.h"

// #define G_A_PART (XG_G1_PIN | XG_G2_PIN | XG_G5_PIN | XG_G6_PIN )
               
// #define G_B_PART (XG_G3_PIN | XG_G4_PIN | XG_G7_PIN | XG_G8_PIN)

// typedef struct
// {
//     GPIO_Regs *port;//引脚部分
//     uint32_t   pin;//引脚名称
// } IR_Pin;  

// static IR_Pin ir_map[IR_NUM] = {
//     {XG_G1_PORT,XG_G1_PIN},
//     {XG_G2_PORT,XG_G2_PIN},
//     {XG_G3_PORT,XG_G3_PIN},
//     {XG_G4_PORT,XG_G4_PIN},
//     {XG_G5_PORT,XG_G5_PIN},
//     {XG_G6_PORT,XG_G6_PIN},
//     {XG_G7_PORT,XG_G7_PIN},
//     {XG_G8_PORT,XG_G8_PIN},
    
    
// };
//uint32_t ir_valA = DL_GPIO_readPins(XG_G1_PORT ,G_A_PART);


//简介：读取循迹的参数并转换为数组
//参数：传入一个存数据的数组
void IR_Read(uint8_t *ir)
{
    ir[0] = (DL_GPIO_readPins(XG_G1_PORT,XG_G1_PIN) ? 1 : 0);
    ir[1] = (DL_GPIO_readPins(XG_G2_PORT,XG_G2_PIN) ? 1 : 0);
    ir[2] = (DL_GPIO_readPins(XG_G3_PORT,XG_G3_PIN) ? 1 : 0);
    ir[3] = (DL_GPIO_readPins(XG_G4_PORT,XG_G4_PIN) ? 1 : 0);
    ir[4] = (DL_GPIO_readPins(XG_G5_PORT,XG_G5_PIN) ? 1 : 0);
    ir[5] = (DL_GPIO_readPins(XG_G6_PORT,XG_G6_PIN) ? 1 : 0);
    ir[6] = (DL_GPIO_readPins(XG_G7_PORT,XG_G7_PIN) ? 1 : 0);
    ir[7] = (DL_GPIO_readPins(XG_G8_PORT,XG_G8_PIN) ? 1 : 0);
}