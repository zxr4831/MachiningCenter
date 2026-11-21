#ifndef __CONFIG_H
#define __CONFIG_H

/***********************编译环境设置**************************/
#ifndef STM32_MODE
#define STM32_MODE		0 //在STM32时为1  VS中使用为0 
#endif
#ifndef ENCODER_MODE
#define ENCODER_MODE STM32_MODE
#endif


/* 默认启用距离手轮模拟与停车边界；VS 通过原编码器样本入口提供手轮输入。
 * 可在工程预处理器中显式关闭；改变开关后必须全量重新编译。 */
#ifndef HANDWHEEL_SIM_DISTANCE_ENABLE
#define HANDWHEEL_SIM_DISTANCE_ENABLE 1
#endif
#ifndef HANDWHEEL_STOP_BOUNDARY_ENABLE
#define HANDWHEEL_STOP_BOUNDARY_ENABLE 1
#endif

#if STM32_MODE || HANDWHEEL_SIM_DISTANCE_ENABLE || HANDWHEEL_STOP_BOUNDARY_ENABLE
#define HANDWHEEL_SIM_DISTANCE_DEFAULT 1
#endif

#if HANDWHEEL_SIM_DISTANCE_ENABLE
#define HWSIM_RESPONSE_TICKS_PER_REV 100.0
#define HWSIM_RESPONSE_MAX_RPM 3000.0
#define HWSIM_RESPONSE_GAIN_TIME 20.0
#define HWSIM_RESPONSE_LOOKAHEAD 40.0
#define HWSIM_RESPONSE_IDLE_MIN 30.0
#define HWSIM_RESPONSE_IDLE_MAX 100.0
#ifndef HANDWHEEL_SIM_G0_SPEED_LIMIT
#define HANDWHEEL_SIM_G0_SPEED_LIMIT 0.1
#endif
#endif

#define SPECIAL_MACHINE 0//专机模式（双主轴机，两个通道共用一套刀具，专机模式时设置为1）



/***********************运行机器配置**************************/
/*************************************************
目前的作用：
1.根据不同的机器配置不同的物理轴映射
2.根据不同的机器置不同的IO
**************************************************/
#define NORMAL_SINGLECHANNEL_MACHINE				0	//单通道_普通二轴机
#define TURNING_AND_MILLING_SINGLECHANNEL_MACHINE	1	//单通道_车铣复合机
#define NORMAL_MULTICHANNEL_ADDYAXIS_MACHINE		2	//多通道Y轴机	1通道：XZ轴+C轴		2通道：Y轴
#define NORMAL_MULTICHANNEL_ADDCAXIS_MACHINE		3	//多通道双主轴机	1通道：XZ轴+C1轴	2通道：XZ轴+C2轴
#define TURNING_AND_MILLING_INCLINED_Y_SINGLECHANNEL_MACHINE	4	//单通道_车铣复合倾斜Y轴机

#define MACHINE_CONFIG        NORMAL_MULTICHANNEL_ADDYAXIS_MACHINE
//配置为双主轴机时，要将common.h文件中的#define TURRET_MAX_NUM 配置为1
//配置为Y轴机时，要将common.h文件中的#define TURRET_MAX_NUM 配置为2
//配置为单通道_普通二轴机时，要将common.h文件中的#define TURRET_MAX_NUM 配置为1
//配置为单通道_车铣复合机时，要将common.h文件中的#define TURRET_MAX_NUM 配置为1， 并且在 initaprray()函数中根据通道对轴的映射修改函数setTurretToAxisMap（1，X）中的参数X


/***********************芯片选择**************************/
#define CotrexF4                                    1  //芯片选择 1：F4 0:H7  H7与F4底层的IO使用方式不同

/***********************专用机器配置**************************/
#define SWING_ARM_MACHINE  0  //摆臂机 0：其他机型  1：摆臂机


#endif
