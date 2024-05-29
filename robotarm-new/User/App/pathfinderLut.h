#ifndef PATHFINDER_LUT_H
#define PATHFINDER_LUT_H

#define PI 3.14159

// 12个动作阶段，对应12个 Index
uint8_t pf_silverOrePick_IndexNum = 6;	

// 每个动作的持续时间
float pathfinder_silverOrePick_tick2IndexLut[] ={
1000,	// 0	准备好
1500,
1300,	// 0	准备好
700,	// 1	对位
1000,	// 2	下降
2500,	// 3	抬升
};
// 每个动作对应的关节Q值
float pathfinder_silverOrePick_tick2QMatLut[] ={
0.00, 1.20, -2.60, 0.f,		// 0	收回来
0.00, 1.50, -3.09, 0.f,		// 0	收回来
0.24, 1.50, -3.09, 0.f,		// 0	抬起来
0.24, 1.10, -2.40, 1.17,	// 1	到矿石上方
0.16, 1.10, -2.40, 1.17,	// 2	下降
0.24, 1.10, -2.40, 1.17,	// 3	上升
};



#endif