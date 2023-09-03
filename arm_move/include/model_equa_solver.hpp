/*
 * @Author: Wei Zhifei
 * @Date: 2022-11-28 17:05:01
 */
#ifndef MODEL_EQUA_SOLVER
#define MODEL_EQUA_SOLVER

#include <robot_bringup/joint.h>

#define L1 0.4
#define L2 0.298
#define L3 0.360
#define pi 3.1415926

bool get_started(double _x, double _y, robot_bringup::joint &target_joint);

#endif