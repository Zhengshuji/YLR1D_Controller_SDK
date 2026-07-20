#ifndef CMATHBASIC_HPP
#define CMATHBASIC_HPP

#include <vector>
#include <cmath>
#include "RMacroDefine.hpp"

namespace robsoft{

enum EXTREME_VALUE{EXTREME_MAX = 0, EXTREME_MIN, EXTREME_ABS_MAX, EXTREME_ABS_MIN};

DECLSPEC_DLLEXPORT bool num_is_zero(double num);   // 判断数值是否小于零元
DECLSPEC_DLLEXPORT bool num_is_zero(double num, uint8_t pre);   // 判断数值是否为0，指定精度
DECLSPEC_DLLEXPORT double fitNumBoundary(double num, double lBound = -1, double rBound = 1);    // 将超出边界附近的数值，调整到边界
DECLSPEC_DLLEXPORT double sqrt_correct_zero(double num); // 修正负数零求根

DECLSPEC_DLLEXPORT double sin_deg(double angle);
DECLSPEC_DLLEXPORT double cos_deg(double angle);
DECLSPEC_DLLEXPORT double tan_deg(double angle);
DECLSPEC_DLLEXPORT double acos_deg(double value);
DECLSPEC_DLLEXPORT double atan2_deg(double y, double x);

DECLSPEC_DLLEXPORT double deg2rad(double degree);
DECLSPEC_DLLEXPORT double rad2deg(double radian);

DECLSPEC_DLLEXPORT int sgn(double value);

DECLSPEC_DLLEXPORT double cube(double value);  // 求解立方根

DECLSPEC_DLLEXPORT std::vector<double> solveQuadraticEquation(double a, double b, double c);   // 求解一元二次方程
DECLSPEC_DLLEXPORT std::vector<double> solveCubicEquation(double a, double b, double c, double d); // 盛金法求一元三次方程
DECLSPEC_DLLEXPORT std::vector<double> solveQuarticEquation(double a, double b, double c, double d, double e); // 求解一元四次

DECLSPEC_DLLEXPORT std::vector<double> solveEquation(const std::vector<double> &coe);//求解一元方程通用接口

// 黄金分割法求解函数最大值对应的自变量
template<typename T>
DECLSPEC_DLLEXPORT double find_max_golden_section(const T& func, double lbound, double rbound){
    double a=rbound-GOLDENRATIO*(rbound-lbound);
    double b=lbound+GOLDENRATIO*(rbound-lbound);
    double fa=func(a);
    double fb=func(b);
    while (fabs((a-b)/b)>EPSLON) {
        if(fa<=fb){
            lbound=a;
            a=b;
            fa=fb;
            b=lbound+GOLDENRATIO*(rbound-lbound);
            fb=func(b);
        }
        else{
            rbound=b;
            b=a;
            fb=fa;
            a=rbound-GOLDENRATIO*(rbound-lbound);
            fa=func(a);
        }
    }
    return (a+b)/2;
}
//黄金分割法求解函数极值，主要针对单峰函数
template<typename T>
DECLSPEC_DLLEXPORT double find_extreme_golden_section(const T& func, double lbound, double rbound, EXTREME_VALUE flag){
    double y = find_max_golden_section([=](double x){
        switch(flag){
        case EXTREME_MAX: return func(x); break;
        case EXTREME_MIN: return -func(x); break;
        case EXTREME_ABS_MAX: return fabs(func(x)); break;
        case EXTREME_ABS_MIN: return -fabs(func(x)); break;
        default: return func(x); break;
        }
    }, lbound, rbound);
    switch (flag) {
    case EXTREME_MAX:
    case EXTREME_MIN:
        return func(y);
        break;
    case EXTREME_ABS_MAX:
    case EXTREME_ABS_MIN:
        return fabs(func(y));
        break;
    default:
        return func(y);
        break;
    }
}

}

#endif
