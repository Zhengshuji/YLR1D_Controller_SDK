#ifndef RMACRODEFINE_HPP
#define RMACRODEFINE_HPP

#ifdef __linux__
#ifndef DECLSPEC_DLLEXPORT
#define DECLSPEC_DLLEXPORT
#endif
#elif _WIN32
#ifndef DECLSPEC_DLLEXPORT
#define DECLSPEC_DLLEXPORT  __declspec(dllexport)
#endif
#endif

#define EPSLON 1e-8 // 零元
#define PI 3.14159265358979323846
#define GOLDENRATIO (sqrt(5)-1)/2
#define GRAVITY_ACC 9.8
#define INVERSE_KINEMATICS_ITERATION 10000

#endif
