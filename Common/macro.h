#ifndef __MACRO_H
#define __MACRO_H
#include <stdint.h>
#include <stdbool.h>
#define ABS(x) ((x)>=0?(x):-(x))
#define GetSign(x) ((x)>=0?1:-1)
#define Clamp(val,min,max) ((val)<(min)?(min):((val)>(max)?(max):(val)))
#define ClampPeak(val,peak) Clamp((val),-(peak),(peak))
#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

#endif
