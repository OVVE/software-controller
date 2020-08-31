
#ifndef __UTILS_UTIL_H__
#define __UTILS_UTIL_H__

// Standard math utility macros

// TODO: Consider safer macros here for these operations
#ifndef max
#define max(a, b) ((a) < (b) ? (b) : (a))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#endif /* __UTILS_UTIL_H__ */