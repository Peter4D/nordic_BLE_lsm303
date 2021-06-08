/**
 * @file common.h
 * @author Peter Medvesek (peter.medvesek@gorenje.com)
 * @brief 
 * @version 0.1
 * @date 2021-06-08
 * 
 * @copyright Copyright (c) 2021 Gorenje d.o.o
 * 
 */
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})
        

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */
