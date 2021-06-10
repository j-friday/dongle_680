/*************************************************************
 * @file        BK_System.h
 * @brief       Header file of BK_System.c
 * @author      GuWenFu
 * @version     V1.0
 * @date        2016-09-29
 * @par         
 * @attention   
 *
 * @history     2016-09-29 gwf    create this file
 */

#ifndef  __BK_SYSTEM_H__
#define  __BK_SYSTEM_H__


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */



#define MAX(a,b)    (((a) > (b)) ? (a) : (b))
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))
#define NUMBER_ROUND_UP(a,b)        ((a) / (b) + (((a) % (b)) ? 1 : 0))
#define NUMBER_ROUND_DOWN(a,b)      ((a) / (b))


typedef unsigned char       BYTE;
typedef signed   char       int8;       // �з���8λ���ͱ���
typedef signed   short      int16;      // �з���16λ���ͱ���
typedef signed   int        int32;      // �з���32λ���ͱ���
typedef unsigned int        uint32; 
typedef unsigned char       uint8;      // �޷���8λ���ͱ���
typedef unsigned short      uint16; 



typedef signed   char       int8_t;     // �з���8λ���ͱ���
typedef signed   short      int16_t;    // �з���16λ���ͱ���
//typedef signed   int       int32_t;    // �з���32λ���ͱ���
typedef unsigned char       uint8_t;    // �޷���8λ���ͱ���
typedef unsigned short      uint16_t;   // �޷���16λ���ͱ���
#if defined(__CC_ARM)
typedef signed   int       int32_t;    // �з���32λ���ͱ���
typedef unsigned int       uint32_t;   // �޷���32λ���ͱ���

#endif


#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif      /* __BK_SYSTEM_H__ */
