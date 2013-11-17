/******************************************************************************
/
/               MM Solutions AD - Sofia, Bulgaria (www.mm-sol.com)
/
/ Filename:		"yuv422yuv420.h"
/
/ Author:		Nikolay V. Nikolov
/
/ *****************************************************************************/

#ifndef YUV422YUV420_H
#define YUV422YUV420_H

#include <stdint.h>

#define CEILING(X,M)	((((X)+(M)-1)/(M))*(M))

#define YUV_PIXPERELEMENT	(2)

typedef enum {
	YUV420_FORMAT_NV12,
	YUV420_FORMAT_NV21,
	YUV420_FORMAT_invalid
} YUV420_FORMAT_t;

typedef struct {
	uint16_t *buf;
	int w;
	int h;
	int bytes;
	YUV420_FORMAT_t fmt;
} YUV420_BUF_t;

typedef enum {
	YUV422_FORMAT_YUYV,
	YUV422_FORMAT_YVYU,
	YUV422_FORMAT_UYVY,
	YUV422_FORMAT_VYUY,
	YUV422_FORMAT_invalid
} YUV422_FORMAT_t;

typedef struct {
	uint32_t *buf;
	int w;
	int h;
	int bytes;
	YUV422_FORMAT_t fmt;
} YUV422_BUF_t;

typedef struct {
	int x;
	int y;
} WINBASE_t;

typedef struct {
	WINBASE_t d;
	WINBASE_t s;
	int w;
	int h;
} YUVWINCONV_PARAMS_t;

typedef enum {
	YUV422YUV420_CONV_STATUS_OK,
	YUV422YUV420_CONV_STATUS_ERROR
} YUV422YUV420_CONV_STATUS_t;

#ifdef __cplusplus
extern "C" {

YUV422YUV420_CONV_STATUS_t yuv422yuv420_neon( const YUV420_BUF_t * const dst, const YUV422_BUF_t * const src,
	const YUVWINCONV_PARAMS_t * const win);

unsigned int yuv422yuv420_get_version(void);

};
#endif	///#ifdef __cplusplus

#endif	///	#ifndef YUV422YUV420_H

