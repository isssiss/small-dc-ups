/*
 * Name:		dc_ups.ino
 * Created:	2022-02-11
 * Author:	Isto Saarinen
 */


#ifndef _UPS_UTIL_h
#define _UPS_UTIL_h

#include <stdint.h>






// Format uint16_t => ###
char* format_uint3(char* buffer, uint16_t value);

// Format float => ##.#
char* format_float2(char* buffer, float value);

// Format float => ###.#
char* format_float3(char* buffer, float value);



#endif

