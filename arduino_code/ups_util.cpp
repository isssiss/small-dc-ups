/*
 * Name:		dc_ups.ino
 * Created:	2022-02-11
 * Author:	Isto Saarinen
 */


#include "ups_util.h"
#include <math.h>
#include <stdio.h>




// Format uint16_t => ###
// Limit to 000 ... 999
char* format_uint3(char* buffer, uint16_t value) {
	if (value < 10) {
		sprintf(buffer, "00%d", value);
	}
	else if (value < 100) {
		sprintf(buffer, "0%d", value);
	}
	else if (value < 1000) {
		sprintf(buffer, "%d", value);
	}
	else {
		sprintf(buffer, "999");
	}
	return buffer;
}




// Format float => ##.#
// Limit to -9.9 ... 99.9
char* format_float2(char* buffer, float value) {
	int w = (int)value;
	int d = (int)round(round(((value < 0 ? -1 * value : value) - (float)w) * 100) / 10);
	int p = 5;
	if (w > 99) w = 99;
	if (w < -9) w = -9;
	if (w > 9) p = 0;
	else p = 1;
	buffer[0] = '0';
	sprintf(buffer + p, "%d.%d", w, d);
	return(buffer);
}

// Format float => ###.#
// Limit to -99.9 ... 999.9
char* format_float3(char* buffer, float value) {
	int w = (int)value;
	int d = (int)round(round(((value < 0 ? -1 * value : value) - (float)w) * 100) / 10);
	int p = 5;
	if (w > 999) w = 999;
	if (w < -99) w = -99;
	if (w > 99) p = 0;
	else if (w > 9) p = 1;
	else p = 2;
	buffer[0] = '0';
	buffer[1] = '0';
	sprintf(buffer + p, "%d.%d", w, d);
	return(buffer);
}




