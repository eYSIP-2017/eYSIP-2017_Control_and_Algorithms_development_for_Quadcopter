/*
 * common.c
 *
 *  Created on: Jun 15, 2017
 *      Author: Heethesh
 */

#include "common.h"

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float lowPassFilter(struct LPF *var, float current)
{
	current = ((1 - var->beta) * current) + (var->beta * var->last);
	var->last = current;
	return current;
}
