/*
 * Copyright (c) 2019 by Vadim Kulakov vad7@yahoo.com, vad711
  *
 * This file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 */

#include "NTC.h"

#ifndef TEMP_TABLE_ADC_VALUES
// ResistorToVcc - Резистор порт в/в между Vcc и NTC, Ом
void NTC::init(uint16_t adcfirst, const TEMP_TABLE *ResistanceTable, uint8_t TableEntries, uint32_t ResisorToVcc)
{
	R_TO_VCC = ResisorToVcc;
#else
// ResistanceTable in Om.
void NTC::init(uint16_t adcfirst, const TEMP_TABLE *ResistanceTable, uint8_t TableEntries)
{
#endif
	res_table = ResistanceTable;
	res_table_max = TableEntries - 1;
	average_sum = 0;
	for(uint8_t i = 0; i < TEMP_AVERAGING_SAMPLES; i++) average_sum += (average[i] = adcfirst);
	adcval = ~adcfirst;
	average_pos = 0;
}

uint16_t NTC::calc_adc(uint32_t R)
{
#if (TEMP_VCC == TEMP_AREF)
	return R * ADC_MAX / (R + R_TO_VCC);
#else
	return R * ADC_MAX / TEMP_AREF * TEMP_VCC / (R + R_TO_VCC);
#endif
}

uint32_t NTC::calc_resistance(void)
{
#if (TEMP_VCC == TEMP_AREF)
	uint16_t sub = ADC_MAX - adcval;
	if(sub == 0) return 0x7FFFFFFF;
	return (uint32_t)R_TO_VCC * adcval / sub;
#else
	uint32_t sub = TEMP_VCC * ADC_MAX - TEMP_AREF * adcval;
	if(sub == 0) return 0xFFFFFFFF;
	return (uint32_t)R_TO_VCC * TEMP_AREF * adcval / sub;
#endif
}

// Calc temperature to variable T
void NTC::calc_temperature(uint16_t adc)
{
#ifdef TEMP_AVERAGING_SAMPLES
	average_sum = average_sum + adc - average[average_pos];
	average[average_pos] = adc;
	adc = average_sum / TEMP_AVERAGING_SAMPLES;
	if(++average_pos == TEMP_AVERAGING_SAMPLES) average_pos = 0;
#endif
	if(adcval == adc) return;
	adcval = adc;

	uint8_t r = res_table_max;
#ifdef TEMP_TABLE_ADC_VALUES
	uint16_t val = adc;
#else
	uint32_t val = calc_resistance();
#endif
	// Двоичный поиск по таблице
	uint8_t l = 0;
	while((r - l) > 1) {
		uint8_t m = (l + r) >> 1;
		TEMP_TABLE mid = TEMP_TABLE_READ(m);
		if(val > mid) {
			r = m;
		} else {
			l = m;
		}
	}
	TEMP_TABLE vl = TEMP_TABLE_READ(l);
	//Serial.print("r="); Serial.print(r); Serial.print(",l="); Serial.println(l);
	if(val > vl) {
		T = l * TEMP_TABLE_STEP + TEMP_TABLE_START - TEMP_TABLE_STEP;
	} else {
		TEMP_TABLE vr = TEMP_TABLE_READ(r);
		if(val < vr) {
			T = r * TEMP_TABLE_STEP + TEMP_TABLE_START + TEMP_TABLE_STEP;
		} else {
			TEMP_TABLE vd = vl - vr;
			int16_t res = TEMP_TABLE_START + r * TEMP_TABLE_STEP;
			if(vd) {
				// Линейная интерполяция
				res -= ((TEMP_TABLE_STEP * (int32_t) (val - vr) + (vd >> 1)) / vd);
			}
			T = res;
		}
	}
}

