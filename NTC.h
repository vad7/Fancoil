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

#ifndef NTC_h
#define NTC_h

#define TEMP_TABLE_ADC_VALUES			// Table contains 16 bit(max) ADC value, when omitted 32 bit resistance in Om
#define TEMP_AVERAGING_SAMPLES	5
#define TEMP_DECIMAL_DIVIDER 	10			// Десятые градуса, Точность температур - после запятой
// Подключение датчика [Vcc] - [ResistorToVcc] - [Analog input] - [NTC] - [Gnd]
#define TEMP_VCC				50			// *10 V
#define TEMP_AREF				50			// *10 V
// Значение температуры соответствующее первому значению таблицы
#define TEMP_TABLE_START		(-20 * TEMP_DECIMAL_DIVIDER)
// Шаг таблицы
#define TEMP_TABLE_STEP			(5 * TEMP_DECIMAL_DIVIDER)
#define TEMP_NULL				(-99 * TEMP_DECIMAL_DIVIDER - TEMP_DECIMAL_DIVIDER + 1 )
#ifdef TEMP_TABLE_ADC_VALUES
#define TEMP_TABLE uint16_t
#else
#define TEMP_TABLE uint32_t
#endif

// Метод доступа к элементу таблицы, должна соответствовать temperature_table_entry_type
#ifdef __AVR__
#ifdef TEMP_TABLE_ADC_VALUES
	#define TEMP_TABLE_READ(i) pgm_read_word(&res_table[i])
#else
	#define TEMP_TABLE_READ(i) pgm_read_dword(&res_table[i])
#endif
	#define ADC_MAX 1024	// 10 bit
#else
	#define TEMP_TABLE_READ(i) res_table[i]
	#define ADC_MAX 4096	// 16 bit, Need check calc_adc for higher resistance!
#endif

class NTC {
public:
	NTC():T(TEMP_NULL),adcval(0) {};
#ifdef TEMP_TABLE_ADC_VALUES
	void init(uint16_t adcfirst, const TEMP_TABLE *ResistanceTable, uint8_t TableEntries); // ResistanceTable in Om.
#else
	void init(uint16_t adcfirst, const TEMP_TABLE *ResistanceTable, uint8_t TableEntries, uint32_t ResisorToVcc);
#endif
	void calc_temperature(uint16_t adc);
	void set_R_to_Vcc(uint16_t R) { R_TO_VCC = R; }
	uint32_t calc_resistance(void);	// Om
	int16_t  T;									// Temperature, dimension depends on TEMPERATURE_TABLE_STEP
	uint16_t adcval;
private:
	uint16_t calc_adc(uint32_t R);
	uint32_t R_TO_VCC;				// Resistor on mk port beetween Vcc and NTC (Om)
	const TEMP_TABLE *res_table;
	uint8_t  res_table_max;
#ifdef TEMP_AVERAGING_SAMPLES
#if ((TEMP_AVERAGING_SAMPLES + 1) * ADC_MAX) > 0x7FFF
	uint32_t average_sum;
#else
	uint16_t average_sum;
#endif
	uint16_t average[TEMP_AVERAGING_SAMPLES];
	uint8_t average_pos;
#endif
};

#endif
