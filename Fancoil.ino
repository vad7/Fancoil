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

#include "Arduino.h"
#include <avr/wdt.h>
#include <util/atomic.h>
#include "LiquidCrystal.h"
#include "NTC.h"

// MiniCore board: Atmega8a
// BOD 4.0V
// Clock 8.192 MHz (UART0=115200)
// LTO enabled
#define VERSION F("1.00")
//#define DEBUG_TO_SERIAL

#define KEYS_MUX		((1<<REFS0) | analogPinToChannel(A2))
//#define EXIST_R10		// R10 = 30k pullup keys line to VCC
#ifndef EXIST_R10
#define KEYS_INIT		PORTC |= (1<<PC2) // Pull-up, to GND: LEFT - 10kOm, OK - 30kOm, RIGHT - 56kOm
#endif
#define FAN_SPEED1_PIN	9	// arduino Dx
#define FAN_SPEED2_PIN	10	// arduino Dx

// LCD ---------- rs, en, d4, d5, d6, d7
LiquidCrystal lcd( 6,  7,  2,  3,  4,  5);

// Датчики температуры аналоговые - NTC
#ifdef TEMP_TABLE_ADC_VALUES
// Для воздуха
// NTC, 100K, B25-3950, Таблица сопротивлений через 5° в Ом. (-20..95°), резистор 99.8k
const TEMP_TABLE NTC_table1[] PROGMEM = { 935, 908, 874, 835, 790, 739, 685, 628, 570, 513, 457, 404, 355, 311, 271, 235, 204, 177, 154, 133, 116, 101, 88, 76 };
#else
// NTC, 100K, B25-3950, Таблица сопротивлений через 5° в Ом. (-20..95°)
const TEMP_TABLE NTC_table1[] PROGMEM = { 1053847, 778981, 582457, 440260, 336206, 259246, 201746, 158371, 125353, 100000, 80371, 65055, 53015, 43481, 35882, 29784, 24862, 20864, 17598, 14917, 12703, 10867, 9336, 8054 };
#endif
#define NTC_table1S sizeof(NTC_table1) / sizeof(NTC_table1[0])

//#define NTC_table2 NTC_table1 // when the same
#ifndef NTC_table2
// NTC, 10K, B25-3950, Таблица сопротивлений через 5° в Ом. (-20..95°)
//const uint32_t NTC_table2[] PROGMEM = { 105385, 77898, 58246, 44026, 33621, 25925, 20175, 15837, 12535, 10000, 8037, 6506, 5301, 4348, 3588, 2978, 2486, 2086, 1760, 1492, 1270, 1087, 934, 805 };
#ifdef TEMP_TABLE_ADC_VALUES
// NTC, 10K, B25-3950, Таблица сопротивлений через 5° в Ом. (-20..95°), резистор 9.978k
const TEMP_TABLE NTC_table2[] PROGMEM = { 935, 908, 874, 835, 790, 739, 685, 628, 570, 513, 457, 404, 355, 311, 271, 235, 204, 177, 154, 133, 116, 101, 88, 76 };
// Thermistor Resistor NTC-MF52-103/3435 10K 3435+-1%, резистор 9.978k
//const TEMP_TABLE NTC_table2[] PROGMEM = { 907, 877, 842, 803, 760, 713, 664, 614, 563, 513, 464, 418, 375, 335, 298, 265, 236, 209, 185, 165, 146, 130, 116, 103 };
#else

// NTC, 10K, B25-3950, Таблица сопротивлений через 5° в Ом. (-20..95°)
const TEMP_TABLE NTC_table2[] PROGMEM = { 105385, 77898, 58246, 44026, 33621, 25925, 20175, 15837, 12535, 10000, 8037, 6506, 5301, 4348, 3588, 2978, 2486, 2086, 1760, 1492, 1270, 1087, 934, 805 };
// Thermistor Resistor NTC-MF52-103/3435 10K 3435+-1%
//const TEMP_TABLE NTC_table2[] PROGMEM = { 77523, 59606, 46290, 36290, 28704, 22897, 18410, 14916, 12171, 10000, 8269, 6881, 5759, 4847, 4101, 3488, 2981, 2559, 2207, 1912, 1662, 1451, 1272, 1118 };
#endif
#endif
#define NTC_table2S sizeof(NTC_table2) / sizeof(NTC_table2[0])
#define AtoMUX(a) ((1<<REFS0) | analogPinToChannel(a))

//
#define TAIR		0
#define TWATER		1
#define NTC_number	2 // Количество
//................................................    Воздух  ,    Подача
const uint16_t  NTC_AnalogMux[NTC_number]      = {  AtoMUX(A0),  AtoMUX(A1) };
const uint32_t  NTC_PullupResistor[NTC_number] = {      100000,       10000 };
const TEMP_TABLE *NTC_Table[NTC_number]        = {  NTC_table1,  NTC_table2 };
const uint8_t   NTC_TableSize[NTC_number]      = { NTC_table1S, NTC_table2S };
uint16_t NTC_adcval[NTC_number];
NTC ntc[NTC_number];

#define MAIN_LOOP_PERIOD			5		// msec
#define DISPLAY_REFRESH_PERIOD		2		// sec
#define DISPLAY_STARTUP_DELAY		5		// sec
#define KEY_TWICE_PRESS_TIMEOUT		3		// sec
#define KEY_FAST_PRESSING_TIME		1		// sec
#define KEY_RELEASE_TIMEOUT			200		// *100 msec
#define SAVE_SETTINGS_AFTER_TIME	15		// sec
#define ENTERING_SETUP_TIME			3		// sec, key - Ok
#define SETUP_TIMEOUT				60		// sec
#define FAN_SPEED_MAX				2		// 1..n
#define FAN_CHANGE_SPEED_DELAY		1000	// ms

enum MODE : uint8_t {
	Mode_Heat = 0,
	Mode_Cool,
	Mode_TOTAL
};

const int16_t   TempAirSetupMin[Mode_TOTAL]   = {  50, 150 }; // C, *TEMP_DECIMAL_DIVIDER
const int16_t   TempAirSetupMax[Mode_TOTAL]   = { 350, 300 };

#ifdef DEBUG_TO_SERIAL
#define DEBUG(s) Serial.print(s)
#define DEBUGN(s) Serial.println(s)
#else
#define DEBUG(s)
#define DEBUGN(s)
#endif

uint8_t ADC_Selector;
volatile uint8_t Keys_Pressed = 0;
uint8_t Keys_Pressed_Prev = 0;
#define KEY_UP		0b0001
#define KEY_OK		0b0010
#define KEY_DOWN	0b0100
volatile uint8_t Key_ADC = 0xFF;
uint8_t  fast_key_pressing = 0;
uint8_t  main_loop_countdown = 0;
uint8_t  save_settings_countdown = 0;
uint8_t  display_countdown;
uint8_t  fan_change_countdown = 0;
uint8_t  fan_speed = 0; // 0 - stopped, 1 - Speed 1, 2 - Speed 2
boolean  water_limit_stop = false;
int8_t   SetupMenu, SetupLevel;
uint8_t  main_loop_sec = 0;
boolean  warming_up_mode = false;

struct WORK {
	uint8_t Key_Up_ADC;
	uint8_t Key_Ok_ADC;
	uint8_t Key_Down_ADC;
	boolean	OnOff;					// 0 - Off, 1 - On
	MODE	mode;					// Текущий режим
	uint8_t fan_speed_max;
	int16_t target_air[Mode_TOTAL];
	int16_t water_limit[Mode_TOTAL];
	int16_t target_air_hysteresis;
	int16_t water_limit_hysteresis;
	int16_t fan_speed2_threshold;	// температурный порог
	int16_t temp_correct[2]; 		// корректировка температуры
	uint8_t fan_work_time_min; 		// sec
	uint8_t fan_pause_min; 			// sec
	int16_t warming_up_temp;
} work;

struct _EEPROM {
	WORK    work;
} __attribute__ ((packed));

struct _EEPROM EEMEM EEPROM;

enum {
	SetupMenu_Exit = 0,
	SetupMenu_WaterHeat,
	SetupMenu_WaterCool,
	SetupMenu_WarmingUpTemp,
	SetupMenu_HysteresisAir,
	SetupMenu_HysteresisWater,
	SetupMenu_TempCorrect1,
	SetupMenu_TempCorrect2,
	SetupMenu_FanSpeedMax,
	SetupMenu_FanSpeed2Threshold,
	SetupMenu_FanWorkTimeMin,
	SetupMenu_FanPauseMin,
	SetupMenu_FanTest,
	SetupMenu_TOTAL
};

// Called in delay()
void yield(void)
{
	sleep_cpu();
	wdt_reset();
}

ISR(ADC_vect)
{
	uint16_t adc = ADCL;
	adc += ADCH << 8;
	if(ADC_Selector < NTC_number) { // NTC
		NTC_adcval[ADC_Selector++] = adc;
		ADMUX = ADC_Selector == NTC_number ? KEYS_MUX : NTC_AnalogMux[ADC_Selector];
	} else { // Keys, to GND: LEFT - 10kOm, OK - 30kOm, RIGHT - 56kOm
		adc >>= 2;
		Key_ADC = adc;
		uint8_t keys;
		if((uint8_t)(uint8_t(adc) > work.Key_Up_ADC ? uint8_t(adc) - work.Key_Up_ADC : work.Key_Up_ADC - uint8_t(adc)) < 16) {
			keys = KEY_UP;
		} else if((uint8_t)(uint8_t(adc) > work.Key_Ok_ADC ? uint8_t(adc) - work.Key_Ok_ADC : work.Key_Ok_ADC - uint8_t(adc)) < 16) {
			keys = KEY_OK;
		} else if((uint8_t)(uint8_t(adc) > work.Key_Down_ADC ? uint8_t(adc) - work.Key_Down_ADC : work.Key_Down_ADC - uint8_t(adc)) < 16) {
			keys = KEY_DOWN;
		} else {
			keys = 0;
		}
		if(Keys_Pressed_Prev == keys) Keys_Pressed = keys;
		else Keys_Pressed_Prev = keys;
		ADMUX = NTC_AnalogMux[ADC_Selector = 0];
	}
	ADCSRA |= (1<<ADSC); // ADC Start conversion
}

void WaitKeysRelease(void)
{
	uint8_t tm = KEY_RELEASE_TIMEOUT;
	while(Keys_Pressed && --tm) delay(100);
	delay(100);
}

void WaitKeysFastPressing(void)
{
	if(!fast_key_pressing) {
		uint8_t tm = 0;
		while(Keys_Pressed) {
			delay(10);
			if(++tm > KEY_FAST_PRESSING_TIME * 100) {
				fast_key_pressing = 1;
				break;
			}
		}
	}
	delay(150);
}

void SaveKeyADC(const __FlashStringHelper *pstr, uint8_t *eprom)
{
	lcd.clear();
	lcd.print("Set keys, press:");
	lcd.setCursor(0, 1);
	while(Key_ADC <= 250) delay(100);
	lcd.print(pstr);
	while(Key_ADC > 250) delay(100);
	delay(200);
	eeprom_update_byte(eprom, Key_ADC);
	lcd.print(" = ");
	lcd.print(Key_ADC);
	delay(5000);
}

void lcd_print_temp(int16_t t)
{
	if(t < 0) {
		lcd.print('-');
		t = -t;
	}
	lcd.print(t / TEMP_DECIMAL_DIVIDER); lcd.print('.'); lcd.print(t % TEMP_DECIMAL_DIVIDER); // for %100 need different formula
}

void SetupDisplay()
{
	lcd.clear();
	lcd.print(F("Setup: "));
	switch (SetupMenu) {
	case SetupMenu_Exit: // Exit
		lcd.print(F("Exit"));
		break;
	case SetupMenu_WaterHeat: // Water Heat
		lcd.print(F("Heat"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Water Min"));
		break;
	case SetupMenu_WaterCool: // Water Cool
		lcd.print(F("Cool"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Water Max"));
		break;
	case SetupMenu_WarmingUpTemp:
		lcd.print(F("Warming"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Temp"));
		break;
	case SetupMenu_HysteresisAir: // Air hysteresis
		lcd.print(F("Hysteres."));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Target"));
		break;
	case SetupMenu_HysteresisWater: // Water hysteresis
		lcd.print(F("Hysteres."));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Water"));
		break;
	case SetupMenu_FanSpeedMax: // Fan speed max
		lcd.print(F("Fan"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Speed max"));
		break;
	case SetupMenu_FanSpeed2Threshold: // Fan speed 2 threshold
		lcd.print(F("FanSpeed2"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Threshold"));
		break;
	case SetupMenu_TempCorrect1:
		lcd.print(F("Correct"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Air t\xDF"));
		break;
	case SetupMenu_TempCorrect2:
		lcd.print(F("Correct"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Water t\xDF"));
		break;
	case SetupMenu_FanWorkTimeMin:
		lcd.print(F("Fan time"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Work"));
		break;
	case SetupMenu_FanPauseMin:
		lcd.print(F("Fan time"));
		lcd.setCursor(0, 1); // Second String
		lcd.print(F("Pause"));
		break;
	case SetupMenu_FanTest:
		lcd.print(F("Fan Test"));
		lcd.setCursor(0, 1); // Second String
		break;
	}
	if(SetupLevel) { // Edit item
		lcd.print(F(": "));
		switch (SetupMenu) {
		case SetupMenu_WaterHeat: // Water Heat
			lcd_print_temp(work.water_limit[Mode_Heat]);
			break;
		case SetupMenu_WaterCool: // Water Cool
			lcd_print_temp(work.water_limit[Mode_Cool]);
			break;
		case SetupMenu_WarmingUpTemp:
			lcd_print_temp(work.warming_up_temp);
			break;
		case SetupMenu_HysteresisAir: // Air hysteresis
			lcd_print_temp(work.target_air_hysteresis);
			break;
		case SetupMenu_HysteresisWater: // Water hysteresis
			lcd_print_temp(work.water_limit_hysteresis);
			break;
		case SetupMenu_FanSpeedMax: // Fan speed max
			lcd.print(work.fan_speed_max);
			break;
		case SetupMenu_FanSpeed2Threshold: // Fan speed 2 threshold
			lcd_print_temp(work.fan_speed2_threshold);
			break;
		case SetupMenu_TempCorrect1:
			lcd_print_temp(work.temp_correct[TAIR]);
			break;
		case SetupMenu_TempCorrect2:
			lcd_print_temp(work.temp_correct[TWATER]);
			break;
		case SetupMenu_FanWorkTimeMin:
			lcd.print(work.fan_work_time_min);
			break;
		case SetupMenu_FanPauseMin:
			lcd.print(work.fan_pause_min);
			break;
		case SetupMenu_FanTest:
			lcd.print(fan_speed);
			break;
		}
	}
}

void SetupSettings(void)
{
	uint8_t SetupTimeout = SETUP_TIMEOUT;
	SetupMenu = SetupLevel = 0;
	SetupDisplay();
	WaitKeysRelease();
	while(1) {
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		delay(MAIN_LOOP_PERIOD);
		if(--main_loop_sec == 0) { // 1 sec passed
			main_loop_sec = 1000 / MAIN_LOOP_PERIOD;
			if(--SetupTimeout == 0) break; // Timeout
			if(main_loop_countdown && --main_loop_countdown == 0) fast_key_pressing = 0;
		}
		if(Keys_Pressed & KEY_OK) {
			if(SetupLevel) {
				eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
				SetupLevel = 0;
			} else {
				if(SetupMenu == 0) break; // Exit
				SetupLevel = 1;
			}
			SetupDisplay();
			WaitKeysRelease();
			SetupTimeout = SETUP_TIMEOUT;
		} else if(Keys_Pressed & KEY_UP) {
			if(SetupLevel) {
				switch (SetupMenu) {
				case SetupMenu_WaterHeat: // Water Heat
					work.water_limit[Mode_Heat]++;
					break;
				case SetupMenu_WaterCool: // Water Cool
					work.water_limit[Mode_Cool]++;
					break;
				case SetupMenu_WarmingUpTemp:
					work.warming_up_temp++;
					break;
				case SetupMenu_HysteresisAir: // Air hysteresis
					work.target_air_hysteresis++;
					break;
				case SetupMenu_HysteresisWater: // Water hysteresis
					work.water_limit_hysteresis++;
					break;
				case SetupMenu_FanSpeedMax: // Fan speed max
					if(work.fan_speed_max < FAN_SPEED_MAX) work.fan_speed_max++;
					break;
				case SetupMenu_FanSpeed2Threshold: // Fan speed 2 threshold
					work.fan_speed2_threshold++;
					break;
				case SetupMenu_TempCorrect1:
					work.temp_correct[TAIR]++;
					break;
				case SetupMenu_TempCorrect2:
					work.temp_correct[TWATER]++;
					break;
				case SetupMenu_FanWorkTimeMin:
					work.fan_work_time_min++;
					break;
				case SetupMenu_FanPauseMin:
					work.fan_pause_min++;
					break;
				case SetupMenu_FanTest:
					SetFanSpeed(fan_speed + 1);
					break;
				}
				WaitKeysFastPressing();
			} else {
				if(++SetupMenu >= SetupMenu_TOTAL) SetupMenu = 0;
				WaitKeysRelease();
			}
			SetupDisplay();
			main_loop_countdown = KEY_FAST_PRESSING_TIME;
			SetupTimeout = SETUP_TIMEOUT;
		} else if(Keys_Pressed & KEY_DOWN) {
			if(SetupLevel) {
				switch (SetupMenu) {
				case SetupMenu_WaterHeat: // Water Heat
					work.water_limit[Mode_Heat]--;
					break;
				case SetupMenu_WaterCool: // Water Cool
					work.water_limit[Mode_Cool]--;
					break;
				case SetupMenu_WarmingUpTemp:
					work.warming_up_temp--;
					break;
				case SetupMenu_HysteresisAir: // Air hysteresis
					if(work.target_air_hysteresis > 0) work.target_air_hysteresis--;
					break;
				case SetupMenu_HysteresisWater: // Water hysteresis
					if(work.water_limit_hysteresis > 0) work.water_limit_hysteresis--;
					break;
				case SetupMenu_FanSpeedMax: // Fan speed max
					if(work.fan_speed_max > 0) work.fan_speed_max--;
					break;
				case SetupMenu_FanSpeed2Threshold: // Fan speed 2 threshold
					if(work.fan_speed2_threshold > 0) work.fan_speed2_threshold--;
					break;
				case SetupMenu_TempCorrect1:
					work.temp_correct[TAIR]--;
					break;
				case SetupMenu_TempCorrect2:
					work.temp_correct[TWATER]--;
					break;
				case SetupMenu_FanWorkTimeMin:
					work.fan_work_time_min--;
					break;
				case SetupMenu_FanPauseMin:
					work.fan_pause_min--;
					break;
				case SetupMenu_FanTest:
					SetFanSpeed(fan_speed - 1);
					break;
				}
				WaitKeysFastPressing();
			} else {
				if(--SetupMenu < 0) SetupMenu = SetupMenu_TOTAL - 1;
				WaitKeysRelease();
			}
			SetupDisplay();
			main_loop_countdown = KEY_FAST_PRESSING_TIME;
			SetupTimeout = SETUP_TIMEOUT;
		}
	}
	WaitKeysRelease();
}

// 0 - stop, 1 -
void SetFanSpeed(uint8_t speed)
{
	if(speed > work.fan_speed_max) speed = work.fan_speed_max;
	if(fan_speed != speed) {
		fan_speed = speed;
		fan_change_countdown = speed ? work.fan_work_time_min : work.fan_pause_min;
		if(speed == 1) {
			digitalWrite(FAN_SPEED2_PIN, 0);
			digitalWrite(FAN_SPEED1_PIN, 1);
		} else if(speed == 2) {
			digitalWrite(FAN_SPEED1_PIN, 0);
			digitalWrite(FAN_SPEED2_PIN, 1);
		} else {
			digitalWrite(FAN_SPEED1_PIN, 0);
			digitalWrite(FAN_SPEED2_PIN, 0);
		}
	}
}

void UpdateFan(void)
{
	if(!work.OnOff) {
		if(fan_speed) SetFanSpeed(0);
		fan_change_countdown = 0;
		return;
	}
	int16_t delta = ntc[TWATER].T - work.water_limit[work.mode];
	if(work.mode == Mode_Cool) delta = -delta;
	if(delta >= work.water_limit_hysteresis) {
		water_limit_stop = false;
	} else if(delta <= 0) {
		water_limit_stop = true;
	}
	int16_t t = ntc[TAIR].T;
	if(water_limit_stop && work.mode == Mode_Heat && t <= work.warming_up_temp) goto SetWarming; // режим разогрева холодного помещения
	if(warming_up_mode) {
SetWarming:
		warming_up_mode = ntc[TWATER].T > t + work.water_limit_hysteresis;
		water_limit_stop = water_limit_stop && !warming_up_mode;
	}
	if(fan_change_countdown) return;
	if(water_limit_stop) {
		SetFanSpeed(0);
		return;
	}
	delta = t - work.target_air[work.mode];
	if(work.mode == Mode_Cool) delta = -delta;
	if(delta >= 0) { // finish
		warming_up_mode = false;
		SetFanSpeed(0);
	} else if(-delta > work.target_air_hysteresis) {
		if(-delta > work.fan_speed2_threshold) { // Speed 2
			SetFanSpeed(2);
		} else {
			SetFanSpeed(1);
		}
	}
}

void RefreshDisplay(void)
{
	if(!display_countdown) {
		display_countdown = DISPLAY_REFRESH_PERIOD;
		lcd.setCursor(0, 0);
		if(!work.OnOff) {
			lcd.print(F("Off "));
		} else if(work.mode == Mode_Heat) {
			lcd.print(warming_up_mode ? F("Warm") : F("Heat"));
		} else if(work.mode == Mode_Cool) {
			lcd.print(F("Cool"));
		} else lcd.print(work.mode);
		lcd.print(':');
		int16_t t = ntc[TAIR].T;
		if(t >= 0) lcd.print(' ');
		lcd_print_temp(t);
		lcd.print('/');
		lcd_print_temp(work.target_air[work.mode]);
		lcd.print('\xDF'); // '°'
		lcd.setCursor(0, 1); // Second String
		if(work.OnOff) { // Show fan
			lcd.print('F');
			if(water_limit_stop && fan_speed == 0) lcd.print('-'); else lcd.print(fan_speed);
		} else {
			lcd.print(work.mode == Mode_Heat ? 'H' : 'C');
			lcd.print('.');
		}
		lcd.print(F(",W:"));
		t = ntc[TWATER].T;
		if(t >= 0) lcd.print(' ');
		lcd_print_temp(t);
		lcd.print(water_limit_stop ? '*' : '/');
		lcd_print_temp(work.water_limit[work.mode]);
		lcd.print('\xDF'); // '°'
	}
}

void setup()
{
#if defined(__AVR_ATmega8__)
	MCUCR |= (1<<SE); // Idle sleep enable
#else // ATmega48/P, ATmega88/P, ATmega168/P, ATmega328/P
	SMCR = (1<<SE); // Idle sleep enable
#endif
	wdt_enable(WDTO_2S); // Enable WDT
	// Setup keys
	KEYS_INIT;
	// Setup ADC, AVcc reference, first read keys
	ADC_Selector = 0;
	ADMUX = (1<<REFS0) | NTC_AnalogMux[ADC_Selector];
	ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC Interrupt Enable, ADC Start, ADC Prescaler: 128 (125000Hz)
	// Setup classes
#ifdef DEBUG_TO_SERIAL
	Serial.begin(250000);
#endif
	DEBUGN(F("Fancoil started."));
	lcd.begin(16, 2); // Setup: cols, rows
	lcd.print(F("Fancoil v"));
	lcd.print(VERSION);
	lcd.setCursor(0, 1);
	lcd.print(F("Vadim Kulakov(c)"));

	if(eeprom_read_byte((uint8_t*)&EEPROM.work.mode) == 255) { // init EEPROM
		memset(&work, 0, sizeof(work));
		work.target_air[Mode_Heat] = 240;
		work.target_air[Mode_Cool] = 200;
		work.water_limit[Mode_Heat] = 290;
		work.water_limit[Mode_Cool] = 190;
		work.warming_up_temp = 170;
		work.target_air_hysteresis = 10;
		work.water_limit_hysteresis = 15;
		work.fan_speed_max = FAN_SPEED_MAX;
		work.fan_speed2_threshold = 25;
		work.fan_work_time_min = 30;
		work.fan_pause_min = 60;
		eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
		goto SetKeys;
	}
	if(Key_ADC < 10) {		// Setup keys ADC values (if KEYS PIN connected to GND)
SetKeys:
		SaveKeyADC(F("1.Up"), &EEPROM.work.Key_Up_ADC);
		SaveKeyADC(F("2.Ok"), &EEPROM.work.Key_Ok_ADC);
		SaveKeyADC(F("3.Down"), &EEPROM.work.Key_Down_ADC);
	}
	Keys_Pressed = Keys_Pressed_Prev = 0;
	eeprom_read_block(&work, &EEPROM.work, sizeof(EEPROM.work));
	display_countdown = DISPLAY_STARTUP_DELAY;
	delay(1); // wait NTC samples
	for(uint8_t i = 0; i < NTC_number; i++) {
		ntc[i].init(NTC_adcval[i], NTC_Table[i], NTC_TableSize[i]);
	}
}

void loop()
{
	delay(MAIN_LOOP_PERIOD);
	for(uint8_t i = 0; i < NTC_number; i++) {
		uint16_t adc;
		ATOMIC_BLOCK(ATOMIC_FORCEON) adc = NTC_adcval[i];
		ntc[i].calc_temperature(adc);
		ntc[i].T += work.temp_correct[i];
	}
	if(--main_loop_sec == 0) { // 1 sec passed
		main_loop_sec = 1000 / MAIN_LOOP_PERIOD;
		if(display_countdown) display_countdown--;
		if(main_loop_countdown && --main_loop_countdown == 0) fast_key_pressing = 0;
		if(save_settings_countdown && --save_settings_countdown == 0) {
			lcd.clear();
			eeprom_update_block(&work, &EEPROM.work, sizeof(EEPROM.work));
			lcd.print(F("Settings saved."));
			display_countdown = DISPLAY_REFRESH_PERIOD;
		}
		if(fan_change_countdown) --fan_change_countdown;
		UpdateFan();
	}
	if(Keys_Pressed & KEY_OK) {
		//DEBUGN("OK");
		uint8_t tm = ENTERING_SETUP_TIME * 10;
		while(Keys_Pressed && --tm) delay(100);
		if(!tm) {
			SetupSettings();
			display_countdown = 0;
			save_settings_countdown = 0;
			main_loop_countdown = 0;
			return;
		} else {
			if(work.OnOff) {
				if(main_loop_countdown == 0) work.OnOff = false;
				else {
					work.mode = MODE(work.mode + 1);
					if(work.mode == Mode_TOTAL) work.mode = MODE(0);
				}
			} else work.OnOff = true;
			if(!work.OnOff) warming_up_mode = false;
		}
		WaitKeysRelease();
		display_countdown = 0;
		save_settings_countdown = SAVE_SETTINGS_AFTER_TIME;
		main_loop_countdown = KEY_TWICE_PRESS_TIMEOUT;
	} else if(Keys_Pressed & KEY_UP) {
		//DEBUGN("UP");
		if(work.OnOff) {
			if(work.target_air[work.mode] < TempAirSetupMax[work.mode]) work.target_air[work.mode]++;
			WaitKeysFastPressing();
		} else WaitKeysRelease();
		display_countdown = 0;
		save_settings_countdown = SAVE_SETTINGS_AFTER_TIME;
		main_loop_countdown = KEY_FAST_PRESSING_TIME;
	} else if(Keys_Pressed & KEY_DOWN) {
		//DEBUGN("DOWN");
		if(work.OnOff) {
			if(work.target_air[work.mode] > TempAirSetupMin[work.mode]) work.target_air[work.mode]--;
			WaitKeysFastPressing();
		} else WaitKeysRelease();
		display_countdown = 0;
		save_settings_countdown = SAVE_SETTINGS_AFTER_TIME;
		main_loop_countdown = KEY_FAST_PRESSING_TIME;
	}
	RefreshDisplay();
}


