/*
 Name:		dc_ups.ino
 Created:	2022-10-10
 Author:	ISS
*/



#include <OneWire.h>
#include <DallasTemperature.h>
#include "ups_adc.h"
#include "ups_util.h"
#include <stdint.h>


/*
// Settings

	wiva1.002		set input_voltage_adjust   [multiplier] (-10.0 ... 10.0)
	wbva1.0085		set battery_voltage_adjust [multiplier] (-10.0 ... 10.0)
	wova1.0			set output_voltage_adjust  [multiplier] (-10.0 ... 10.0)
	wcca1.05		set charge_current_adjust  [multiplier] (-10.0 ... 10.0)
	woca1.028		set output_current_adjust  [multiplier] (-10.0 ... 10.0)

	wbmv13.7		set battery_max_voltage [V] battery full voltage with charge (0.0 ... 15.0)
	wbfv12.7		set battery_full_voltage [V] battery full voltage without charge (0.0 ... 15.0)
	wblv11.7		set battery_low_voltage [V] battery empty voltage (0.0 ... 15.0)
	wbbv10.5		set battery_bad_voltage [V] battery too low voltage (0.0 ... 15.0)
	wbmc4.0			set battery_max_capacity [Ah] battary capacity (0.0 ... 9.0)
	wbcm1440		set battery_max_charge_mins [minutes] = 24h   max charge time (0 ... 4320)
	wbbm360			set battery_max_bad_mins [minutes] = 6h  max battery too low time (0 ... 1440)
	wblm720			set battery_max_low_mins [minutes] = 12h max battery low time (0 ... 1440)
	wbdt2022-01-10		set battery_date [yyyy-mm-dd]       battery change date (string, 10 chars)
	wobm120         	set on_battery_max_time_mins [minutes] = 2h  max run time on battery (0 ... 540)
	wmoc4.0			set max_output_current [A]   max output current (1.0 ... 5.0)
 
	wati1440		set auto_test_interval_mins [minutes] = 24h    automatic test interval (>= 0)
	watd0			set auto_test_duration_mins [minutes], 0 = 30seconds, >0 minutes   automatic test duration (0 ... 60)
	wtmi10			set measure_interval_secs [seconds]   temperature measure interval (0 ... 65535)
	wthl30.0		set temperature_high_limit [C degrees]   temperature high limit (25.0 ... 60.0)

	wfmi40			set fan_min_pwm [%]    fan min speed (0 ... 100)
	wfma80			set fan_max_pwm [%]    fan max speed (0 ... 100)

	wdbg1			set debug_override [0 = normal, 1 = debug messages on]    debug messages



*/


// Vakiot
#define SERIAL_SPEED					2400
#define FAN_PWM_TOP						320

#define INPUT_VOLTAGE_LOW				16.0
#define INPUT_VOLTAGE_OK				16.1
#define BATTERY_TEST_CURRENT			680.0
#define MAX_CHARGE_CURRENT				1000.0

#define CHARGE_CURRENT_FULL				20.0
#define CHARGE_CURRENT_STOP_FAN			200.0
#define CHARGE_CURRENT_START			200.0
#define CHARGE_CURRENT_END				100.0
#define CHARGE_TIME_START_FAN			30000UL
#define CHARGE_TIME_ADJUST_FAN			35000UL
#define TEST_TIME_START_FAN				30000UL		// probably a bit too long... maybe 20seconds
#define BATTERY_TEST_TIME_CONNECTED		30000UL
#define BATTERY_TEST_TIME_DISCONNECTED	600000UL

// Output state
#define OUTPUT_STATE_OFF				0
#define OUTPUT_STATE_ON_DELAY			1
#define OUTPUT_STATE_ON					2



// EEPROM
float    EEMEM ee_battery_max_voltage 			= 13.7;			// Max battery voltage with charge (V)	[EEPROM]
float    EEMEM ee_battery_full_voltage 			= 12.7;			// Max battery voltage without charge (V)	[EEPROM]
float    EEMEM ee_battery_low_voltage 			= 11.5;			// Battery low voltage (V)	[EEPROM]
float    EEMEM ee_battery_bad_voltage 			= 10.5;			// Battery dead voltage (V)	[EEPROM]
float    EEMEM ee_battery_max_capacity 			= 3.6;			// Max usable battery capacity (Ah)	[EEPROM]
float    EEMEM ee_max_output_current 			= 4;			// Max output current (A)	[EEPROM]
uint16_t EEMEM ee_auto_test_interval_mins 		= 1440;			// Automatic test (long) interval (minute)	[EEPROM]
uint8_t  EEMEM ee_auto_test_duration_mins 		= 0;			// Automatic test duration (minute), 0 => 10 sek, 255 => akku tyhjäksi	[EEPROM]
uint16_t EEMEM ee_temp_measure_interval_secs 	= 10;			// Temperature measure interval (second)	[EEPROM]
char     EEMEM ee_battery_date[11] 				= "2022-01-10";	// Battery install date (YYYY-MM-DD)	[EEPROM]
float    EEMEM ee_input_voltage_adjust 			= 0.0;			// Adjustment multiplier	[EEPROM]
float    EEMEM ee_battery_voltage_adjust 		= 0.0;			// Adjustment multiplier	[EEPROM]
float    EEMEM ee_output_voltage_adjust 		= 0.0;			// Adjustment multiplier	[EEPROM]
float    EEMEM ee_charge_current_adjust 		= 0.0;			// Adjustment multiplier	[EEPROM]
float    EEMEM ee_output_current_adjust 		= 0.0;			// Adjustment multiplier	[EEPROM]
uint8_t  EEMEM ee_debug_override 				= 1;			// Force debug mode	[EEPROM]
uint8_t  EEMEM ee_fan_min_pwm 					= 30;			// Fan min speed	[EEPROM]
uint8_t  EEMEM ee_fan_max_pwm 					= 100;			// Fan max speed	[EEPROM]
uint16_t EEMEM ee_battery_max_charge_mins		= 1440;			// Max charge duration (longer => battery dead)	[EEPROM]
uint16_t EEMEM ee_battery_max_low_mins 			= 360;			// Max battery low duration (longer => battery dead)	[EEPROM]
uint16_t EEMEM ee_battery_max_bad_mins 			= 720;			// Max battery too low duration (longer => battery dead)	[EEPROM]
uint16_t EEMEM ee_on_battery_max_time_mins 		= 120;			// Max run time on battery (2h). If longer utility loss => treat as battery low [EEPROM]
float    EEMEM ee_temperature_high_limit 		= 30.0;			// Temperature limit, start fan if higher	[EEPROM]




// arvot
float    battery_max_voltage        = 0.0;
float    battery_full_voltage       = 0.0;
float    battery_low_voltage        = 0.0;
float    battery_bad_voltage        = 0.0;
float    battery_max_capacity       = 0.0;
float    max_output_current         = 0.0;
uint16_t auto_test_interval_mins    = 0;
uint8_t  auto_test_duration_mins    = 0;
uint16_t temp_measure_interval_secs = 0;
char     battery_date[11]           = "";
float    input_voltage_adjust       = 0.0;
float    battery_voltage_adjust     = 0.0;
float    output_voltage_adjust      = 0.0;
float    charge_current_adjust      = 0.0;
float    output_current_adjust      = 0.0;
uint8_t  debug_override             = 0;
uint8_t  fan_min_pwm                = 0;
uint8_t  fan_max_pwm                = 0;
uint16_t battery_max_charge_mins    = 0;
uint16_t battery_max_low_mins       = 0;
uint16_t battery_max_bad_mins       = 0;
uint16_t on_battery_max_time_mins   = 0;
float    temperature_high_limit     = 0.0;


uint8_t fan_pwm = 0;			// Fan speed
char buf[16];					// Temporary buffer



// tila
uint8_t is_debug_mode    = 0;	// Debug mode
uint8_t output_state     = 0;	// Output state
uint8_t source_on        = 0;	// Utility on
uint8_t charge_on        = 0;	// Charge on
uint8_t battery_on       = 0;	// Battery on
uint8_t output_on        = 0;	// Output on
uint8_t fan_on           = 0;	// Fan output state
uint8_t fan_on_test      = 0;	// Fan on during test
uint8_t fan_on_charge    = 0;	// Fan on during charge
uint8_t fan_on_temp      = 0;	// Fan on with temperature
uint8_t battery_fail     = 0;	// Battery dead
uint8_t utility_fail     = 0;	// Q1 status
uint8_t battery_low      = 0;	// Q1 status
uint8_t battery_bad      = 0;	// Battery too low
uint8_t ups_fail         = 0;	// Q1 status
uint8_t test_in_progress = 0;	// Q1 status
uint8_t shutdown_active  = 0;	// Q1 status



// mittaukset
float input_voltage   = 0.0;	// Input voltage
float battery_voltage = 0.0;	// Battery voltage
float charge_current  = 0.0;	// Charge current
float output_voltage  = 0.0;	// Output voltage
float output_current  = 0.0;	// Output current
float temperature     = 0.0;	// Temperature

// ajat
uint16_t battery_runtime_mins   = 0.0;			// Run time on battery (minutes)
uint8_t  battery_charge_percent = 0;			// Battery charge level (%)
uint16_t startup_delay_secs     = 10;			// Startup delay (seconds)
uint16_t shutdown_delay_secs    = 0;			// Shutdown delay (seconds)

unsigned long startup_start_time        = 0;	// Start timestamp
unsigned long shutdown_start_time       = 0;	// Shutdown start timestamp
unsigned long last_short_auto_test_time = 0;	// Last automatic test (short) timestamp
unsigned long last_long_auto_test_time  = 0;	// Last automatic test (long) timestamp
unsigned long last_measure_time         = 0;	// Last temperature measurement timestamp


unsigned long test_start_time           = 0;	// Test start timestamp
uint16_t      test_run_time_secs        = 0;	// Test time (seconds)

unsigned long battery_charge_start_time = 0;	// Charge start timestamp
uint16_t      battery_charge_time_mins  = 0;	// Charge suration (minutes)

unsigned long battery_low_start_time    = 0;	// Battery low start timestamp
uint16_t      battery_low_time_mins     = 0;	// Battery low duration (minutes)

unsigned long battery_bad_start_time    = 0;	// Battery too low start timestamp
uint16_t      battery_bad_time_mins     = 0;	// Battery too low duration (minutes)

unsigned long on_battery_start_time     = 0;	// On battery start timestamp
uint16_t      on_battery_time_mins      = 0;	// On battery duration (minutes)

unsigned long last_battery_check_time   = 0;	// Last battery check timestamp





// PB5 (D13) => OUT ON
#define OUTPUT_ON()     (PORTB |= (1 << PB5))
#define OUTPUT_OFF()	(PORTB &= ~(1 << PB5))

// PB4 (D12)

// PB3 (D11)

// PB2 (D10) => FAN PWR
#define FAN_ON()		(PORTB |= (1 << PB2));  fan_on = 1
#define FAN_OFF()		(PORTB &= ~(1 << PB2)); fan_on = 0

// PB1 (D9) => FAN PWM (OC1A)

// PB0 (D8) => SETUP MODE INPUT
#define IS_DEBUG_INPUT()	((PINB & (1 << PB0)) > 0 ? 0 : 1)

// PD7 (D7)

// PD6 (D6) => SOURCE OFF
#define SOURCE_ON()     (PORTD &= ~(1 << PD6)); source_on = 1
#define SOURCE_OFF()	(PORTD |= (1 << PD6));  source_on = 0

// PD5 (D5) => 1-WIRE ?
#define  ow_pin  5

// PD4 (D4) => TESTLOAD ON
#define TESTLOAD_ON()   (PORTD |= (1 << PD4))
#define TESTLOAD_OFF()	(PORTD &= ~(1 << PD4))

// PD3 (D3) => BATT ON
#define BATTERY_ON()    (PORTD |= (1 << PD3));  battery_on = 1
#define BATTERY_OFF()	(PORTD &= ~(1 << PD3)); battery_on = 0

// PD2 (D2 ) => CHARGE ON
#define CHARGE_ON()     (PORTD |= (1 << PD2));  charge_on = 1
#define CHARGE_OFF()	(PORTD &= ~(1 << PD2)); charge_on = 0




// Temperature, 1 kpl DS18B20
OneWire ds = OneWire(ow_pin);
DallasTemperature sensor = DallasTemperature(&ds);

void measure_temperature() {
	DeviceAddress ow_device;
	// Init 1-wire system, resolution = 12bit, wait for conversion
	sensor.begin();
	sensor.setResolution(TEMP_12_BIT);
	sensor.setWaitForConversion(TRUE);
	// Send measure commad for all sensors
	sensor.requestTemperatures();
	if (sensor.getDeviceCount() > 0) {
		sensor.getAddress(ow_device, 0);
		temperature = sensor.getTempC(ow_device);
	}
}




// Fan speed set %,  0 .. 100

uint8_t set_fan_pwm(uint8_t percent, uint8_t ident) {
	if (percent < fan_min_pwm) {
		percent = 0;
	} else if (percent > fan_max_pwm) {
		percent = fan_max_pwm;
	}
	//fan_pwm = percent;
	if (is_debug_mode > 0) {
		Serial.print(F("f pwm ")); Serial.print(ident); Serial.print(F(" => ")); Serial.println(percent);
	}
	float v = ((float)FAN_PWM_TOP * ((float)percent / 100.0));
	if (v > (float)FAN_PWM_TOP) {
		v = FAN_PWM_TOP;
	}
	if (v < 0) v = 0;
	OCR1A = (uint16_t)v;
	return(percent);
}



// Measure voltages and currents
void measure_values() {
	battery_voltage = ((float)readADC(ADC_V_BATT) / 1024.0) * 20.0 * battery_voltage_adjust;
	charge_current =  ((float)readADC(ADC_A_CGH)  / 1024.0) * 1050.0 * charge_current_adjust;
	input_voltage =   ((float)readADC(ADC_V_IN)   / 1024.0) * 25.0 * input_voltage_adjust;
	output_voltage =  ((float)readADC(ADC_V_OUT)  / 1024.0) * 25.0 * output_voltage_adjust;
	output_current =  ((float)readADC(ADC_A_OUT)  / 1024.0) * 5000.0 * output_current_adjust;
}





// Start test
void start_test() {
	// 1. Charge off
	CHARGE_OFF();
	// 2. If battery is disconnected, connect
	if (battery_on == 0) {
		BATTERY_ON();
	}
	// 3. delay for relay connect and voltage stabilization
	delay(1000);
	// 4. measure
	measure_values();

	if (is_debug_mode > 0) {
		Serial.print(F("start test BV = ")); Serial.println(battery_voltage, 2);
	}
	// Battery too low, no need to test -> disconnect & fail
	if (battery_voltage < battery_bad_voltage) {
		// 5. Too low -> battery dead. Disconnect
		if (is_debug_mode > 0) {
			Serial.println(F("start test BV B -> fail"));
		}
		battery_bad = 1;
		battery_low = 1;
		battery_fail = 1;
		ups_fail = 1;
		BATTERY_OFF();
	} else {
		// 6. Battery voltage > bad -> run test
		test_in_progress = 1;
		test_start_time = millis();
		TESTLOAD_ON();
	}
}



// Stop test
void stop_test() {
	TESTLOAD_OFF();
	test_in_progress = 0;
	test_start_time = 0;
	if (fan_on_test > 0) {
		fan_on_test = 0;
		fan_pwm = set_fan_pwm(fan_min_pwm, 103);
	}
	CHARGE_ON();
}

// Start shutdown
void start_shutdown() {
	shutdown_active = 1;
	shutdown_start_time = millis();
}

// Cancel shutdown
void cancel_shutdown() {
	shutdown_active = 0; 
	shutdown_start_time = 0;
}

// Output off
void do_shutdown() {
	cancel_shutdown();
	OUTPUT_OFF();
	output_state = OUTPUT_STATE_OFF;
	FAN_OFF();
	fan_pwm = set_fan_pwm(fan_min_pwm, 200);
	fan_on_test = 0;
	fan_on_temp = 0;
	fan_on_charge = 0;
}

// Output on
void do_startup() {
	cancel_shutdown(); 
	stop_test();
	OUTPUT_ON();
	output_state = OUTPUT_STATE_ON;
	startup_start_time = 0;
	fan_pwm = set_fan_pwm(fan_min_pwm, 201);
	FAN_ON();
	fan_on_test = 0;
	fan_on_temp = 0;
	fan_on_charge = 0;
}










// Q1 status message
void send_status() {
	// NOTE: 100 * current(mA) / 1000 => current(mA) / 10
	uint8_t output_load = (uint8_t)round((output_current / 10.0) / max_output_current);
	Serial.print('(');
	Serial.print(format_float3(buf, input_voltage));   Serial.print(' ');	// input voltage
	Serial.print(buf);                                 Serial.print(' ');	// fault voltage
	Serial.print(format_float3(buf, output_voltage));  Serial.print(' ');	// output voltage
	Serial.print(format_uint3(buf, output_load));      Serial.print(' ');	// output load %
	Serial.print("00.0");                              Serial.print(' ');	// input frequency
	Serial.print(format_float2(buf, battery_voltage)); Serial.print(' ');	// battery voltage
	Serial.print(format_float2(buf, temperature));     Serial.print(' ');	// temperature
	Serial.print(utility_fail > 0 ? '1' : '0');        // Utility fail
	Serial.print(battery_low > 0 ? '1' : '0');         // Battery low
	Serial.print('0');                                 // bypass/boost or buck active
	Serial.print(ups_fail > 0 ? '1' : '0');            // Ups failed
	Serial.print('1');                                 // Ups type (0 = online, 1 = standby)
	Serial.print(test_in_progress > 0 ? '1' : '0');    // Test in progress
	Serial.print(shutdown_active > 0 ? '1' : '0');     // Shutdown active
	Serial.print('0');                                 // Beeper on
	Serial.print('\r');
	if (is_debug_mode > 0) Serial.print('\n');
}




// Debug: values
void print_values() {
	Serial.print(F("in V    = ")); Serial.println(input_voltage, 2);
	Serial.print(F("batt V  = ")); Serial.println(battery_voltage, 2);
	Serial.print(F("out V   = ")); Serial.println(output_voltage, 2);
	Serial.print(F("chg mA  = ")); Serial.println(charge_current, 2);
	Serial.print(F("out mA  = ")); Serial.println(output_current, 2);
	Serial.print(F("temp C  = ")); Serial.println(temperature, 2);
	Serial.print(F("batt %  = ")); Serial.println(battery_charge_percent);
	Serial.print(F("batt rt = ")); Serial.println(battery_runtime_mins);
	Serial.print(F("b run m = ")); Serial.println(on_battery_time_mins);
	Serial.print(F("b chg m = ")); Serial.println(battery_charge_time_mins);
	Serial.print(F("b low m = ")); Serial.println(battery_low_time_mins);
}


// Debug: state
void print_state() {
	Serial.print(F("utl f  = ")); Serial.println(utility_fail);
	Serial.print(F("fan on = ")); Serial.println(fan_on);
	Serial.print(F("fan %  = ")); Serial.println(fan_pwm);
	Serial.print(F("batt o = ")); Serial.println(battery_on);
	Serial.print(F("batt l = ")); Serial.println(battery_low);
	Serial.print(F("ups f  = ")); Serial.println(ups_fail);
	Serial.print(F("test a = ")); Serial.println(test_in_progress);
	if (test_in_progress > 0) {
		Serial.print(F("test time = ")); Serial.println((millis() - test_start_time) / 1000);
	}
	Serial.print(F("shdn a = ")); Serial.println(shutdown_active);
	if (shutdown_active > 0) {
		Serial.print(F("shdn s = ")); Serial.println((millis() - shutdown_start_time) / 1000);
	}
	if (output_state == OUTPUT_STATE_ON) {
		Serial.println(F("out on"));
	} else if (output_state == OUTPUT_STATE_ON_DELAY) {
		Serial.println(F("out on delay"));
		Serial.print(F("out on s = ")); Serial.println((millis() - startup_start_time) / 1000);
	} else if (output_state == OUTPUT_STATE_OFF) {
		Serial.println(F("out off"));
	} else {
		Serial.println(F("unknown state"));
	}
}



// Debug: settings
void print_settings() {
	Serial.print(F("b max V   = ")); Serial.println(battery_max_voltage, 2);
	Serial.print(F("b full V  = ")); Serial.println(battery_full_voltage, 2);
	Serial.print(F("b low V   = ")); Serial.println(battery_low_voltage, 2);
	Serial.print(F("b bad V   = ")); Serial.println(battery_bad_voltage, 2);
	Serial.print(F("b max Ah  = ")); Serial.println(battery_max_capacity, 1);
	Serial.print(F("max out A = ")); Serial.println(max_output_current, 1);
	Serial.print(F("a tst int = ")); Serial.println(auto_test_interval_mins);
	Serial.print(F("a tst dur = ")); Serial.println(auto_test_duration_mins);
	Serial.print(F("m int s   = ")); Serial.println(temp_measure_interval_secs);
	Serial.print(F("b date    = ")); Serial.println(battery_date);
	Serial.print(F("in V adj  = ")); Serial.println(input_voltage_adjust, 6);
	Serial.print(F("b V adj   = ")); Serial.println(battery_voltage_adjust, 6);
	Serial.print(F("out V adj = ")); Serial.println(output_voltage_adjust, 6);
	Serial.print(F("chg A adj = ")); Serial.println(charge_current_adjust, 6);
	Serial.print(F("out A adj = ")); Serial.println(output_current_adjust, 6);
	Serial.print(F("dbg force = ")); Serial.println(debug_override);
	Serial.print(F("fan min % = ")); Serial.println(fan_min_pwm);
	Serial.print(F("fan max % = ")); Serial.println(fan_max_pwm);
	Serial.print(F("b max chg = ")); Serial.println(battery_max_charge_mins);
	Serial.print(F("b max low = ")); Serial.println(battery_max_low_mins);
	Serial.print(F("b max min = ")); Serial.println(on_battery_max_time_mins);
	Serial.print(F("temp high = ")); Serial.println(temperature_high_limit);
}











// Setup
void setup() {

	// Outputs
	// PB5 (D13) => OUT ON
	DDRB |= (1 << DDB5);
	// PB4 (D12)
	// PB3 (D11)
	// PB2 (D10) => FAN PWR
	DDRB |= (1 << DDB2);
	// PB1 (D9) => FAN PWM (OC1A)
	DDRB |= (1 << DDB1);
	// PB0 (D8) <= SETUP MODE INPUT, ylösveto
	PORTB |= (1 << PB0);

	// PD7 (D7)
	// PD6 (D6) => SOURCE OFF
	DDRD |= (1 << DDD6);
	// PD5 (D5) => 1-WIRE ?
	// PD4 (D4) => TESTLOAD ON
	DDRD |= (1 << DDD4);
	// PD3 (D3) => BATT ON
	DDRD |= (1 << DDD3);
	// PD2 (D2 ) => CHARGE ON
	DDRD |= (1 << DDD2);


	// Initial off
	TESTLOAD_OFF();
	SOURCE_ON();
	FAN_OFF();
	fan_on_test = 0;
	fan_on_temp = 0;
	fan_on_charge = 0;

	Serial.begin(2400);

	// Setup ADC
	initADC();
	sei();

	// Read EEPROM settings
	battery_max_voltage = eeprom_read_float(&ee_battery_max_voltage);
	battery_full_voltage = eeprom_read_float(&ee_battery_full_voltage);
	battery_low_voltage = eeprom_read_float(&ee_battery_low_voltage);
	battery_bad_voltage = eeprom_read_float(&ee_battery_bad_voltage);
	battery_max_capacity = eeprom_read_float(&ee_battery_max_capacity);
	max_output_current = eeprom_read_float(&ee_max_output_current);
	auto_test_interval_mins = eeprom_read_word(&ee_auto_test_interval_mins);
	auto_test_duration_mins = eeprom_read_byte(&ee_auto_test_duration_mins);
	temp_measure_interval_secs = eeprom_read_word(&ee_temp_measure_interval_secs);
	eeprom_read_block(&battery_date, &ee_battery_date, 10);
	battery_date[10] = '\0';
	input_voltage_adjust = eeprom_read_float(&ee_input_voltage_adjust);
	battery_voltage_adjust = eeprom_read_float(&ee_battery_voltage_adjust);
	output_voltage_adjust = eeprom_read_float(&ee_output_voltage_adjust);
	charge_current_adjust = eeprom_read_float(&ee_charge_current_adjust);
	output_current_adjust = eeprom_read_float(&ee_output_current_adjust);
	debug_override = eeprom_read_byte(&ee_debug_override);
	fan_min_pwm = eeprom_read_byte(&ee_fan_min_pwm);
	fan_max_pwm = eeprom_read_byte(&ee_fan_max_pwm);
	battery_max_charge_mins = eeprom_read_word(&ee_battery_max_charge_mins);
	battery_max_bad_mins = eeprom_read_word(&ee_battery_max_bad_mins);
	battery_max_low_mins = eeprom_read_word(&ee_battery_max_low_mins);
	on_battery_max_time_mins = eeprom_read_word(&ee_on_battery_max_time_mins);
	temperature_high_limit = eeprom_read_float(&ee_temperature_high_limit);

	// Debug mode ?
	is_debug_mode = IS_DEBUG_INPUT();
	if (debug_override > 0) is_debug_mode = 1;

	// Connect battery
	BATTERY_ON();

	startup_delay_secs = 10;
	output_state = OUTPUT_STATE_ON_DELAY;
	ups_fail = 0;
	utility_fail = 0;

	// Delay before measure
	delay(500);

	measure_values();


	// Source down ?
	if (input_voltage < INPUT_VOLTAGE_LOW) {
		utility_fail = 1;
		on_battery_start_time = millis();
		if (is_debug_mode > 0) {
			Serial.println(F("setup -> U fail"));
		}
	}

	// Battery voltage check
	if (battery_voltage < battery_bad_voltage) {
		if (is_debug_mode > 0) {
			Serial.println(F("setup -> B fail"));
		}
		BATTERY_OFF();
	}

	// Source on & battery connected -> start charge
	if ((utility_fail == 0) && (battery_on == 1)) {
		CHARGE_ON();
		if (is_debug_mode > 0) {
			Serial.println(F("setup -> B on"));
		}
	}

	// Fan pwm setup
	ICR1 = FAN_PWM_TOP;						// TOP = frequency, 25kHz @16MHz
	OCR1A = 0;
	TCCR1A = (1 << COM1A1) | (1 << COM1A0);	// Inverted pwm -> pull down driver 
	TCCR1B = (1 << WGM13) | (1 << CS10);	// Phase and frequency correct PWM, mode 8

	fan_pwm = set_fan_pwm(fan_min_pwm, 100);

	if (is_debug_mode > 0) {
		Serial.println(F("---"));
		print_settings();
		Serial.println(F("---"));
		print_state();
		Serial.println(F("---"));
		print_values();
		Serial.println(F("---"));
		Serial.println(F("setup done"));
	}
}







void loop() {

	unsigned long t = 0;
	double temp_double = 0.0;
	int temp_int = 0;
	long temp_long = 0;

	// Debug mode ?
	is_debug_mode = IS_DEBUG_INPUT();
	if (debug_override == 1) {
		is_debug_mode = 1;
	}

	// Measure
	measure_values();

	// Source
	if (input_voltage < INPUT_VOLTAGE_LOW) {

		// Input voltage low -> stop test, stop charge, start on battery timer

		utility_fail = 1;
		stop_test();	// For safety
		CHARGE_OFF();
		if (on_battery_start_time == 0) {
			on_battery_start_time = millis();
			if (is_debug_mode > 0) {
				Serial.println(F("VIN L -> utl f = 1"));
			}
		} else {
			// Battery run time
			on_battery_time_mins = (millis() - on_battery_start_time) / 60000;
		}
	} else if (input_voltage >= INPUT_VOLTAGE_OK) {

		// Input voltage ok -> clear on battery timer

		if (utility_fail > 0) {
			if (is_debug_mode > 0) {
				Serial.println(F("VIN OK -> utl f = 0"));
			}
			utility_fail = 0;
			CHARGE_ON();
			on_battery_start_time = 0;
			on_battery_time_mins = 0;
		}
	} // Source




	// Source ok
	if (utility_fail == 0) {

		// On startup delay ?
		if (output_state == OUTPUT_STATE_ON_DELAY) {
			if ((millis() - startup_start_time) / 1000 > startup_delay_secs) {
				// Delay done -> startup
				if (is_debug_mode > 0) Serial.println(F("=> START"));
				do_startup();
				delay(200);
			}
		}
		// Shut down
		if (output_state == OUTPUT_STATE_OFF) {
			// Start after normal delay
			output_state = OUTPUT_STATE_ON_DELAY;
			startup_start_time = millis();
			if (is_debug_mode > 0) Serial.println(F("=> START DLY"));
		}
	} else {
		// Source off

		// Shut down or startup delay -> shut down and save battery
		// Next startup when source is back on
		if ((output_state == OUTPUT_STATE_OFF) || (output_state == OUTPUT_STATE_ON_DELAY)) {
			if (is_debug_mode > 0) Serial.println(F("UTL F + SHDN|DLY -> B OFF"));
			output_state = OUTPUT_STATE_OFF;
			BATTERY_OFF();
			delay(1000);
		}

	}


	// Shutdown in progress, does not depend on source
	if (shutdown_active > 0) {
		if ((millis() - shutdown_start_time) / 1000 > shutdown_delay_secs) {
			if (is_debug_mode > 0) Serial.println(F("=> SHDN"));
			do_shutdown();
			delay(100);
		}
	}




	// Battery connected, check
	//if ((battery_on == 1) && (battery_connected == 1)) {
	if (battery_on == 1) {

		// Too low ?
		if (battery_voltage < battery_bad_voltage) {
			// Voltage < bad => battery is dead if long enough time spent
			if (test_in_progress > 0) {
				// Too low during test -> battery dead & fail
				if (fan_on_test > 0) {
					fan_on_test = 0;
					fan_pwm = set_fan_pwm(fan_min_pwm, 102);
				}
				stop_test();
				CHARGE_OFF();
				BATTERY_OFF();
				battery_low = 1;
				battery_bad = 1;
				battery_fail = 1;
				ups_fail = 1;
				if (is_debug_mode > 0) {
					Serial.println(F("in test VB B -> stop test + fail"));
				}
			} else {
				// During normal operation (not during test)
				if (battery_bad == 0) {
					battery_low = 1;
					battery_bad = 1;
					ups_fail = 1;
					battery_bad_start_time = millis();	// Save start timestamp
					// NOTE: mark battery bad after delay, not on first measurement
					if (is_debug_mode > 0) {
						Serial.println(F("VB B -> bad"));
					}
				} else {
					// Battery too low duration
					battery_bad_time_mins = (millis() - battery_bad_start_time) / 60000;

					if (battery_bad_time_mins > battery_max_bad_mins) {
						// Battery too low too long -> dead
						// Stop charge and disconnect battery
						battery_fail = 1;
						ups_fail = 1;
						CHARGE_OFF();
						BATTERY_OFF();
						if (is_debug_mode > 0) {
							Serial.println(F("VB B time -> fail"));
						}
					}

				}

			}
		}

		// 7.1. Battery low
		else if (battery_voltage < battery_low_voltage) {
			// Voltage between bad ... low -> battery is dead if too long

			// Voltage was < bad and rise to bad ... low -> remove bad flag
			if (battery_bad == 1) {
				battery_bad = 0;
			}

			if (test_in_progress > 0) {
				// Stop test if running. Do not mark bad, leave connected
				stop_test();
				battery_low = 1;
				if (is_debug_mode > 0) {
					Serial.println(F("in test VB L => stop test + low"));
				}
			} else {
				// During normal operation (not during test)
				if (battery_low == 0) {
					battery_low = 1;
					battery_low_start_time = millis();
					if (is_debug_mode > 0) {
						Serial.println(F("VB L -> low"));
					}
				} else {
					// Battery low duration
					battery_low_time_mins = (millis() - battery_low_start_time) / 60000;

					if (battery_low_time_mins > battery_max_low_mins) {
						// Battery low too long -> dead
						// Stop charge and disconnect battery
						battery_low_time_mins = 0;
						battery_fail = 1;
						ups_fail = 1;
						CHARGE_OFF();
						BATTERY_OFF();
						if (is_debug_mode > 0) {
							Serial.println(F("VB L time -> fail"));
						}
					}
				}

			}
		} else {
			// Battery ok, restore
			if (ups_fail == 1) {
				ups_fail = 0;
			}
			if (battery_fail == 1) {
				battery_fail = 0;
			}

			if ((battery_low == 1) || (battery_bad == 1)) {
				battery_bad = 0;
				battery_bad_start_time = 0;
				battery_bad_time_mins = 0;
				battery_low = 0;
				battery_low_start_time = 0;
				battery_low_time_mins = 0;
				if (is_debug_mode > 0) {
					Serial.println(F("VB OK"));
				}
			}

		}


		// Charge
		if (utility_fail == 0) {
			// Only if source ok

			// Only if not testing
			if (test_in_progress == 0) {

				// Charge check
				// Charge start if current > 200mA

				if (charge_current > CHARGE_CURRENT_START) {
					FAN_ON();
					t = millis() - battery_charge_start_time;

					if (battery_charge_start_time == 0) {
						battery_charge_start_time = millis();
						if (is_debug_mode > 0) {
							Serial.println(F("B chg start"));
						}
					} else {

						//t = millis() - battery_charge_start_time;
						battery_charge_time_mins = t / 60000UL;

						// 5 sec fan full, after that adjust with charge current
						//if ((millis() - battery_charge_start_time) > CHARGE_TIME_ADJUST_FAN) {
						if (t > CHARGE_TIME_ADJUST_FAN) {
							if (fan_on_charge == 1) {
								uint8_t f = fan_min_pwm + (uint8_t)((charge_current / MAX_CHARGE_CURRENT) * (float)(fan_max_pwm - fan_min_pwm));
								if (abs(f - fan_pwm) > 2) {
									if (is_debug_mode > 0) {
										Serial.print(F("B chg -> fan set ")); Serial.println(f);
									}
									fan_pwm = set_fan_pwm(f, 105);
								}
							}

						} else if (t > CHARGE_TIME_START_FAN) {
							// Charge duration > 30sec -> start fan
							if (fan_on_charge == 0) {
								fan_on_charge = 1;
								fan_pwm = set_fan_pwm(fan_max_pwm, 104);
								if (is_debug_mode > 0) {
									Serial.print(F("B chg >30sek -> fan start ")); Serial.println(fan_max_pwm);
								}
							}

						}

					}

				}

				// Charge end when current < 100mA
				else if (charge_current < CHARGE_CURRENT_END) {
					if (battery_charge_start_time > 0) {
						battery_charge_start_time = 0;
						if (is_debug_mode > 0) {
							Serial.println(F("B chg end -> stby"));
						}
						battery_charge_time_mins = 0;
					}
				} else {
					// Charge between 100mA ... 200mA -> stop fan when < 200mA
					if (test_in_progress == 0) {
						// Only if not testing (test controls the fan). Just to be sure
						if (fan_on_charge == 1) {
							fan_on_charge = 0;
							fan_pwm = set_fan_pwm(fan_min_pwm, 106);
							if (is_debug_mode > 0) {
								Serial.println(F("B chg < 200 -> fan min"));
							}
						}

					}
				}


				// Maximum charge time reached ?
				// yes -> battery is dead. Disconnect
				if (battery_charge_time_mins >= battery_max_charge_mins) {
					battery_charge_start_time = 0;
					battery_charge_time_mins = 0;
					ups_fail = 1;
					battery_fail = 1;
					CHARGE_OFF();
					BATTERY_OFF();

					if (is_debug_mode > 0) {
						Serial.println(F("B chg t -> b fail"));
					}

					if (fan_on_charge > 0) {
						fan_on_charge = 0;
						fan_pwm = set_fan_pwm(fan_min_pwm, 107);
						if (is_debug_mode > 0) {
							Serial.println(F("B chg t -> fan min"));
						}
					}

				}


				// Automatic test
				// Start only if not running and battery is ok
				if ((battery_on == 1) && (battery_fail == 0)) {
					// Time to start ?
					if ((millis() - last_long_auto_test_time) > (unsigned long)auto_test_interval_mins * 60000UL) {
						// Start timestamp
						last_long_auto_test_time = millis();
						// Duration = 0 min -> test 30 sec
						if (auto_test_duration_mins == 0) {
							test_run_time_secs = 30;
						} else {
							// Duration >= 1 min
							test_run_time_secs = auto_test_duration_mins * 60;
						}

						if (is_debug_mode > 0) {
							Serial.print(F("auto T start: ")); Serial.println(test_run_time_secs);
						}
						// Run as normal test
						start_test();
					}
				}
			} else {

				// Test in progress
				t = millis() - test_start_time;

				if (t > TEST_TIME_START_FAN) {
					FAN_ON();
					if (fan_on_test < 1) {
						if (fan_max_pwm != fan_pwm) {
							fan_pwm = set_fan_pwm(fan_max_pwm, 108);
							if (is_debug_mode > 0) {
								Serial.print(F("test > 30sek -> fan start ")); Serial.println(fan_max_pwm);
							}
						}
					}

				}

				// Test duration reached
				if (t > (unsigned long)test_run_time_secs * 1000) {
					if (is_debug_mode > 0) {
						Serial.println(F("T time => stop test"));
					}
					// Stop test & fan
					stop_test();

					if (fan_on_test > 0) {
						fan_on_test = 0;
						fan_pwm = set_fan_pwm(fan_min_pwm, 109);
						if (is_debug_mode > 0) {
							Serial.println(F("T time -> fan min"));
						}

					}

				}

			}

		}
	}


	// Battery connected, calculate capacity %
	if (battery_on == 1) {
		// NOTE: basic calculation based only on voltage.

		// NOTE: calculate as multiplier 0.0 .. 1.0
		//if (battery_voltage < battery_low_voltage) {
		//	battery_charge_percent = 0;
		//} else {
		//	temp_double = ((battery_voltage - battery_low_voltage) / (temp_double - battery_low_voltage));
		//	if (temp_double > 1.0) temp_double = 1.0;
		//}

		temp_double = 0.0;
		if (battery_voltage > battery_low_voltage) {
			temp_double = ((battery_voltage - battery_low_voltage) / (temp_double - battery_low_voltage));
			if (temp_double > 1.0) temp_double = 1.0;
		}

		// NOTE: measured output current in mA
		// Run time min = ( ( capacity % multiplier * max capacity ) / (output current / 1000) ) * 60
		if (output_current > 0.0) {
			battery_runtime_mins = (uint16_t)((temp_double * battery_max_capacity) / (output_current / 1000.0)) * 60;
		} else {
			battery_runtime_mins = UINT16_MAX;
		}

		// Capacity multiplier => %
		battery_charge_percent = (uint8_t)(temp_double * 100.0);
	} else {
		// If battery disconnected
		battery_charge_percent = 0;
		battery_runtime_mins = 0;
	}


	// 8. Too long on battery -> report battery low to shutdown.
	if (on_battery_time_mins > on_battery_max_time_mins) {
		battery_low = 1;
	}



	// 2. Temperature
	if ((millis() - last_measure_time) > (temp_measure_interval_secs * 1000)) {
		measure_temperature();
		last_measure_time = millis();
		if (is_debug_mode > 0) {
			Serial.print(F("temp = ")); Serial.println(temperature);
		}

		// 2.1. Only if not testing or charging
		//if ((test_in_progress == 0) && 
		if ((fan_on_test < 1) && (fan_on_charge < 1)) {
			if (temperature > temperature_high_limit) {
				FAN_ON();

				if (fan_on_temp < 1) {
					fan_on_temp = 1;
					uint8_t f = fan_min_pwm + ((fan_max_pwm - fan_min_pwm) * 0.8);
					fan_pwm = set_fan_pwm(f, 110);
					if (is_debug_mode > 0) {
						Serial.print(F("temp h -> fan start ")); Serial.println(f);
					}
				}

			} else {
				// On and charge in standby -> can stop
				if (fan_on_temp > 0) {
					fan_pwm = set_fan_pwm(fan_min_pwm, 111);
					fan_on_temp = 0;
					if (is_debug_mode > 0) {
						Serial.println(F("temp l -> fan min"));
					}
				}

			}
		}
	}




	// 9. Serial commands

	int bytes = Serial.available();
	if (bytes > 0) {
		//Serial.println("avail!");
		memset(buf, 0, sizeof(buf));
		uint8_t chars = Serial.readBytesUntil('\r', buf, sizeof(buf));
		//Serial.print("chars = ");  Serial.println(chars);
		if (is_debug_mode > 0) {
			Serial.print(F("in buf = '"));  Serial.print(buf); Serial.println(F("'"));
		}

		/*
		*	Megatec- protocol
		*	https://networkupstools.org/protocols/megatec.html
		*
		*	Q1	Status Inquiry
		*	Q	Turn On/Off beep (not implemented)
		*	D	Status Inquiry Disable (not implemented)
		*	CT	Cancel Test
		*	C	Cancel Shut Down
		*	I	UPS Information
		*	F	UPS Rating Information
		*	T	10 Seconds Test
		*	TL	Test until Battery Low
		*	T<n>	Test for Specified Time Period   (T05)
		*	S<n>	Shut Down Command
		*	S<n>R<m>	Shut Down and Restore Command
		*
		*
		*	Additional commands
		*	p				print settings
		*	v               print values
		*	l               print flags
		*
		*	wiva1.002		set input_voltage_adjust   [multiplier] (-10.0 ... 10.0)
		*	wbva1.0085		set battery_voltage_adjust [multiplier] (-10.0 ... 10.0)
		*	wova1.0			set output_voltage_adjust  [multiplier] (-10.0 ... 10.0)
		*	wcca1.05		set charge_current_adjust  [multiplier] (-10.0 ... 10.0)
		*	woca1.028		set output_current_adjust  [multiplier] (-10.0 ... 10.0)
		*
		*	wbmv13.7		set battery_max_voltage [V] battery full voltage with charge (0.0 ... 15.0)
		*	wbfv12.7		set battery_full_voltage [V] battery full voltage without charge (0.0 ... 15.0)
		*	wblv11.7		set battery_low_voltage [V] battery empty voltage (0.0 ... 15.0)
		*	wbbv10.5		set battery_bad_voltage [V] battery too low voltage (0.0 ... 15.0)
		*	wbmc4.0			set battery_max_capacity [Ah] battary capacity (0.0 ... 9.0)
		*	wbcm1440		set battery_max_charge_mins [minutes] = 24h   max charge time (0 ... 4320)
		*	wbbm360			set battery_max_bad_mins [minutes] = 6h  max battery too low time (0 ... 1440)
		*	wblm720			set battery_max_low_mins [minutes] = 12h max battery low time (0 ... 1440)
		*	wbdt2022-01-10	set battery_date [yyyy-mm-dd]       battery change date (string, 10 chars)
		*	wobm120         set on_battery_max_time_mins [minutes] = 2h  max run time on battery (0 ... 540)
		*	wmoc4.0			set max_output_current [A]   max output current (1.0 ... 5.0)
		* 
		*	wati1440		set auto_test_interval_mins [minutes] = 24h    automatic test interval (>= 0)
		*	watd0			set auto_test_duration_mins [minutes], 0 = 30seconds, >0 minutes   automatic test duration (0 ... 60)
		*	wtmi10			set measure_interval_secs [seconds]   temperature measure interval (0 ... 65535)
		*	wthl30.0		set temperature_high_limit [C degrees]   temperature high limit (25.0 ... 60.0)
		*
		*	wfmi20			set fan_min_pwm [%]    fan min speed (0 ... 100)
		*	wfma80			set fan_max_pwm [%]    fan max speed (0 ... 100)
		*
		*	wdbg1			set debug_override [0 = normal, 1 = debug messages on]    debug messages
		*
		*
		*
		*/

		if ((buf[0] == 'Q') && (buf[1] == '1') && (chars >= 2)) {
			// Q1	Status Inquiry
			send_status();
		} else if ((buf[0] == 'Q') && (chars == 1)) {
			// Q	Toggle beeper => NOP
		} else if ((buf[0] == 'p') && (chars == 1)) {
			// p => print settings
			if (is_debug_mode > 0) {
				print_settings();
			}
		} else if ((buf[0] == 'v') && (chars == 1)) {
			// v => print values
			if (is_debug_mode > 0) {
				print_values();
			}
		} else if ((buf[0] == 'l') && (chars == 1)) {
			// l => print state
			if (is_debug_mode > 0) {
				print_state();
			}
		} else if ((buf[0] == 'D') && (chars == 1)) {
			// D	Status disable => NOP
		} else if ((buf[0] == 'C') && (buf[1] == 'T') && (chars == 2)) {
			// CT	Cancel Test Command
			if (is_debug_mode > 0) Serial.println(F("=> CT"));
			stop_test();
		} else if ((buf[0] == 'C') && (chars == 1)) {
			// C	Cancel Shutdown Command
			if (is_debug_mode > 0) Serial.println(F("=> C"));
			cancel_shutdown();
		} else if ((buf[0] == 'I') && (chars == 1)) {
			// I	UPS Information
			//             "#123456789012345 1234567890 1234567890"
			Serial.print(F("#Isto Saarinen   DC UPS     20220212  "));
			Serial.print('\r');
			if (is_debug_mode > 0) Serial.print('\n');
		} else if ((buf[0] == 'F') && (chars == 1)) {
			// F	UPS rating
			Serial.print(F("#019.5 "));								// rating voltage (= source voltage)
			Serial.print(format_uint3(buf, max_output_current));	// rating current
			Serial.print(F(" 12.00"));								// battery voltage
			Serial.print(F(" 00.0\r"));								// frequency (= DC operation)
			if (is_debug_mode > 0) Serial.print('\n');
		} else if ((buf[0] == 'T') && (chars >= 1)) {
			// T	10 Seconds Test
			// TL	Test until Battery Low
			// T<n>	Test for Specified Time Period
			// Accept:
			// TNN      -> NN minutes
//			if ((output_state != OUTPUT_STATE_ON_DELAY) && (ups_fail == 0) && (utility_fail == 0) && (test_in_progress == 0) && (shutdown_active == 0) && (battery_low == 0) && (battery_on == 1) && (battery_connected == 1)) {

			if ((output_state != OUTPUT_STATE_ON_DELAY) &&
				(utility_fail == 0) &&
				(test_in_progress == 0) &&
				(shutdown_active == 0)) {

				test_run_time_secs = 0;
				if (chars == 1) {
					// T	10 Seconds Test
					test_run_time_secs = 10;
					if (is_debug_mode > 0) {
						Serial.print(F("T = ")); Serial.println(test_run_time_secs);
					}
				} else if ((buf[1] == 'L') && (chars == 2)) {
					// TL	Test until Low Battery occurs
					//test_until_low_battery = 1;
					// Fake: test only 10 minutes
					test_run_time_secs = 600;
					if (is_debug_mode > 0) {
						Serial.print(F("TL = ")); Serial.println(test_run_time_secs);
					}
				} else if (chars >= 3) {
					// Tnn	Test for a Specified Time Period
					temp_long = atol(buf + 1);

					// Fake, max test time is 10 minutes, not 99
					if ((temp_long > 0) && (temp_long <= 10)) {
						test_run_time_secs = 60 * (uint16_t)temp_long;
					}

					if (is_debug_mode > 0) {
						Serial.print(F("T")); Serial.print(temp_long); Serial.print(" = "); Serial.println(test_run_time_secs);
					}
				}
				if (is_debug_mode > 0) {
					Serial.print(F("=> T, time = ")); Serial.println(test_run_time_secs);
				}
				//START_TEST();
				start_test();
			}
		} else if ((buf[0] == 'S') && (chars >= 3)) {
			// S<n>			Shut Down Command
			// S<n>R<m>		Shut Down and Restore Command
			// Accept:
			// S.N		-> .N minutes
			// SNN      -> NN minutes
			// RN, RNN, RNNN, RNNNN  -> N...NNNN minutes
			if ((output_state == OUTPUT_STATE_ON) && ((buf[1] == '.') || (buf[1] == '0')) && ((buf[2] >= '0') && (buf[2] <= '9'))) {
				uint16_t secs = 0;
				uint16_t t1 = 0;
				uint16_t t2 = 0;
				t2 = (buf[2] - '0');
				if (t2 > 9) t2 = 9;
				if (t2 < 0) t2 = 0;

				if (buf[1] == '.') {
					secs = (uint16_t)(60 * ((float)t2 / 10.0));
				} else {
					t1 = (buf[1] - '0');
					if (t1 > 9) t1 = 9;
					if (t1 < 0) t1 = 0;
					secs = (60 * (t1 * 10 + t2));
				}

				startup_delay_secs = 10;
				shutdown_delay_secs = secs;

				if (buf[3] == 'R') {
					temp_long = atol(buf + 4);
					if ((temp_long > 0) && (temp_long <= 9999)) {
						startup_delay_secs = 60 * temp_long;
					}
				}

				if (is_debug_mode > 0) {
					Serial.print(F("=> S = ")); Serial.print(shutdown_delay_secs); Serial.print(F(" s, R = ")); Serial.println(startup_delay_secs);
				}
				stop_test();
				start_shutdown();
			}
		} else if (buf[0] == 'w') {
			// EEPROM settings
			char* p = buf + 4;

			if (strncmp(buf, "wiva", 4) == 0) {
				// Input voltage adjust
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wiva=")); Serial.println(temp_double, 5);

				}
				if ((temp_double >= -10.0) && (temp_double <= 10.0)) {
					input_voltage_adjust = temp_double;
					eeprom_update_float(&ee_input_voltage_adjust, input_voltage_adjust);
				}
			} else if (strncmp(buf, "wbva", 4) == 0) {
				// Battery voltage adjust
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wbva=")); Serial.println(temp_double, 5);
				}
				if ((temp_double >= -10.0) && (temp_double <= 10.0)) {
					battery_voltage_adjust = temp_double;
					eeprom_update_float(&ee_battery_voltage_adjust, battery_voltage_adjust);
				}
			} else if (strncmp(buf, "wova", 4) == 0) {
				// Output voltage adjust
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wova=")); Serial.println(temp_double, 5);
				}
				if ((temp_double >= -10.0) && (temp_double <= 10.0)) {
					output_voltage_adjust = temp_double;
					eeprom_update_float(&ee_output_voltage_adjust, output_voltage_adjust);
				}
			} else if (strncmp(buf, "wcca", 4) == 0) {
				// Charge current adjust
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wcca=")); Serial.println(temp_double, 5);
				}
				if ((temp_double >= -10.0) && (temp_double <= 10.0)) {
					charge_current_adjust = temp_double;
					eeprom_update_float(&ee_charge_current_adjust, charge_current_adjust);
				}
			} else if (strncmp(buf, "woca", 4) == 0) {
				// Output current adjust
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("woca=")); Serial.println(temp_double, 5);
				}
				if ((temp_double >= -10.0) && (temp_double <= 10.0)) {
					output_current_adjust = temp_double;
					eeprom_update_float(&ee_output_current_adjust, output_current_adjust);
				}
			} else if (strncmp(buf, "wbmv", 4) == 0) {
				// Battery max voltage
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wbmv=")); Serial.println(temp_double, 2);
				}
				if ((temp_double >= 0.0) && (temp_double <= 15.0)) {
					battery_max_voltage = temp_double;
					eeprom_update_float(&ee_battery_max_voltage, battery_max_voltage);
				}
			} else if (strncmp(buf, "wbfv", 4) == 0) {
				// Battery full voltage
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wbfv=")); Serial.println(temp_double, 2);

				}
				if ((temp_double >= 0.0) && (temp_double <= 15.0)) {
					battery_full_voltage = temp_double;
					eeprom_update_float(&ee_battery_full_voltage, battery_full_voltage);
				}
			} else if (strncmp(buf, "wblv", 4) == 0) {
				// Battery low voltage
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wblv=")); Serial.println(temp_double, 2);

				}
				if ((temp_double >= 0.0) && (temp_double <= 15.0)) {
					battery_low_voltage = temp_double;
					eeprom_update_float(&ee_battery_low_voltage, battery_low_voltage);
				}
			} else if (strncmp(buf, "wbbv", 4) == 0) {
				// Battery bad voltage
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wbbv=")); Serial.println(temp_double, 2);

				}
				if ((temp_double >= 0.0) && (temp_double <= 15.0)) {
					battery_bad_voltage = temp_double;
					eeprom_update_float(&ee_battery_bad_voltage, battery_bad_voltage);
				}
			} else if (strncmp(buf, "wbmc", 4) == 0) {
				// Battery max capacity
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wbmc=")); Serial.println(temp_double, 1);

				}
				if ((temp_double >= 0.0) && (temp_double <= 9.0)) {
					battery_max_capacity = temp_double;
					eeprom_update_float(&ee_battery_max_capacity, battery_max_capacity);
				}
			} else if (strncmp(buf, "wbcm", 4) == 0) {
				// Battery max charge mins
				temp_long = atol(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wbcm=")); Serial.println(temp_long);

				}
				if ((temp_long >= 0) && (temp_long <= 4320)) {
					battery_max_charge_mins = temp_long;
					eeprom_update_word(&ee_battery_max_charge_mins, battery_max_charge_mins);
				}
			} else if (strncmp(buf, "wbbm", 4) == 0) {
				// Battery max bad mins
				temp_long = atol(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wbbm=")); Serial.println(temp_long);

				}
				if ((temp_long >= 0) && (temp_long <= 1440)) {
					battery_max_bad_mins = temp_long;
					eeprom_update_word(&ee_battery_max_bad_mins, battery_max_bad_mins);
				}
			} else if (strncmp(buf, "wblm", 4) == 0) {
				// Battery max low mins
				temp_long = atol(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wblm=")); Serial.println(temp_long);

				}
				if ((temp_long >= 0) && (temp_long <= 1440)) {
					battery_max_low_mins = temp_long;
					eeprom_update_word(&ee_battery_max_low_mins, battery_max_low_mins);
				}
			} else if (strncmp(buf, "wbdt", 4) == 0) {
				// Battery date
				memmove(battery_date, p, 10);
				eeprom_update_block(battery_date, &ee_battery_date, 10);
				if (is_debug_mode > 0) {
					Serial.print(F("wbdt=")); Serial.println(battery_date);
				}
			} else if (strncmp(buf, "wobm", 4) == 0) {
				// On battery max time mins
				temp_long = atol(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wobm=")); Serial.println(temp_long);

				}
				if ((temp_long >= 0) && (temp_long <= 540)) {
					on_battery_max_time_mins = temp_long;
					eeprom_update_word(&ee_on_battery_max_time_mins, on_battery_max_time_mins);
				}
			} else if (strncmp(buf, "wmoc", 4) == 0) {
				// Max output current
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wmoc=")); Serial.println(temp_double, 1);
				}
				if ((temp_double >= 1.0) && (temp_double <= 5.0)) {
					max_output_current = temp_double;
					eeprom_update_float(&ee_max_output_current, max_output_current);
				}
			} else if (strncmp(buf, "wati", 4) == 0) {
				// Auto test interval mins
				temp_long = atol(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wati=")); Serial.println(temp_long);
				}
				if (temp_long >= 0) {
					auto_test_interval_mins = temp_long;
					eeprom_update_word(&ee_auto_test_interval_mins, auto_test_interval_mins);
				}
			} else if (strncmp(buf, "watd", 4) == 0) {
				// Auto test duration mins
				temp_int = atoi(p);
				if (is_debug_mode > 0) {
					Serial.print(F("watd=")); Serial.println(temp_int);
				}
				if ((temp_int >= 0) && (temp_int <= 60)) {
					auto_test_duration_mins = temp_int;
					eeprom_update_byte(&ee_auto_test_duration_mins, auto_test_duration_mins);
				}
			} else if (strncmp(buf, "wtmi", 4) == 0) {
				// Temp measure interval secs
				temp_long = atol(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wtmi=")); Serial.println(temp_long);
				}
				if ((temp_long >= 0) && (temp_long <= 65535)) {
					temp_measure_interval_secs = temp_long;
					eeprom_update_word(&ee_temp_measure_interval_secs, temp_measure_interval_secs);
				}
			} else if (strncmp(buf, "wthl", 4) == 0) {
				// Temperature high limit
				temp_double = atof(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wthl=")); Serial.println(temp_double, 1);
				}
				if ((temp_double >= 25.0) && (temp_double <= 60.0)) {
					temperature_high_limit = temp_double;
					eeprom_update_float(&ee_temperature_high_limit, temperature_high_limit);
				}
			} else if (strncmp(buf, "wfmi", 4) == 0) {
				// Fan min pwm
				temp_int = atoi(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wfmi=")); Serial.println(temp_int);
				}
				if ((temp_int >= 0) && (temp_int <= 100)) {
					fan_min_pwm = temp_int;
					eeprom_update_byte(&ee_fan_min_pwm, fan_min_pwm);
					if (fan_pwm < fan_min_pwm) {
						fan_pwm = set_fan_pwm(fan_min_pwm, 10);
					}
				}
			} else if (strncmp(buf, "wfma", 4) == 0) {
				// Fan max pwm
				temp_int = atoi(p);
				if (is_debug_mode > 0) {
					Serial.print(F("wfma=")); Serial.println(temp_int);
				}
				if ((temp_int >= 0) && (temp_int <= 100)) {
					fan_max_pwm = temp_int;
					eeprom_update_byte(&ee_fan_max_pwm, fan_max_pwm);
					if (fan_pwm > fan_max_pwm) {
						fan_pwm = set_fan_pwm(fan_max_pwm, 20);
					}
				}
			} else if (strncmp(buf, "wdbg", 4) == 0) {
				// Debug override
				if (buf[4] == '0') {
					debug_override = 0;
				}
				else if (buf[4] == '1') {
					debug_override = 1;
				}
				if (debug_override > 0) {
					Serial.print(F("wdbg=")); Serial.println(debug_override);
				}
				eeprom_update_byte(&ee_debug_override, debug_override);
				is_debug_mode = IS_DEBUG_INPUT();
				if (debug_override == 1) {
					is_debug_mode = 1;
				}
			}

		}
		
	} // bytes > 0






}
