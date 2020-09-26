/*
 * M4C_Thermometer/src/main.c
 *
 * Created: 6/9/2020 10:18:34 PM
 * Author : Jan Kok
 */ 

// F_CPU should be defined in the project settings so the definition gets passed into the compiler command line.
//#define F_CPU 1000000    // CPU clock frequency, Hz.
// The Release build environment should define NDEBUG. The Debug environment should define DEBUG.

#include <stdbool.h>
#include <avr/io.h>
#ifdef DEBUG
//#define __DELAY_BACKWARD_COMPATIBLE__    // This might help if single stepping or setting breakpoints near _delay_ms()
#endif
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "i2c_master.h"

//////////////////////////////////////// Solution configurations

// ONE of the following preprocessor symbols should be defined on the compiler command line. This can be managed through the Solution Configurations in Atmel Studio, or through a makefile,
// NDEBUG for the Release config (what gets shipped in the final product).
// VALIDATE for the Validate config. For testing/validating temperature accuracy.
// DEBUG for the Debug config.

#ifdef NDEBUG
	// ***** Set parameters for Release (non-debug) firmware here: *****
	#define SCALE DEG_F          // Set this to DEG_F or DEG_C to select which temperature scale to display readings.
	#define DECIMALS TENTHS      // In Release, display temperature as [1]xx.x in deg F or C, depending on SCALE
	#define MAX_DEG 1099         // Above 109.9 deg F is displayed as "HI"
	#define MIN_DEG 600          // Below 60.0 deg F is displayed as "LO"
	#define HOLD_TIME_MS 5000    // How long (ms) to display temperature reading if button is not held down.
#endif

#ifdef VALIDATE
	#define NDEBUG
	// ***** Set parameters for validation firmware here: *****
	// This configuration is intended to be identical to the Release configuration except for the following display-related parameters.
	#define SCALE DEG_C         // Set this to DEG_F or DEG_C to select which temperature scale to display readings.
	#define DECIMALS HUNDREDTHS // Display temperature as xx.xx
	#define MAX_DEG 19999       // Above 199.99 deg C is displayed as "HI"
	#define MIN_DEG -999        // Below -9.99 deg C is displayed as "LO"
	#define HOLD_TIME_MS 5000   // How long (ms) to display temperature reading if button is not held down.
#endif

#ifdef DEBUG
	// Set parameters for DEBUG firmware.
	#define SCALE DEG_C          // Always use DEG_C while debugging.
	#define DECIMALS HUNDREDTHS  // In DEBUG, display temperature as xx.xx (deg C)
	#define MAX_DEG 4999         // Above 49.99 deg C is displayed as "HI"
	#define MIN_DEG -999         // Below -9.99 deg C is displayed as "LO"
	//#define HOLD_TIME_MS 5000  // How long (ms) to display temperature reading if button is not held down.
	//#define NO_ADC             // Optionally define this to debug ADC current draw issues.
	//#define NO_SENSOR          // Optionally define this to debug MLX sensor current draw or other issues.
	#define NO_DISPLAY           // Define this to disable display to get more stable battery and Vcc voltages.
	#define ALLOW_LOW_VDD        // Define this to allow displaying temperature readings when Vdd is less than sensor's minimum operating Vdd.

	#define AUTORUN            // Automatically wake up after sleeping. Don't wait for button.
		//#define DISP988      // Define this to force temperature display to 98.8 to give consistent load for investigating expected battery life.
		//#define SLEEP_TIME_MS (20000 - 5318)  // Sleep time when autorunning. Want 5 seconds of display time and 15 seconds of low power per cycle.

	#define DATALOG           // Define this to output various info through serial (port B bit 7, MCU pin 10)
		#define SLEEP_TIME_MS 1000  // Sleep time when autorunning. Want 5 seconds of display time and 15 seconds of low power per cycle.
		#define HOLD_TIME_MS 1000   // How long (ms) to display temperature reading if button is not held down.

	#define DISPLAY_TA        // Define this to display ambient temperature briefly before object temperature.
	#define DISP_BATLOW_VOLTS // Define this to have Vcc voltage displayed if low battery is detected.
	//#define DEBUG_READSTATE   // Define this to take multiple readings in a row, for investigating noise, drift, etc. of temperature sensor.
#endif

#define CHOOSE_EMISSIVITY       // If defined, lets user choose emissivity value at battery insertion time.
#define IR_TO_ORAL_OFFSET 200   // Add this amount (hundredths of a degree C) to the forehead skin temperature to get the equivalent oral temperature.
                                // Choose 2.00C offset for now because it's a round number and because it's in the ballpark of what I think the offset is for me with room temperature ~27C.
								// I think the offset depends on the ambient temperature. For ambient temperature below 27C, the offset may be greater.
#define EMISSIVITY_X1000 980    // Desired emissivity setting * 1000. So .95 or 95% would be entered here as 950.
#define INITIAL_LOAD_MS	20      // Time in ms to leave 20mA load on.
#define LOAD_RECOVERY_MS 80     // Time allowed for batteries to recover (shouldn't matter except possibly with low batteries.
#define READING_WAIT_MS 730     // was 278 Time allowed after sensor wakeup for sensor to get reading.

#define TENTHS 1
#define HUNDREDTHS 2
#define DEG_F 1
#define DEG_C 0

/////////////////////////////// Button check function

#define BUTTON_BIT    0   // Bit number in port B for button input line.
#define BUTTON_PUSHED_VALUE (1<<BUTTON_BIT)   // Value of PINB & (1<<BUTTON_BIT) when button is pushed.
#define BUTTON_RELEASED_VALUE (0<<BUTTON_BIT) // Value of PINB & (1<<BUTTON_BIT) when button is released.

// Pin change interrupt ISR. Needed because pin change interrupt is used to wake processor up from power down mode
// when button is pressed, and global interrupts must be enabled in order for wakeup to occur.
ISR(PCINT0_vect)
{
}

// Keep track of previous and latest button state (1 is pushed).
uint8_t OldButtonState = 0;
uint8_t NewButtonState = 0;

// Return 1 if button is pushed, else return 0.
// Contains no button debouncing logic. Should not be needed because bouncing only occurs for a few ms after contacts close,
// and we're not intensely monitoring the button during that bounce period.
static uint8_t ButtonIsPushed(void)
{
	OldButtonState = NewButtonState;
	NewButtonState = (PINB & (1<<BUTTON_BIT)) == BUTTON_PUSHED_VALUE;
	return NewButtonState;
}

// Check and update button state. Return 1 if button has just transitioned to pushed state.
static int8_t ButtonJustPushed(void)
{
	ButtonIsPushed();
	return NewButtonState && !OldButtonState;
}

// Return if/when button is released.
static void WaitButtonRelease(void)
{
	_delay_ms(10); // Some crude button debouncing.
	while (ButtonIsPushed()) {
	}
}

////////////////////////  Display stuff

// Number of digits in 7-segment display.
#define DISP_N_DIGITS 4

// Bits in PORTC that control digit drivers.
#define DIGIT_DRIVER_MASK 0xE

// LS Digit is controlled by this bit in PORTB.
#define LS_DIGIT_BIT 6

// Number of ms to display each digit while scanning.
#define SCAN_STEP_MS 4

// Define which bit controls which segment of 7-segment display (A-G & decimal point).
#define sA (1<<2)
#define sB (1<<0)
#define sC (1<<3)
#define sD (1<<7)
#define sE (1<<6)
#define sF (1<<4)
#define sG (1<<1)
#define sDP (1<<5)

// This table defines which segments of 7-segment display are enabled for each digit 0-9.
const uint8_t DigitToSegments[] = {
    sA|sB|sC|sD|sE|sF,    // 0
    sB|sC,                // 1
    sA|sB|sD|sE|sG,       // 2
    sA|sB|sC|sD|sG,       // 3
    sB|sC|sF|sG,          // 4
    sA|sC|sD|sF|sG,       // 5
    sA|sC|sD|sE|sF|sG,    // 6
    sA|sB|sC,             // 7
    sA|sB|sC|sD|sE|sF|sG, // 8
    sA|sB|sC|sD|sF|sG     // 9
};

// Define some more segment patterns.
#define BLANK_SEGMENTS 0                      // blank
#define MINUS_SIGN_SEGMENTS sG                // -
#define EXCLAMATION_POINT_SEGMENTS (sB|sDP)   // !
#define QUESTION_MARK_SEGMENTS (sA|sB|sE|sG)  // ?
#define EQUALS_SEGMENTS (sD|sG)        // =
#define A_SEGMENTS (sA|sB|sC|sE|sF|sG) // A
#define b_SEGMENTS (sC|sD|sE|sF|sG)    // b
#define C_SEGMENTS (sA|sD|sE|sF)       // C
#define c_SEGMENTS (sD|sE|sG)          // c
#define d_SEGMENTS (sB|sC|sD|sE|sG)    // d
#define E_SEGMENTS (sA|sD|sE|sF|sG)    // E
#define F_SEGMENTS (sA|sE|sF|sG)       // F
#define G_SEGMENTS (sA|sC|sD|sE|sF)    // G
#define H_SEGMENTS (sB|sC|sE|sF|sG)    // H
#define I_SEGMENTS (sB|sC)             // I
#define L_SEGMENTS (sD|sE|sF)          // L
#define n_SEGMENTS (sC|sE|sG)          // n
#define O_SEGMENTS (sA|sB|sC|sD|sE|sF) // O
#define o_SEGMENTS (sC|sD|sE|sG)       // o
#define r_SEGMENTS (sE|sG)             // r
#define S_SEGMENTS (sA|sC|sD|sF|sG)    // S
#define t_SEGMENTS (sD|sE|sF|sG)       // t
#define U_SEGMENTS (sB|sC|sD|sE|sF)    // U

// This array determines which segments are turned on for each digit. Index 0 is LSDigit.
int8_t DisplaySegments[DISP_N_DIGITS];

// Which digit is currently being displayed.
int8_t CurrentDigit;

// Helper function for DecimalDisplayPrep, below.
static void DecimalDisplayPrep1(uint16_t num, bool minus, int8_t force_digits, int8_t decimal_points)
{
	// Convert num to decimal digits and set up DisplaySegments.
	int8_t digit_num = 0; // Digit position we are processing. 0 = least significant digit.
	do {
		int8_t digit_val = num % 10;
		int8_t segments = DigitToSegments[digit_val];
		if (digit_num >= force_digits) {
			// Handle leading 0 blanking and minus sign if appropriate.
			if (num == 0) {
				// We're in the leading zeros region. Display a blank or a minus sign as appropriate.
				if (minus) {
					segments = MINUS_SIGN_SEGMENTS;
					minus = false; // Don't output any more minus signs.
					} else {
					segments = BLANK_SEGMENTS;
				}
			}
		}
		// If this digit should have the decimal point turned on, do that here.
		if (decimal_points & (1<<digit_num)) {
			segments |= sDP;
		}
		DisplaySegments[digit_num] = segments;
		num = num / 10;
		digit_num++;
	} while (digit_num < DISP_N_DIGITS);
}

// Set up DisplaySegments array to display num as a decimal.
// num: number to display.
// max_num: display "HI" if number is > this value.
// min_num: display "LO" if number is < this value.
// force_digits: number of digits to display without leading zero blanking.
// decimal_points: bit mask indicating which decimal point(s) to turn on.
static void DecimalDisplayPrep(int16_t num, int16_t max_num, int16_t min_num, int8_t force_digits, int8_t decimal_points)
{
    // Handle num too large or too small.
    if (num > max_num || num < min_num) {
        int8_t i = DISP_N_DIGITS-1;
        if (num > max_num) {
            DisplaySegments[i--] = H_SEGMENTS;
            DisplaySegments[i--] = I_SEGMENTS;
        } else {
            DisplaySegments[i--] = L_SEGMENTS;
            DisplaySegments[i--] = O_SEGMENTS;
        }
        // Set remaining digits to blanks.
        while (i >= 0) {
            DisplaySegments[i--] = BLANK_SEGMENTS;
        }
        return;
    }
    // Handle negative numbers. May not work correctly for -32768, but that should be too small anyway.
    bool minus = num < 0;
    if (minus) {
        num = -num;
    }
	// Do the decimal decoding and formatting.
	DecimalDisplayPrep1(num, minus, force_digits, decimal_points);
}

// Turn off all segment drivers.
static inline void TurnOffDigits(void)
{
	PORTC &= ~DIGIT_DRIVER_MASK;
	PORTB &= ~(1<<LS_DIGIT_BIT);	
}

// Turn on the given digit driver..
static inline void TurnOnDigit(uint8_t digit)
{
	if (digit > 0) {
		PORTC |= 1<<digit;		
	} else {
		PORTB |= 1<<LS_DIGIT_BIT;
	}
}

// Turn off the current digit and illuminate the next digit.
// This function should be called every 4 milliseconds or less to scan the display.
static void DisplayScanStep(void)
{
	// Turn all the segments OFF before changing digit.
	PORTD = BLANK_SEGMENTS;
	// Turn OFF current digit.
	TurnOffDigits();
	// Step to next digit.
	if (--CurrentDigit < 0) {
		CurrentDigit = DISP_N_DIGITS - 1;
	}
#ifndef NO_DISPLAY
	// Enable next digit.
	TurnOnDigit(CurrentDigit);
	PORTD = DisplaySegments[CurrentDigit];
#endif
}

// Turn on a decimal point. This is for creating a load on battery and as a side-effect gives some feedback to user that button was pushed.
static void DecimalOn(void)
{
	TurnOnDigit(1);
	PORTD = sDP;
}

// Let there not be light!
static void BlankDisplay(void)
{
    PORTD = BLANK_SEGMENTS;
    TurnOffDigits();
}

// Display whatever is in DisplaySegments for approx. n_ms milliseconds.
// Return 1 immediately if button is pressed (transitions to ON) while displaying. Otherwise return 0 when done displaying.
static uint8_t DisplayNms(uint16_t n_ms)
{
	uint16_t n_steps = n_ms / SCAN_STEP_MS;
	for (uint16_t i = 0; i < n_steps; i++) {
		DisplayScanStep();
		_delay_ms(SCAN_STEP_MS);
		if (ButtonJustPushed()) {
			BlankDisplay();
			return 1;
		}
	}
	BlankDisplay();
	return 0;
}

// Display a possibly signed number for disp_ms milliseconds.
static uint8_t DisplayNumber(int16_t value, uint8_t decimal_place, uint16_t disp_ms)
{
	uint8_t force_digits = 1;
	uint8_t decimal_points = 0;
	if (decimal_place) {
		force_digits = decimal_place;
		decimal_points = 1<<decimal_place;
	}
	DecimalDisplayPrep(value, 9999, -999, force_digits, decimal_points);
	return DisplayNms(disp_ms);
}

// Set 4 digits of the display with the given segment patterns, then display for disp_ms.
// Return 1 if button was pressed while displaying, else return 0.
static uint8_t DisplayChars(uint8_t d3, uint8_t d2, uint8_t d1, uint8_t d0, uint16_t disp_ms)
{
	DisplaySegments[3] = d3;
	DisplaySegments[2] = d2;
	DisplaySegments[1] = d1;
	DisplaySegments[0] = d0;
	return DisplayNms(disp_ms);
}

// In the (hopefully) highly unlikely event that something goes wrong, e.g. I2C gets hung,
// display "ErrN" (where N = the value of the digit arg) for a few seconds, then continue.
static uint8_t error(uint8_t digit)
{
	return DisplayChars(E_SEGMENTS, r_SEGMENTS, r_SEGMENTS, DigitToSegments[digit], 2000);
}

#ifdef DISP_BATLOW_VOLTS
// Display millivolts arg as x.xxx.
static uint8_t DisplayVolts(uint16_t mv)
{
	DecimalDisplayPrep(mv, 4000, 0, 4, 1<<3);
	uint8_t status = DisplayNms(1000);
	BlankDisplay();
	return status;
}
#endif

// Display "bAt" for 1 second, then display "LO" for 1 second.
// Return 1 immediately if button becomes pressed while displaying. Otherwise return 0 when done displaying.
static uint8_t DisplayBatLo(uint16_t mv)
{
	if (DisplayChars(b_SEGMENTS, A_SEGMENTS, t_SEGMENTS, BLANK_SEGMENTS, 1000)) {
		return 1;
	}
	if (DisplayChars(L_SEGMENTS, O_SEGMENTS, BLANK_SEGMENTS, BLANK_SEGMENTS, 1000)) {
		return 1;
	}

#ifdef DISP_BATLOW_VOLTS
	if (DisplayVolts(mv)) {
		return 1;
	}
#endif

	return 0;
}

#ifdef DEBUG
// Display "dEb" for a few seconds to indicate that this is a debug build, not a release build.
// Return 1 immediately if button becomes pressed while displaying. Otherwise return 0 when done displaying.
static uint8_t DisplayDeb(void)
{
	return DisplayChars(d_SEGMENTS, E_SEGMENTS, b_SEGMENTS, BLANK_SEGMENTS, 2000);
}
#endif

#ifdef NO_ADC
// Display "-AdC" for a few seconds to indicate that ADC code is stubbed out.
// Return 1 immediately if button becomes pressed while displaying. Otherwise return 0 when done displaying.
static uint8_t Display_ADC(void)
{
	return DisplayChars(MINUS_SIGN_SEGMENTS, A_SEGMENTS, d_SEGMENTS, C_SEGMENTS, 2000);
}
#endif

#ifdef NO_SENSOR
// Display "-SEn" for a few seconds to indicate that sensor code is stubbed out.
// Return 1 immediately if button becomes pressed while displaying. Otherwise return 0 when done displaying.
static uint8_t Display_SENSOR(void)
{
	return DisplayChars(MINUS_SIGN_SEGMENTS, S_SEGMENTS, E_SEGMENTS, n_SEGMENTS, 2000);
}
#endif

////////////////////////////// Serial output (for debugging or logging)

#define SERIAL_OUT_BIT 7

#ifdef DATALOG

#define SERIAL_BIT_TIME 104 // For 9600 baud, it's about 1000000/9600 = 104 us.
#define SERIAL_WAIT_TIME (SERIAL_BIT_TIME - 12)  // Account for loop overhead.

static void PutChar(char c)
{
	// Send start bit.
	PORTB |= (1<<SERIAL_OUT_BIT);
	_delay_us(SERIAL_WAIT_TIME + 4);
	for (uint8_t i=0; i<8; i++) { // Output 8 bits, LSB first, inverted.
		if (c & 1) {
			PORTB &= ~(1<<SERIAL_OUT_BIT); // Output a 1.
			} else {
			PORTB |= (1<<SERIAL_OUT_BIT);  // Output a 0.
		}
		c >>= 1;
		_delay_us(SERIAL_WAIT_TIME);
	}
	PORTB &= ~(1<<SERIAL_OUT_BIT); // Output stop bit.
	_delay_us(SERIAL_WAIT_TIME - 2);
}

static void PutStr(char *str)
{
	for (uint8_t i = 0; str[i]; i++) {
		PutChar(str[i]);
	}
}

// Buffer for use in conversion to decimal number.
#define FMT_BUF_SIZE 10
static char fmt_buf[FMT_BUF_SIZE+1];

// Format num in decimal, for sending to serial output.
static void DecimalFormat(int32_t num, int8_t force_digits, int8_t decimal_points)
{
	fmt_buf[FMT_BUF_SIZE] = '\0';
	// Handle negative numbers.
	bool minus = num < 0;
	if (minus) {
		num = -num;
	}
	// Convert num to decimal digits and set up DisplaySegments.
	int8_t digit_num = 0; // Digit position we are processing. 0 = least significant digit.
	int8_t buf_index = FMT_BUF_SIZE; // Index into buf.
	do {
		char digit_char = '0' + num % 10;
		if (digit_num >= force_digits) {
			// Handle leading 0 blanking and minus sign if appropriate.
			if (num == 0) {
				// We're in the leading zeros region. Output a blank or a minus sign as appropriate.
				if (minus) {
					digit_char = '-';
					minus = false; // Don't output any more minus signs.
					} else {
					digit_char = ' ';
				}
			}
		}
		// If this digit should have the decimal point turned on, do that here.
		if (decimal_points & (1<<digit_num)) {
			fmt_buf[--buf_index] = '.';
		}
		fmt_buf[--buf_index] = digit_char;
		num = num / 10;
		digit_num++;
	} while (num || digit_num <= force_digits);
	
	// Output the string to serial port.
	PutStr(fmt_buf + buf_index);
}

#endif

/////////////////////////////// Data conversion stuff

// Convert raw temperature reading from sensor, Vcc voltage correction and additional offset to tenths or hundredths of a degree C or F.
static uint16_t Convert(uint16_t raw, int16_t max_deg, int16_t min_deg, uint8_t decimals, uint8_t scale, int32_t avgVcc, uint16_t offset)
{
	int32_t value = raw * 2; // Temp in centiK. Any emissivity corrections have already been done by sensor.
	value -= 27315;          // Temp in centidegrees C.
	
	// Compensate for variance in Vcc from the standard 3V.
	// "The typical VDD dependence of the ambient and object temperature is 0.6°C/V" - MLX datasheet.
	int16_t dTdVcc_offset = ((3000 - avgVcc) * 60 + 500) / 1000;
#ifdef DATALOG
	PutStr(",Vofs,");
	DecimalFormat(dTdVcc_offset, 4, 2);
#endif
	value += dTdVcc_offset;
	
	// Now, add in this offset to convert SST into the equivalent oral thermometer reading.
	value += offset;
	
#ifdef DATALOG
	PutStr(",Tadj,");
	DecimalFormat(value, 4, 2);
#endif

	int16_t divisor = (decimals == TENTHS) ? 10 : 1;
	if (scale == DEG_F) {
		value *= 18;
		value += 32000L;
		divisor *= 10;
	}
	// Divide with rounding.
	if (value >= 0) {
		value = (value + divisor/2) / divisor;
		} else {
		value = -( (-value + divisor/2) / divisor);
	}
	return value;  // Return the integer value that gets displayed, for debugging.
}

////////////////////////////// TWI (a.k.a. I2C) stuff

#define MLX_BUS_ADDR        0       // TWI slave bus address for MLX90614xxx IR temperature sensor.
#define SDA_BIT             4       // Bit number in port C for SDA line.
#define SCL_BIT             5       // Bit number in port C for SCL line.

// Register addresses in MLX chip.
#define RAMADDR_TA 6                  // Address from which to read ambient (sensor's own) temperature.
#define RAMADDR_TOBJ1 7               // Address from which to read object (person's) temperature.
#define EEPROMADDR_EMISSIVITY 0x24    // EEPROM address containing Emissivity.
#define MLX90614_REGISTER_SLEEP 0xFF  // MLX command to go to low power.

// crc8 and part of ReadSensor are adapted from https://github.com/sparkfun/SparkFun_MLX90614_Arduino_Library/blob/master/src/SparkFunMLX90614.cpp
static uint8_t crc8 (uint8_t inCrc, uint8_t inData)
{
	uint8_t data = inCrc ^ inData;
	for (uint8_t i = 0; i < 8; i++)	{
		if (( data & 0x80 ) != 0 ) {
			data <<= 1;
			data ^= 0x07;
		} else {
			data <<= 1;
		}
	}
	return data;
}

// Note that I2C bus must be set up and working before calling this function!
// Send sleep command to MLX sensor. Set SDA and SCL appropriately for sleep.
// Sensor uses 1.3mA typical when awake and 2.5uA typical when sleeping.
static void MLXPowerDown(void)
{
#ifndef NO_SENSOR
	uint8_t crc = crc8(0, (MLX_BUS_ADDR << 1));
	crc = crc8(crc, MLX90614_REGISTER_SLEEP);
	if (i2c_writeReg(MLX_BUS_ADDR, MLX90614_REGISTER_SLEEP, &crc, 1)) {
		// Can get here if you attempt to power down sensor when it's already powered down! Don't do that!
		// Also can get here if Vcc is too low (below 2.6V?) when MLXPowerDown is called. The MLX sensor apparently hangs the I2C bus,
		// preventing issuance of a Start sequence. Unfortunately, when that happens, the system is left in a state where it uses 60uA.
		error(0);
	}
	_delay_ms(1); // Allow time for I2C bus to complete STOP sequence.
#endif

	TWCR = 0;      // Turn off TWI, return control of SCL and SDA to PORTC GPIO. Saves ~60uA!

	// Set SCL and SDA for lowest possible current draw during sleep.
	// From datasheet: "In order to limit the current consumption to the typical 2.5uA Melexis
	//     recommends that the SCL pin is kept low during sleep..." It also says that a wakeup request is "SCL pin high and then
	//     PWM/SDA pin low for at least tDDQ > 33ms."

	// It's not clear from sensor datasheet what should happen to SDA during sleep. Experiments show leaving SDA pulled-up high
	// results in lowest sleep current, about 2.2uA for the sensor.
	PORTC = 0 | (0<<SCL_BIT) | (1<<SDA_BIT);  // Set digit drivers all OFF. Set SCL to 0. Set SDA to 1.
	DDRC = DIGIT_DRIVER_MASK | (1<<SCL_BIT);  // Set bits 1-3 and SCL to outputs.
}

// Wake up the MLX sensor after being powered down.
static void MLXWakeUp(void)
{
#ifdef NO_SENSOR
	_delay_ms(40); // Simulate normal timing.
#else
	// Power up the TWI peripheral.
	PRR &= ~(1<<PRTWI);
	TWCR = 0;  // Set TWEN to 0 which disconnects TWI from SDA and SCL lines, returning control of those lines to GPIO interface.
	// Turn OFF the push-pull output drivers for SCL and SDA, turn ON the pullups.
	// According to MLX datasheet page 22, a wakeup request is "SCL pin high and then PWM/SDA pin low for at least tDDQ > 33ms."
	// See page 22 Figure 12. We need to set SCL to input with pullup...
	PORTC |= (1<<SCL_BIT);  // Set SCL high.
	DDRC &= ~(1<<SCL_BIT);  // Set SCL to input.
	// Now drive SDA low...
	PORTC &= ~(1<<SDA_BIT); // Set SDA low.
	DDRC |= (1<<SDA_BIT);   // Set SDA to output.
	// Wait >33ms. This causes MLX sensor to exit sleep mode.
	_delay_ms(40);
	// Set SDA to pulled-up high input.
	PORTC |= (1<<SDA_BIT);  // Set SDA high.
	DDRC &= ~(1<<SDA_BIT);  // Set SDA to input.
	// At this point, SDA and SCL should both be inputs with pullups.
	// Power up the TWI peripheral.
	PRR &= ~(1<<PRTWI);
	// Later operations that set the TWI Enable bit will cause the TWI peripheral to take control of the SCL and SDA lines.
#endif
}

// Read the RAM or EEPROM address indicated by regaddr, and return the value through the rawvalue return argument.
// Function return value is 0 if OK. No checking of return value.
static uint8_t ReadSensorN(uint8_t regaddr, uint16_t *rawvalue)
{
#ifdef NO_SENSOR
	*rawvalue = (27315+3887)/2;   // Return fake reading of 38.87 deg C.
	return 0;                     // Indicate no error.
#else
	uint8_t readbuf[3];
	uint8_t status = i2c_readReg(MLX_BUS_ADDR, regaddr, readbuf, 3);
	*rawvalue = readbuf[0] + (readbuf[1] << 8);

	uint8_t crc = crc8(0, (MLX_BUS_ADDR << 1));
	crc = crc8(crc, regaddr);
	crc = crc8(crc, (MLX_BUS_ADDR << 1) + 1);
	crc = crc8(crc, readbuf[0]);
	crc = crc8(crc, readbuf[1]);

	return status || (crc != readbuf[2]);
#endif
}

// Read sensor temperature. Same as ReadSensorN but checks the return value to see if it's indicating an error.
static uint8_t ReadSensorT(uint8_t regaddr, uint16_t *rawvalue)
{
	return ReadSensorN(regaddr, rawvalue) || ((*rawvalue & 0x8000) != 0);
}

// The following code is needed for setting the emissivity value in the sensor.

// Write a single word to MLX sensor. This is just a helper function for writeEEPROM, below.
static uint8_t I2CWriteWord(uint8_t reg, uint16_t data)
{
	uint8_t crc;
	uint8_t lsb = data & 0x00FF;
	uint8_t msb = (data >> 8);
	
	crc = crc8(0, (MLX_BUS_ADDR << 1));
	crc = crc8(crc, reg);
	crc = crc8(crc, lsb);
	crc = crc8(crc, msb);
	uint8_t writebuf[] = {lsb, msb, crc};
	
	return i2c_writeReg(MLX_BUS_ADDR, reg, writebuf, 3);  // Returns 0 if OK.
}

// Write a 16-bit parameter into the MLX sensor's EEPROM.
// Returns 0 if OK.
static uint8_t writeEEPROM(uint8_t reg, uint16_t data)
{
	// Clear out EEPROM first:
	uint8_t i2cRet = I2CWriteWord(reg, 0);
	if (i2cRet != 0) {
		return i2cRet; // If the write failed, return the error code.
	}
	_delay_ms(5 + 5); // Delay tErase + extra
	
	i2cRet = I2CWriteWord(reg, data);
	_delay_ms(5 + 5); // Delay tWrite + extra
	
	return i2cRet;
}

// The emissivity value is stored in the sensor's EEPROM as a value from 0 to 65535 with 65535 => emissivity=1.
// The desired emissivity is specified at the top of this file in the macro EMISSIVITY_X1000 as the desired emissivity * 1000.
// This calculation converts 0-1000 to 0-65535 with proper rounding. (The rounding is a bit of overkill given that the 0-65535 resolution is much finer than needed.)
// If EMISSIVITY_X1000 = 1000, the corresponding EEPROM value should be exactly 65535.
// If EMISSIVITY_X1000 = 500, the corresponding EEPROM value should be 65535/2 = 32767.5 which should round up to 32768.
static uint16_t Emissivity_X65535(uint16_t E_X1000)
{
	return ((65535UL * 2 * E_X1000) / 1000 + 1) / 2;
}

// Convert Emissivity EEPROM value back to 0-1000 value.
static uint16_t Emissivity_X1000(uint16_t E_X65535)
{
	return ((1000UL * 2 * E_X65535) / 65535 + 1) / 2;
}

// Return 0 if OK.
static uint8_t readEmissivity(uint16_t *E_X65535)
{
	return ReadSensorN(EEPROMADDR_EMISSIVITY, E_X65535);
}

// Return 0 if OK.
static uint8_t setEmissivity(uint16_t E_X65535)
{
	return writeEEPROM(EEPROMADDR_EMISSIVITY, E_X65535);
}

// Flash "ErrN" where N is the errnum. Hang here forever (until batteries are replaced).
static void ErrorHang(uint8_t errnum)
{
	error(errnum);
	BlankDisplay();
	_delay_ms(2000);
	error(errnum);
	BlankDisplay();
	_delay_ms(2000);
	while (1) {
		error(errnum);
		BlankDisplay();
		_delay_ms(20000); // Conserve the batteries.
	}
}

// Display "x E=" "n.nnn" where x is the segments code for some character and n.nnn is the emissivity in the range .000 to 1.000.
static void DispEmissivity(uint8_t digit1seg, uint16_t E_X65536)
{
	DisplayChars(digit1seg, BLANK_SEGMENTS, E_SEGMENTS, EQUALS_SEGMENTS, 2000);
	DecimalDisplayPrep(Emissivity_X1000(E_X65536), 1000, 0, 3, (1<<3));
	DisplayNms(2000);
}

// Check if sensor has been programmed to the desired emissivity. If not, attempt to program the desired value.
// If the programming attempt fails, HANG here displaying an error. We don't want to continue with normal operation if the emissivity setting is bad.
static void CheckAndSetEmissivity(bool show_info, uint16_t want_emissivity_x1000)
{
	uint16_t old_e_x65535, want_e_x65535, new_e_x65535;
	// Read the current emissivity setting.
	uint8_t status = readEmissivity(&old_e_x65535);
	if (status) {
		ErrorHang(1); // The initial read emissivity failed.
	}
	want_e_x65535 = Emissivity_X65535(want_emissivity_x1000);
	if (old_e_x65535 == want_e_x65535) {
		// Sensor already has the correct emissivity setting.
		if (show_info) {
			// Display the programmed emissivity only if the button was held down while batteries were inserted.
			DispEmissivity(BLANK_SEGMENTS, old_e_x65535);	
		}
		return;
	}
	// Program the new emissivity value.
	status = setEmissivity(want_e_x65535);
	if (status) {
		DispEmissivity(O_SEGMENTS, old_e_x65535);
		DispEmissivity(n_SEGMENTS, want_e_x65535);
		ErrorHang(2); // The write failed.
	}
	// Power cycle and verify that the sensor has the new emissivity setting.
	MLXPowerDown();
	MLXWakeUp();
	status = readEmissivity(&new_e_x65535);
	if (status) {
		ErrorHang(3); // The read after power cycle failed.
	}
	if (new_e_x65535 != want_e_x65535) {
		DispEmissivity(n_SEGMENTS, want_e_x65535);
		DispEmissivity(n_SEGMENTS, new_e_x65535);
		ErrorHang(4); // The read after power cycle got wrong value.
	}
	// Yay, reprogramming was successful!
	// Display the old and new emissivity values. This should only happen on first turn-on during manufacturing.
	DispEmissivity(O_SEGMENTS, old_e_x65535);
	DispEmissivity(n_SEGMENTS, new_e_x65535);
	// Done!
}

#define INFO_HOLD_MS 5000  // Time to display each info display, below. Can be accelerated by pushing button.

// Display various parameters, then allow user to choose an emissivity setting.
static void ChooseEmissivity(uint16_t Vcc)
{
	// Read the ambient temperature first, before displaying other stuff. We'll display it later.
	_delay_ms(500); // Give sensor some time to initialize and stabilize. May not be needed.
	uint16_t rawAmbient;
	ReadSensorT(RAMADDR_TA, &rawAmbient);
	
	// Display Vcc.
	DisplayChars(U_SEGMENTS, c_SEGMENTS, c_SEGMENTS, EQUALS_SEGMENTS, 1000);
	DisplayNumber(Vcc, 3, INFO_HOLD_MS);
	WaitButtonRelease();
	
	// Display ambient temperature.
	DisplayChars(t_SEGMENTS, A_SEGMENTS, EQUALS_SEGMENTS, BLANK_SEGMENTS, 1000);
	int16_t Ambient = Convert(rawAmbient, MAX_DEG, MIN_DEG, HUNDREDTHS, DEG_C, Vcc, 0);
	DisplayNumber(Ambient, 2, INFO_HOLD_MS);
	WaitButtonRelease();
	
	// Display skin surface temp to oral temperature offset.
	DisplayChars(O_SEGMENTS, F_SEGMENTS, S_SEGMENTS, EQUALS_SEGMENTS, 1000);
	DisplayNumber(IR_TO_ORAL_OFFSET, 2, INFO_HOLD_MS);
	WaitButtonRelease();
	
	// Read and display the emissivity setting currently programmed in the sensor.
	uint16_t old_e_x65535;
	uint8_t status = readEmissivity(&old_e_x65535);
	if (status) {
		ErrorHang(1); // The initial read emissivity failed.
	}
	DispEmissivity(O_SEGMENTS, old_e_x65535);
	WaitButtonRelease();
	
	// Display "CHG?" meaning, do you want to change emissivity? If user presses button while that's displaying,
	// proceed to offer choices. Otherwise make no change, we're done!
	_delay_ms(1000);
	if (DisplayChars(C_SEGMENTS, H_SEGMENTS, G_SEGMENTS|sDP, QUESTION_MARK_SEGMENTS, INFO_HOLD_MS)) {
	
		// Display some emissivity value, let user choose one by pressing button when one is displayed. If none chosen, leave unchanged.
		uint16_t EmissivityChoices[] = {333, 500, 800, 900, 950, 960, 970, 980, 990, 1000};
		uint16_t want_e_x1000 = 0;
		for (uint8_t i = 0; ; i++) {
			_delay_ms(500);
			uint8_t status = DisplayNumber(EmissivityChoices[i], 3, 1500);
			if (status) {
				// User pressed button. Choose this emissivity to program into sensor.
				want_e_x1000 = EmissivityChoices[i];
				break;
			}
			if (EmissivityChoices[i] == 1000) {
				break;
			}
		}
		if (want_e_x1000) {
			CheckAndSetEmissivity(true, want_e_x1000);
		} else {
			// Reassure user that emissivity hasn't changed.
			DispEmissivity(BLANK_SEGMENTS, old_e_x65535);
		}
	}
	// Done!
}

/////////////////////////////// Battery check

#define VREFPWR_BIT 1    // The voltage ref used for low battery detection is powered by PORTB bit 1.
#define ADCIN_BIT 0      // The voltage ref is applied to ADC0 input.

// The TLV431A precision shunt regulator is used as the ADC reference voltage, connected to the ADC0 pin.
// Its voltage is min: 1.215V typ: 1.240V max: 1.265V.
// (An LT1004CZ-1.2 voltage regulator was used for prototyping. Its voltage is 1.235+-.004V. Measured 1.2378V)
// The main voltage regulator is the MCP1700-3002E/TO.
// Its output voltage is min: Vin-.350V typ: 3+-.012V max: 3.09V. Measured 3.005-3.006V on test unit.
// The MLX90614-BAA and -DCH sensors are spec'd for Vcc min: 2.6V typ: 3.0V max: 3.6V.
// (However later in the datasheet under Vdd compensation there are graphs showing the sensor operated down to 2.4V,
// so Melexis is apparently being pretty conservative with the 2.6V spec.)
// We have a requirement that after giving a low battery warning, the thermometer continues to operate as the battery
// drops another .1V. So we need to give a warning at not lower than 2.6 + .1 = 2.7V. We should also stop giving readings at 2.6V
// to avoid possibly bad readings.
// "The typical VDD dependence of the ambient and object temperature is 0.6°C/V" - MLX datasheet.
// ADC absolute accuracy is spec'd as +-2LSB typical. No spec for the maximum! Datasheet p319.
#define ADC_ERROR 3  // Since there's no spec for the maximum, just make a guess!

// Some low battery performance data: With a 3 AA cell battery that is at 2.682V with just MCU running continuously,
// voltage drops to 2.667 when sensor is running. 20mA load for 1 secod drops it to 2.65V and it gradually recovers
// to 2.678 over several minutes. With battery at 2.580V, Vcc is 2.518

// If there is a possibility (accounting for voltage reference and ADC tolerances) that Vdd is less than 2.6V,
// don't display a reading because it may be inaccurate. Just display "bAt LO" and go back to sleep.
// Here we choose the ADC reading threshold for quitting (not giving a reading).
// Note that a lower ADC value indicates a higher Vdd voltage.
#ifdef ALLOW_LOW_VDD
#define ADC_LOBAT_QUIT (635 - ADC_ERROR)   // quit at 2.0V - for testing. 1024*1.24/2.0=634.8
#else
// If ADC reading is >= 1024*1.215V/2.6V - ADC_ERROR = 478 - ADC_ERROR, stop giving temperature readings.
#define ADC_LOBAT_QUIT (478 - ADC_ERROR)
#endif

// If computed average Vcc voltage is less than 2.7V (.1 greater than quit voltage), give low battery warning.
#define ADC_LOBAT_WARN_V 2700  // millivolts.
// If main voltage regulator is 3% low and reference voltage is max, the ADC reading would be 1024*1.265V/2.93V + ADC_ERROR = 445,
// which is < ADC_LOBAT_WARN. So, no danger of giving low battery warning if batteries are above 3.43V = 2.93V + .350V dropout.

// Supply power to Vref, start conversion, get reading, power down Vref.
// Prerequisite: ADC should be powered on, reference and input selected, etc.
// At 125kHz ADC clock, the conversion takes .2ms 1st time, .104ms thereafter.
static uint16_t GetADC(void)
{
#ifdef NO_ADC
	return 423;  // Typical ADC reading: 1024*1.24/3.0
#else
	// Supply power to voltage reference.
	PORTB |= 1<<VREFPWR_BIT;
	_delay_ms(1); // Wait for reference voltage to stabilize (charge the bypass cap).
	// Start conversion.
	// PRR &= ~(1<<PRADC); // Should have been done already.
	ADCSRA |= (1<<ADSC);   // Start conversion.
	// Wait for conversion complete. Takes 25 ADC clock cycles 1st time after ADC is turned on (ADEN set), 13 cycles thereafter.
	while (ADCSRA & (1<<ADSC)) ;
	// ADCL should be read before ADCH.
	uint8_t lsb = ADCL;
	PORTB &= ~(1<<VREFPWR_BIT);  // Stop powering Vref.
	return lsb | (ADCH<<8);
#endif
}
// This comment applies to using the ATmega328's internal bandgap reference (which has horrible 9% tolerance, so let's not use it).
// "When the bandgap reference voltage is used as input to the ADC, it will take a certain time for the voltage to stabilize.
// If not stabilized, the first value read after the first conversion may be wrong." This sort of implies that the second reading
// would be valid. Bandgap reference start-up time is spec'd at 70us max at Vcc=2.7V and Ta=25C.

// Initialize ADC using external voltage reference.
static void ADCInitVRef(void)
{
#ifndef NO_ADC
	DIDR0 |= 1<<ADCIN_BIT;   // Disable digital input for the analog input pin.
	PRR &= ~(1<<PRADC);      // Power up ADC and internal bandgap voltage reference.
	ADMUX = 0x40 | ADCIN_BIT; // Use AVCC (with cap on AREF pin) as reference and precision shunt regulator as the ADC input.
	
	ADCSRA = (1<<ADEN) | 3;  // Enable ADC, set ADPS field to 3 which causes ADC prescaler to divide by 8, giving an ADC
                             // clock of 125KHz. (50-200KHz is allowed.)
	// Get a first reading and throw it away to initialize the ADC. Might as well do it here since it only takes .2ms.
	GetADC();
#endif
}

static void ADCPowerDown(void)
{
#ifndef NO_ADC
	ADCSRA = 0;
#endif
}

static uint16_t GetBatADC(void)
{
	PORTB |= 1<VREFPWR_BIT;
	ADCInitVRef();             // Setup ADC so we can do low battery detection.
	_delay_ms(1);			   // Time for external voltage ref to stabilize (probably not needed).
	uint16_t adc = GetADC();   // Get initial ADC reading. adc1 = 1024*Vref/Vcc.
	ADCPowerDown();
	PORTB &= ~(1<VREFPWR_BIT); // Power down external voltage ref.
	return adc;
}

// Convert ADC value to integer millivolts.
static uint16_t ADC2mv(uint16_t adc)
{
	return (1024UL * 1240 + adc/2) / adc;
}

/////////////////////////////// Run states

typedef enum {
	STARTUP_STATE,
	SLEEP_STATE,
	READ_STATE,
	DISPLAY_STATE,
	BAT_LO_STATE,
	ERROR_STATE
} RunState;

// State that needs to be carried between the run states.
RunState state;              // The current run state. Set to the _next_ run state by the TransitionX functions.
uint16_t BatLo_Vcc_mv;       // Vcc voltage in millivolts. (Sent to BatLoState, for debugging.)
int16_t Display_To;          // Object temperature to pass to DisplayState().
int16_t Display_Ta;          // Ambient temperature to pass to DisplayState().
uint16_t Display_testVcc;    // Vcc voltage to pass to DisplayState() for low battery warning test.
uint8_t Error_num;           // Error number to pass to ErrorState().
#ifdef DEBUG
uint16_t adc00;              // First adc reading after button push, before  20mA load test.
#endif

static void TransitionStartupState(void)
{
	state = STARTUP_STATE;
}

static void TransitionSleepState(void)
{
	state = SLEEP_STATE;
}

static void TransitionReadState(void)
{
	state = READ_STATE;
}

static void TransitionDisplayState(int16_t final_To, int16_t final_Ta, uint16_t testVcc)
{
	Display_To = final_To;
	Display_Ta = final_Ta;
	Display_testVcc = testVcc;
	state = DISPLAY_STATE;
}

static void TransitionBatLoState(uint16_t Vcc_mv)
{
	BatLo_Vcc_mv = Vcc_mv;
	state = BAT_LO_STATE;
}

static void TransitionErrorState(uint8_t err_num)
{
	Error_num = err_num;
	state = ERROR_STATE;
}

///// S1.1 StartupState.
// This is called when power turns on (batteries inserted).
static void StartupState(void)
{
	// Initialize port B.
	// Bit 0 is the button input.
	// Bit 1 supplies power to voltage reference.
	// Bit 2 is unused.
	// Bit 3 is MOSI input (ISP connector to AVR).
	// Bit 4 is MISO output (AVR to ISP connector).
	// Bit 5 is SCK input (ISP connector to AVR).
	// Bit 6 is LED digit driver for L.S. digit, 1=ON.
	// Bit 7 is unused in production FW. Serial output for debugging/logging.
	// Configure least significant digit driver as an output, initially low.
	// Configure the Vref power supply pin as an output, initially low.
	// Configure the button input as an input with pullup enabled or disabled, depending on
	// whether the button pulls the signal to ground or to Vcc.
	// All others are configured as inputs with pullups. The pullups prevent the pins from
	// floating to an intermediate voltage which can cause higher current usage.
	PORTB = ~(1<<LS_DIGIT_BIT) & ~(1<<VREFPWR_BIT) & ~BUTTON_PUSHED_VALUE & ~(1<<SERIAL_OUT_BIT);
	DDRB = (1<<LS_DIGIT_BIT) | (1<<VREFPWR_BIT) | (1<<SERIAL_OUT_BIT);

	// Initialize GPIO port D - LED segment drivers. Output high = segment ON.
	PORTD = 0;   // Set segments all OFF.
	DDRD = 0xFF; // Set all bits to outputs.

	// Initialize GPIO port C.
	// Bit 0 - not used. This pin is used as ADC0 input.
	// Bits 1-3 control LED cathode drivers, with bit 3 controlling Most Significant Digit of LED display. Output high = digit ON.
	// Bit 4 is SDA to/from MLX sensor.
	// Bit 5 is SCL to MLX sensor.
	// Bit 6 is RESET# from ISP connector.
	// Bit 7 is unimplemented in this MCU.
	// PORTC and DDRC are initially 0 = floating inputs.
	// SCL and SDA are configured by MLXWakeUp and MLXPowerDown.
	PORTC = 0;                    // Cathode drivers off.
	DDRC = DIGIT_DRIVER_MASK;     // Set bits 1-3 to outputs.

	// Initialize display scanning. Do this now, before anything that might display something.
	CurrentDigit = 0;

	// Initialize button state tracking.
	OldButtonState = 0;
	NewButtonState = 0;
	uint8_t DebugState = ButtonIsPushed(); // Capture whether the button is pushed when batteries are inserted. Presently used to dump emissivity and offset settings at power-up time.
	if (DebugState) {
		WaitButtonRelease();
	}
	
	// Power saving: Disable watchdog timer. From ATmega datasheet p60-61.
	cli();
	wdt_reset();
	MCUSR &= ~(1<<WDRF); // Clear Watchdog System Reset Flag.
	// Write logical one to WDCE and WDE. Keep old prescaler setting to prevent unintentional time-out.
	WDTCSR |= (1<<WDCE) | (1<<WDE); // Watchdog Change Enable and Watchdog Enable.
	WDTCSR = 0x00;  // Turn off WDT.

	// Set TWI bit rate.
#ifndef NO_SENSOR
	i2c_init();
#endif

	// Set the sleep mode we want to use.
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	// Check battery by measuring the voltage out of the regulator (Vcc).
#ifndef NO_DISPLAY
	DecimalOn();  // Turn on 20mA load.
	_delay_ms(1000);
#endif
	uint16_t Vcc = ADC2mv(GetBatADC());
	BlankDisplay();  // Turn off load.
	
	///// MLX sensor initialization.
	
	// The MLX sensor should automatically wake up on power-up. We want to re-do the wakeup in order to get the I2C bus properly initialized
	// in preparation for following I2C operations.
	MLXWakeUp();
	
	// The MLX sensor has a minimum spec'd operating voltage of 2.6V. Don't allow user to continue if Vdd < 2.7V.
	// (Actually, it seems to work down to 2.1V or lower, but accuracy is not guaranteed!)
#ifndef ALLOW_LOW_VDD
	if (Vcc < 2700) {
		MLXPowerDown();
		while (1) {
			do {
				DisplayBatLo(Vcc);
			} while(ButtonIsPushed());
		}
	}
#endif
	// Once past this point, thermometer will operate down to ADC_LOBAT_QUIT voltage.

#ifdef CHOOSE_EMISSIVITY
	if (DebugState) {
		// Display currently programmed emissivity and let user choose a new emissivity value.
		ChooseEmissivity(Vcc);
	}
#else
	// Make sure sensor is using the right emissivity value.
	CheckAndSetEmissivity(DebugState, EMISSIVITY_X1000);
#endif

#ifdef DATALOG
	// Output some diagnostics to serial port.

	// Power supply voltage.
	PutStr("Vcc,");
	DecimalFormat(Vcc, 3, (1<<3));

	// Emissivity.
	uint16_t Emissivity;
	ReadSensorN(EEPROMADDR_EMISSIVITY, &Emissivity);
	PutStr(",e,");
	DecimalFormat(Emissivity, 5, 0); // Should be 65535 if unchanged from factory settings.
	
	// Offset.
	PutStr(",Ofs,");
	DecimalFormat(IR_TO_ORAL_OFFSET, 5, 0); // Degrees C * 100.
	
	PutChar('\n');
#endif

	// Almost done with initialization! Display some warnings if we have non-standard firmware.
#ifdef DEBUG
	// Try to avoid the embarrassment of having debug FW in production devices by displaying "dEb" for a few seconds
	// after inserting battery.
	DisplayDeb();
	BlankDisplay();
#endif
#ifdef NO_ADC
	// Display "-ADC" to warn that ADC code has been stubbed out.
	Display_ADC();
	BlankDisplay();
#endif
#ifdef NO_SENSOR
	// Display "-SEn" to warn that sensor code has been stubbed out.
	Display_SENSOR();
	BlankDisplay();
#endif

	MLXPowerDown();
	
	// Give low battery warning if Vcc is even slightly out of regulation.
	if (Vcc < 2940) {  // Display low battery warning if Vcc is 2% low.
		// T1.3 Transition to BatLoState.
		TransitionBatLoState(Vcc);
		return;
	}

	// T1.1 Transition to SleepState.
	TransitionSleepState();
}

///// S1.2 SleepState.
// Sleep while waiting for button push.
static void SleepState(void)
{
	BlankDisplay();
	// MLXPowerDown();  // DO NOT do this here, because Vcc may be too low to properly shut down sensor.
	ADCPowerDown();
	PCIFR = (1<<PCIF0); // Clear Pin Change Interrupt Flag 0.
	while (ButtonIsPushed() == 0) {
		// Power down as much stuff as we can while sleeping.
		PRR = (1<<PRTWI)  // Turn off power to Two Wire Interface
		| (1<<PRTIM2)     // Counter/timer 2
		| (1<<PRTIM0)     // Counter/timer 0
		| (1<<PRTIM1)     // Counter/timer 1
#if defined(NDEBUG) // Disable iff not using debugWire.
		| (1<<PRSPI)      // SPI interface - this must stay powered up if using debugWire for programming or debugging.
#endif
		| (1<<PRUSART0)   // USART 0
		| (1<<PRADC);     // ADC

#ifdef AUTORUN
		_delay_ms(SLEEP_TIME_MS);
		break;           // Exit the button wait loop.
#else
		// Set up to wake up on button push. Button is connected to PCINT0 pin which can trigger PCI0 interrupt.
		PCMSK0 = (1<<PCINT0); // Enable interrupts from PCINT0 pin change.
		PCICR = (1<<PCIE0);   // Pin Change Interrupt Enable 0 - enable PCI0 interrupt.
		sei();                // Global interrupt enable. REQUIRED in order for wakeup to occur.
		sleep_mode();         // Go to sleep (power down mode). Wake up on pin change PCINT0.
		// Typical (at 25 deg C) current draw from battery during sleep should be:
		// .15uA    (Optional) LM66100 ideal diode.
		// 1.7uA    MCP1700 Voltage regulator with battery at 4.5V.
		// 0.1uA    ATmega328P in power down mode, all peripherals disabled and WDT disabled.
		// 2.5uA    MLX90614Bxx, Dxx sensor.
		// 4.45uA   TOTAL. Actual measured 5.2uA before cleaning, 4.0uA after cleaning.
#endif
	}
	cli();                    // Stop responding to interrupts.
	
	// T1.4 Transition to ReadState.
	TransitionReadState();
}

///// S1.3 ReadState.
// Read power supply voltage and object temperature, calculate temperature to be reported to user.
static void ReadState()
{
#ifdef DEBUG
	uint16_t adc00 = GetBatADC(); // Get initial ADC reading. adc00 = 1024*Vref/Vcc.
#endif
	DecimalOn();                  // Turn on a decimal point to create a 20mA load for checking battery level.
	_delay_ms(INITIAL_LOAD_MS);
	uint16_t adc0 = GetBatADC();  // Get initial ADC reading. adc0 = 1024*Vref/Vcc.
	BlankDisplay();				  // Turn off the load.

	// If there is any possibility (accounting for measurement errors) that Vcc is below minimum operating voltage of
	// sensor, don't give a bad reading, DON'T EVEN WAKE UP SENSOR! Just flash "bAt" "LO" then go back to sleep.
	if (adc0 > ADC_LOBAT_QUIT) {
		// T1.10 Transition to BatLoState.
		TransitionBatLoState(ADC2mv(adc0));
		return;
	}

	// Wake up the sensor.
	MLXWakeUp();  // Takes about 40ms holding SDA low. Battery can recover from the 20mA load during this time.
		
	// Read the Vcc voltage again, this time without the added 20mA load. Main purpose is to get the voltage as the sensor begins its operations (calibration
	// and then continuously reading temperatures). Note that GetBatADC still turns on shunt regulator (about 2mA) and waits about 1ms
	// before reading Vcc voltage.
	uint16_t adc1 = GetBatADC();  // Get initial ADC reading. adc1 = 1024*Vref/Vcc.

	// It's unlikely the battery check would fail here, but might as well check anyway...
	// If there is any possibility (accounting for measurement errors) that Vcc is below minimum operating voltage of
	// sensor, don't give a bad reading. Just try to power down sensor, flash "bAt" "LO" and go back to sleep.
	if (adc1 > ADC_LOBAT_QUIT) {
		// T1.10 Transition to BatLoState.
		TransitionBatLoState(ADC2mv(adc1));
		return;
	}

	// Wait for sensor to acquire a stable temperature reading.
		
	// According to MLX data sheet, "After wake up the first data is available after 0.25 seconds (typ)."
	// It's not clear how that relates to the IIR filter "settling times" shown below:
	// MLX90614BAA sensor has settling time of .10 seconds with factory settings.
	// MLX90614DCH sensor has settling time of .65 seconds with factory settings.
	// /////Let's wait at least .25 seconds. Multiply that by 1.11 to allow for 10% fast MCU clock. So 278ms.
	// Due to readings ~.8C too high at ambient and too high at target=38C, let's try waiting longer. 
	// Let's wait at least .65 seconds. Multiply that by 1.11 to allow for 10% fast MCU clock. So 730ms.
	// Actual tests showed that readings were stable at ~100ms!
	
	_delay_ms(READING_WAIT_MS);

	// Read the sensor!
	uint16_t raw_temp_value;
	if (ReadSensorT(RAMADDR_TOBJ1, &raw_temp_value)) {
		// Most likely cause of this error is missing or broken MLX sensor.
		// Might also be due to low Vcc.
		_delay_ms(1000); // In case of low battery, give a bit of time for battery to recover.
		MLXPowerDown();  // Since sensor (should be) powered up now, try to put sensor to sleep.
		// T1.2 Transition to ErrorState.
		TransitionErrorState(1);
		return;
	}

	uint16_t raw_Ta_value;
#if defined(DISPLAY_Ta) || defined(DATALOG)
	// Read ambient temperature.
	if (ReadSensorT(RAMADDR_TA, &raw_Ta_value)) {
		// Most likely cause of this error is missing or broken MLX sensor.
		// Might also be due to low Vcc.
		MLXPowerDown();  // Since sensor (should be) powered up now, try to put sensor to sleep.
		// T1.2 Transition to ErrorState.
		TransitionErrorState(1);
		return;
	}
#endif

	// Check the Vcc voltage again.
	uint16_t adc2 = GetBatADC();

	// It's unlikely the battery check would fail here, but might as well check anyway...
	// If there is any possibility (accounting for measurement errors) that Vcc is below minimum operating voltage of
	// sensor, don't give a bad reading. Just try to power down sensor, flash "bAt" "LO" and go back to sleep.
	if (adc2 > ADC_LOBAT_QUIT) {
		// T1.10 Transition to BatLoState.
		TransitionBatLoState(ADC2mv(adc2));
		return;
	}

	// Power down the sensor now, BEFORE we start displaying stuff.
	// (Display can drag down battery to the point that we can't shut down the sensor properly!)
	MLXPowerDown();

	uint16_t V1 = ADC2mv(adc1); // Convert ADC readings to millivolts.
	uint16_t V2 = ADC2mv(adc2);
	uint16_t avgVcc = (V1 + V2 + 1) / 2;

#ifdef DATALOG
	// Send voltages to serial output.
	PutStr("adc00,");
	DecimalFormat(adc00, 4, 0);
	PutStr(",V00,");
	DecimalFormat(ADC2mv(adc00), 4, 3);
	PutStr(",V0,");
	DecimalFormat(ADC2mv(adc0), 4, 3);
	PutStr(",V1,");
	DecimalFormat(V1, 4, 3);
	PutStr(",V2,");
	DecimalFormat(V2, 4, 3);
	PutStr(",avgVcc,");
	DecimalFormat(avgVcc, 4, 3);
	
	PutStr("\nrawTo,");
	DecimalFormat(raw_temp_value, 5, 2);
#endif
	
	// Prepare to display the object temperature reading.
	int16_t final_To = Convert(raw_temp_value, MAX_DEG, MIN_DEG, DECIMALS, SCALE, avgVcc, IR_TO_ORAL_OFFSET);
	
	int16_t final_Ta = 0;
#if defined(DISPLAY_TA) || defined(DATALOG)
 #ifdef DATALOG
	PutStr("\nrawTa,");
	DecimalFormat(raw_Ta_value, 5, 2);
 #endif	
	// Prepare to display the ambient temperature reading.
	final_Ta = Convert(raw_Ta_value, MAX_DEG, MIN_DEG, DECIMALS, SCALE, avgVcc, 0);
#endif

	// T1.5 Transition to DisplayState.
	TransitionDisplayState(final_To, final_Ta, V2);
	return;
}

///// S1.4 DisplayState.
// Display temperature for HOLD_TIME_MS milliseconds.
// Continue to display if the button is held down continuously.
// Exit early if user releases and re-presses button (indicating they want a new reading).
static void DisplayState(int16_t To, int16_t Ta, uint16_t testVcc)
{
#ifdef DISPLAY_TA
	// Display ambient temperature briefly.
	// Binary to decimal convert and send to display buffer.
	DecimalDisplayPrep(Ta, MAX_DEG, MIN_DEG, DECIMALS+1, 1<<DECIMALS);
	_delay_ms(200);
	DisplayNms(1000);
#endif
	
#ifdef DISP988
	// For battery life test, always display 98.8 for consistency.
	DecimalDisplayPrep(988, MAX_DEG, MIN_DEG, DECIMALS+1, 1<<DECIMALS);
#else
	// Binary to decimal convert and send to display buffer.
	DecimalDisplayPrep(To, MAX_DEG, MIN_DEG, DECIMALS+1, 1<<DECIMALS);
#endif
	
	// Illuminate display.
	if (DisplayNms(HOLD_TIME_MS)) {
		// We are here because user released and then pressed button again which we interpret to mean
		// they want a new reading.
		// T1.11 Transition to ReadState.
		TransitionReadState();
		return;
	}
	// If NewButtonState is 1, user has been holding the button down all this time. Continue displaying the temperature.
	while (NewButtonState) {
		DisplayNms(100);
	}

	if (testVcc <= ADC_LOBAT_WARN_V) {
		// T1.6 Transition to LoBatState.
		TransitionBatLoState(testVcc);
		return;
	}

	// T1.8 Transition to SleepState.
	TransitionSleepState();
}

///// S1.5 BatLoState.
// Display "bAt" "LO" briefly, then transition to SleepState.
static void BatLoState(uint16_t Vcc_mv)
{
	DisplayBatLo(Vcc_mv);
	// T1.8 Transition to SleepState.
	TransitionSleepState();
}

///// S1.6 ErrorState.
// Display "ErrN" briefly (where "N" is the value of the err_num argument), then transitions to SleepState.
static void ErrorState(uint8_t err_num)
{
	error(err_num);
	// T1.9 Transition to SleepState.
	TransitionSleepState();
}

/////////////////////////////// Thermometer main()

int main(void)
{
	TransitionStartupState();  // Start the state machine in the STARTUP_STATE.
	
	// The main loop is implemented as a state machine. Each xState function sets the state to goto next.
	while (true) {
		switch (state) {
			
			case STARTUP_STATE:
				StartupState();
				break;
			
			case SLEEP_STATE:
				SleepState();
				break;
				
			case READ_STATE:
				ReadState();
				break;
				
			case DISPLAY_STATE:
				DisplayState(Display_To, Display_Ta, Display_testVcc);
				break;
			
			case BAT_LO_STATE:
				BatLoState(BatLo_Vcc_mv);
				break;
			
			case ERROR_STATE:
				ErrorState(Error_num);
				break;

		}
	}
}
