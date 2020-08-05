/*
 * M4C_Thermometer.c
 *
 * Created: 6/9/2020 10:18:34 PM
 * Author : Jan Kok
 */ 

// F_CPU should be defined in the project settings so the definition gets passed into the compiler command line.
//#define F_CPU 1000000    // CPU clock frequency, Hz.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
//#include "board.h"
//#include "sysclk.h"
//#include "twi_master.h"
#include "i2c_master.h"

#ifdef NDEBUG

// ***** Set parameters for Release (non-debug) firmware here: *****
#define SCALE DEG_F    // Set this to DEG_F or DEG_C to select which temperature scale to display readings.
#define MAX_DEG 1099   // Above 109.9 deg F is displayed as "HI"
#define MIN_DEG 600    // Below 60.0 deg F is displayed as "LO"
#define HOLD_TIME_MS 5000  // How long (ms) to display temperature reading if button is not held down.

#else

// Set parameters for DEBUG firmware.
#define SCALE DEG_C    // Always use DEG_C while debugging.
#define MAX_DEG 4999   // Above 49.99 deg C is displayed as "HI"
#define MIN_DEG -999   // Below -9.99 deg C is displayed as "LO"
#define HOLD_TIME_MS 2000  // How long (ms) to display temperature reading if button is not held down.
//#define NO_ADC         // Optionally define this to debug ADC current draw issues.
//#define NO_SENSOR      // Optionally define this to debug MLX sensor current draw or other issues.

#endif

/////////////////////////////// Button check function

#define BUTTON_BIT    0   // Bit number in port B for button input line.

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
	NewButtonState = (PINB & (1<<BUTTON_BIT)) != 0;
	return NewButtonState;
}

// Check and update button state. Return 1 if button has just transitioned to pushed state.
static int8_t ButtonJustPushed(void)
{
	ButtonIsPushed();
	return NewButtonState && !OldButtonState;
}

#if 0 // not needed?
// Check and update button state. Return 1 if button has just transitioned to released state.
static int8_t ButtonJustReleased(void)
{
	ButtonIsPushed();
	return OldButtonState && !NewButtonState;
}
#endif

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
#define BLANK_SEGMENTS 0               // blank
#define MINUS_SIGN_SEGMENTS sG         // -
#define EXCLAMATION_POINT_SEGMENTS (sB|sDP) // !
#define A_SEGMENTS (sA|sB|sC|sE|sF|sG) // A
#define b_SEGMENTS (sC|sD|sE|sF|sG)    // b
#define C_SEGMENTS (sA|sD|sE|sF)       // C
#define d_SEGMENTS (sB|sC|sD|sE|sG)    // d
#define E_SEGMENTS (sA|sD|sE|sF|sG)    // E
#define H_SEGMENTS (sB|sC|sE|sF|sG)    // H
#define I_SEGMENTS (sB|sC)             // I
#define L_SEGMENTS (sD|sE|sF)          // L
#define n_SEGMENTS (sC|sE|sG)          // n
#define O_SEGMENTS (sA|sB|sC|sD|sE|sF) // O
#define S_SEGMENTS (sA|sC|sD|sF|sG)    // S
#define t_SEGMENTS (sD|sE|sF|sG)       // t

// This array determines which segments are turned on for each digit. Index 0 is LSDigit.
int8_t DisplaySegments[DISP_N_DIGITS];

// Which digit is currently being displayed.
int8_t CurrentDigit;

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
        }
        if (num < min_num) {
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
    int8_t minus = num < 0;
    if (minus) {
        num = -num;
    }
    // Convert num to decimal digits and set up DisplaySegments.
    int8_t digit_num = 0; // Digit position we are processing. 0 = L.S. digit.
    do {
        int8_t digit_val = num % 10;
        int8_t segments = DigitToSegments[digit_val];
        if (digit_num >= force_digits) {
            // Handle leading 0 blanking and minus sign if appropriate.
            if (num == 0) {
                // We're in the leading zeros region. Display a blank or a minus sign as appropriate.
                if (minus) {
                    segments = MINUS_SIGN_SEGMENTS;
                    minus = 0; // Don't output any more minus signs.
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
	// Enable next digit.
	TurnOnDigit(CurrentDigit);
	PORTD = DisplaySegments[CurrentDigit];
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
			return 1;
		}
	}
	return 0;
}

// Turn on a decimal point for the given time. This is for creating a load on battery and as a side-effect gives some feedback to user that button was pushed.
static void DecimalOn(uint16_t ms)
{
	TurnOnDigit(1);
	PORTD = sDP;
	_delay_ms(ms);
}

// Let there not be light!
static void BlankDisplay(void)
{
    PORTD = BLANK_SEGMENTS;
    TurnOffDigits();
}

#ifdef DEBUG
// Display an integer (no decimal point) for disp_ms milliseconds.
static uint8_t DisplayInt(int16_t value, uint16_t disp_ms)
{
	DecimalDisplayPrep(value, 9999, -999, 1, 0);
	return DisplayNms(disp_ms);
}
#endif

// In the (hopefully) highly unlikely event that something goes wrong, e.g. I2C gets hung,
// just display an "E" for a few seconds in the indicated digit, then continue.
static void error(uint8_t digit)
{
	TurnOffDigits();        // Turn off all digit drivers.
	TurnOnDigit(digit);     // Turn on selected digit.
	PORTD = E_SEGMENTS;
	_delay_ms(1000);
	BlankDisplay();
}

#if DEBUG
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
	int8_t i = DISP_N_DIGITS-1;
	DisplaySegments[i--] = b_SEGMENTS;
	DisplaySegments[i--] = A_SEGMENTS;
	DisplaySegments[i--] = t_SEGMENTS;
	DisplaySegments[i--] = BLANK_SEGMENTS;
	if (DisplayNms(1000)) {
		return 1;
	}
	i = DISP_N_DIGITS-1;
	DisplaySegments[i--] = L_SEGMENTS;
	DisplaySegments[i--] = O_SEGMENTS;
	DisplaySegments[i--] = BLANK_SEGMENTS;
	if (DisplayNms(1000)) {
		return 1;
	}

#ifdef DEBUG
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
	int8_t i = DISP_N_DIGITS-1;
	DisplaySegments[i--] = d_SEGMENTS;
	DisplaySegments[i--] = E_SEGMENTS;
	DisplaySegments[i--] = b_SEGMENTS;
	DisplaySegments[i--] = BLANK_SEGMENTS;
	return DisplayNms(2000);
}
#endif

#ifdef NO_ADC
// Display "-AdC" for a few seconds to indicate that ADC code is stubbed out.
// Return 1 immediately if button becomes pressed while displaying. Otherwise return 0 when done displaying.
static uint8_t Display_ADC(void)
{
	int8_t i = DISP_N_DIGITS-1;
	DisplaySegments[i--] = MINUS_SIGN_SEGMENTS;
	DisplaySegments[i--] = A_SEGMENTS;
	DisplaySegments[i--] = d_SEGMENTS;
	DisplaySegments[i--] = C_SEGMENTS;
	return DisplayNms(2000);
}
#endif

#ifdef NO_SENSOR
// Display "-SEn" for a few seconds to indicate that sensor code is stubbed out.
// Return 1 immediately if button becomes pressed while displaying. Otherwise return 0 when done displaying.
static uint8_t Display_SENSOR(void)
{
	int8_t i = DISP_N_DIGITS-1;
	DisplaySegments[i--] = MINUS_SIGN_SEGMENTS;
	DisplaySegments[i--] = S_SEGMENTS;
	DisplaySegments[i--] = E_SEGMENTS;
	DisplaySegments[i--] = n_SEGMENTS;
	return DisplayNms(2000);
}
#endif

////////////////////////////// TWI (a.k.a. I2C) stuff

#define MLX_BUS_ADDR        0       // TWI slave bus address for MLX90614xxx IR temperature sensor.
#define SDA_BIT             4       // Bit number in port C for SDA line.
#define SCL_BIT             5       // Bit number in port C for SCL line.

// Register addresses in MLX chip.
#define RAMADDR_TA 6                  // Address from which to read ambient (sensor's own) temperature.
#define RAMADDR_TOBJ1 7               // Address from which to read object (person's) temperature.
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

static uint16_t ReadSensor(uint8_t regaddr, uint16_t *rawvalue)
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

	return status || (crc != readbuf[2]) || ((*rawvalue & 0x8000) != 0);
#endif
}

/////////////////////////////// Data conversion stuff

#define TENTHS 1
#define HUNDREDTHS 2
#define DEG_F 1
#define DEG_C 0

// Convert raw temperature reading from sensor to tenths or hundredths
// of a degree C or F and set up to display it.
static void Convert(uint16_t raw, int16_t max_deg, int16_t min_deg, uint8_t decimals, uint8_t scale, int32_t avgVcc)
{
	int32_t value = raw * 2; // Temp in centiK.
	value -= 27315;          // Temp in centidegrees C.
	// Compensate for variance in Vcc from the standard 3V.
	// "The typical VDD dependence of the ambient and object temperature is 0.6°C/V" - MLX datasheet.
	value -= ((avgVcc - 3000) * 60 + 500) / 1000;
	
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
	// Binary to decimal convert and send to display buffer.
	DecimalDisplayPrep(value, max_deg, min_deg, decimals+1, 1<<decimals);
}

/////////////////////////////// Battery check

#define VREFPWR_BIT 2    // The voltage ref used for low battery detection is powered by PORTB bit 2.
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
#ifdef NDEBUG
// If ADC reading is >= 1024*1.215V/2.6V - ADC_ERROR = 478 - ADC_ERROR, stop giving temperature readings.
#define ADC_LOBAT_QUIT (478 - ADC_ERROR)
#else
#define ADC_LOBAT_QUIT (603 - ADC_ERROR)   // quit at 2.1V - for testing
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
	return 375;  // Typical ADC reading: 1024*1.1/3.0
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


// What to do if Vcc is found to be below MLX sensor's min operating voltage AFTER sensor has been powered up.
static void LoBatShutdown(uint16_t adc)
{
	_delay_ms(1000); // Try to give some time for battery to recover.
	MLXPowerDown();  // Try to power down sensor.
	DisplayBatLo(ADC2mv(adc));
}

/////////////////////////////// Thermometer main()

int main(void)
{
	// Initialize port B.
	// Bit 0 is the button input, high = button pressed.
	// Bit 1 is unused.
	// Bit 2 supplies power to voltage reference.
	// Bit 3 is MOSI input (ISP connector to AVR).
	// Bit 4 is MISO output (AVR to ISP connector).
	// Bit 5 is SCK input (ISP connector to AVR).
	// Bit 6 is LED digit driver for L.S. digit, 1=ON.
	// Bit 7 is unused.
	PORTB = 0;   // Set all output bits low.
	DDRB = 0xEF; // Set bit 0 to input, all others to outputs.

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

	// Set TWI bit rate.
#ifndef NO_SENSOR
	i2c_init();
#endif
	// The MLX sensor wakes up on power-up. But we want to re-do the wakeup in order to get the I2C bus properly initialized
	// in preparation for next MLXPowerDown.
	MLXWakeUp();
	MLXPowerDown();

	// Power saving: Disable watchdog timer. From ATmega datasheet p60-61.
	cli();
	wdt_reset();
	MCUSR &= ~(1<<WDRF); // Clear Watchdog System Reset Flag.
	// Write logical one to WDCE and WDE. Keep old prescaler setting to prevent unintentional time-out.
	WDTCSR |= (1<<WDCE) | (1<<WDE); // Watchdog Change Enable and Watchdog Enable.
	WDTCSR = 0x00;  // Turn off WDT.

	// Set the sleep mode we want to use.
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
#ifdef DEBUG
	// Try to avoid the embarrassment of having debug FW in production devices by displaying "dEb" for a few seconds
	// after inserting battery.
	// Also, set DebugMode to 1 if button is pressed during "dEb" display. This allows additional debug modes, for example
	// for battery life testing.
	uint8_t DebugMode = DisplayDeb();
	BlankDisplay();
#endif
#ifdef NO_ADC
	// Display "-ADC" to warn that ADC code has been stubbed out.
	Display_ADC();
#endif
#ifdef NO_SENSOR
	// Display "-SEn" to warn that sensor code has been stubbed out.
	Display_SENSOR();
#endif

	// Check battery, give low battery warning if Vcc is even slightly out of regulation.
	DecimalOn(1000);  // Turn on 20mA load for 1 second.
	uint16_t batv = ADC2mv(GetBatADC());
	BlankDisplay();  // Turn off load.
	if (batv < 2940) {  // Display warning if Vcc is 2% low.
		DisplayBatLo(batv);
	}

	// Main loop. Sleep and wait for button push, take reading, display for a few seconds or until button is released.
	while(1) {
		
		// Sleep while waiting for button push.
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
		}
		cli();                    // Stop responding to interrupts.

		// The button has been pushed! Let's get busy!
		DecimalOn(100);               // Turn on a decimal point to create a 20mA load for checking battery level.
		uint16_t adc1 = GetBatADC();  // Get initial ADC reading. adc1 = 1024*Vref/Vcc.
		BlankDisplay();				  // Turn off the load.
		
		// If there is any possibility (accounting for measurement errors) that Vcc is below minimum operating voltage of
		// sensor, don't give a bad reading, DON'T EVEN WAKE UP SENSOR! Just flash "bAt" "LO" and go back to sleep.
		if (adc1 > ADC_LOBAT_QUIT) {
			DisplayBatLo(ADC2mv(adc1));
			continue;
		}

		// Wake up the sensor.
		MLXWakeUp();  // Takes about 40ms.
		
		// Read the Vcc voltage again, this time without the added 20mA load. Main purpose is to get the voltage as the sensor begins it's operations (calibration
		// and then continuously reading temperatures). Note that GetBatADC still turns on shunt regulator (about 2mA) and waits about 1ms
		// before reading Vcc voltage.
		adc1 = GetBatADC();  // Get initial ADC reading. adc1 = 1024*Vref/Vcc.

		// It's unlikely the battery check would fail here, but might as well check anyway...
		// If there is any possibility (accounting for measurement errors) that Vcc is below minimum operating voltage of
		// sensor, don't give a bad reading. Just try to power down sensor, flash "bAt" "LO" and go back to sleep.
		if (adc1 > ADC_LOBAT_QUIT) {
			LoBatShutdown(adc1);
			continue;
		}
		
		// According to MLX data sheet, "After wake up the first data is available after 0.25 seconds (typ)."
		// It's not clear how that relates to the IIR filter "settling times" shown below:
		// MLX90614BAA sensor has settling time of .10 seconds with factory settings.
		// MLX90614DCH sensor has settling time of .65 seconds with factory settings.
		// Let's wait at least .25 seconds. Multiply that by 1.11 to allow for 10% fast MCU clock. So 278ms
		_delay_ms(278);

		// Read the sensor!
		uint16_t raw_temp_value;
		if (ReadSensor(RAMADDR_TOBJ1, &raw_temp_value)) {
			// Most likely cause of this error is missing or broken MLX sensor.
			// Might also be due to low Vcc.
			_delay_ms(1000); // In case of low battery, give a bit of time for battery to recover.
			MLXPowerDown();  // Since sensor (should be) powered up now, try to put sensor to sleep.
			error(1);
			continue;
		}

		// Check the Vcc voltage again.
		uint16_t adc2 = GetBatADC();

		// It's unlikely the battery check would fail here, but might as well check anyway...
		// If there is any possibility (accounting for measurement errors) that Vcc is below minimum operating voltage of
		// sensor, don't give a bad reading. Just try to power down sensor, flash "bAt" "LO" and go back to sleep.
		if (adc2 > ADC_LOBAT_QUIT) {
			LoBatShutdown(adc2);
			continue;
		}

#ifdef DEBUG		
		// Read the ambient (sensor's) temperature.
		uint16_t amb_temp_value;
		if (ReadSensor(RAMADDR_TA, &amb_temp_value)) {
			// Most likely cause of this error is missing or broken MLX sensor.
			// Might also be due to low Vcc.
			_delay_ms(1000); // In case of low battery, give a bit of time for battery to recover.
			MLXPowerDown();  // Since sensor (should be) powered up now, try to put sensor to sleep.
			error(1);
			continue;
		}
		
		_delay_ms(1500); // See if there is some kind of settling.
		
		// Read the object temperature again
		uint16_t raw_temp_value2;
		if (ReadSensor(RAMADDR_TOBJ1, &raw_temp_value2)) {
			// Most likely cause of this error is missing or broken MLX sensor.
			// Might also be due to low Vcc.
			_delay_ms(1000); // In case of low battery, give a bit of time for battery to recover.
			MLXPowerDown();  // Since sensor (should be) powered up now, try to put sensor to sleep.
			error(1);
			continue;
		}

		// Read the ambient (sensor's) temperature again.
		uint16_t amb_temp_value2;
		if (ReadSensor(RAMADDR_TA, &amb_temp_value2)) {
			// Most likely cause of this error is missing or broken MLX sensor.
			// Might also be due to low Vcc.
			_delay_ms(1000); // In case of low battery, give a bit of time for battery to recover.
			MLXPowerDown();  // Since sensor (should be) powered up now, try to put sensor to sleep.
			error(1);
			continue;
		}
		
#endif

		// Power down the sensor now, BEFORE we start displaying stuff.
		// (Display can drag down battery to the point that we can't shut down the sensor properly!)
		MLXPowerDown();

#ifdef DEBUG
		int16_t sag = adc1 - adc2;
		DisplayInt(sag, 300);
#endif
		
		adc1 = ADC2mv(adc1); // Convert ADC readings to millivolts.
		adc2 = ADC2mv(adc2);
		uint16_t avgVcc = (adc1 + adc2 + 1) / 2;
#if DEBUG
		// Display Vcc in volts.
		DisplayVolts(avgVcc);
#endif

		// Prepare to display the temperature reading.
#ifdef DEBUG
#define DECIMALS HUNDREDTHS  // In DEBUG, display temperature as xx.xx (deg C)
#else
#define DECIMALS TENTHS      // In Release, display temperature as [1]xx.x in deg F or C, depending on SCALE
#endif

#ifdef DEBUG
		// Display ambient (sensor's) temperature.
		Convert(amb_temp_value, MAX_DEG, MIN_DEG, DECIMALS, SCALE, adc2);
		DisplayNms(1000);
		DisplayInt(amb_temp_value2 - amb_temp_value, 1000);
		DisplayInt(raw_temp_value2 - raw_temp_value, 1000);
#endif

		Convert(raw_temp_value, MAX_DEG, MIN_DEG, DECIMALS, SCALE, avgVcc);

		// Illuminate display for HOLD_TIME_MS milliseconds.
		if (DisplayNms(HOLD_TIME_MS)) {
			// We are here because user released and then pressed button again which we interpret to mean
			// they want a new reading.
			continue; // Go back around the main loop.
		}
		// If NewButtonState is 1, user has been holding the button down all this time. Continue displaying the temperature.
		while (NewButtonState) {
			DisplayNms(100);
		}

		if (avgVcc <= ADC_LOBAT_WARN_V) {
			DisplayBatLo(avgVcc);
			continue;
		}

	}  // Main while loop.
}
