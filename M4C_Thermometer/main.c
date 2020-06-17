/*
 * M4C_Thermometer.c
 *
 * Created: 6/9/2020 10:18:34 PM
 * Author : Jan Kok
 */ 

#define F_CPU 1000000    // CPU clock frequency, Hz.
#include <avr/io.h>
#include <util/delay.h>

////////////////////////  Display stuff

// Number of digits in 7-segment display.
#define DISP_N_DIGITS 4

// Largest number that can be displayed w/o error indication.
#define DISP_MAX_INT 9999

// Largest number that can be displayed w/o error indication for diagnostics.
// 10999 would be displayed as 0999.
#define DISP_MAX_INT_DIAG 15999

// Smallest number that can be displayed w/o error indication
#define DISP_MIN_INT -999

// Define which bit controls which segment (A-G & decimal point)
#define sA (1<<2)
#define sB (1<<0)
#define sC (1<<3)
#define sD (1<<7)
#define sE (1<<6)
#define sF (1<<4)
#define sG (1<<1)
#define sDP (1<<5)

// This table defines which segments of 7-segment display are enabled for each digit 0-9.
const int8_t DigitToSegments[] = {
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
#define H_SEGMENTS (sB|sC|sE|sF|sG)    // H
#define I_SEGMENTS (sB|sC)             // I
#define L_SEGMENTS (sD|sE|sF)          // L
#define O_SEGMENTS (sA|sB|sC|sD|sE|sF) // O

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
void DecimalDisplayPrep(int16_t num, int16_t max_num, int16_t min_num, int8_t force_digits, int8_t decimal_points)
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


void DisplayScanStep(void)
{
	// Turn all the segments OFF before changing digit
	PORTD = BLANK_SEGMENTS;
	// Step to next digit.
	if (--CurrentDigit < 0) {
		CurrentDigit = DISP_N_DIGITS - 1;
	}
	// Enable next digit.
	PORTC = 1<<CurrentDigit; // ASSUMES no other outputs on port C!
	PORTD = DisplaySegments[CurrentDigit];
}

// Let there not be light!
void BlankDisplay(void)
{
    PORTD = BLANK_SEGMENTS;
    PORTC = 0;
}

int mainx(void)
{
    // Initialize GPIO port C.
    // Bits 0-3 control LED cathode drivers, with bit 0 controlling Least Significant Digit of LED display. Output high = digit ON.
    PORTC = 0;   // Set digit drivers all OFF.
    DDRC = 0xF;  // Set bits 0-3 to outputs.
    // Initialize GPIO port D - LED segment drivers. Output high = segment ON.
    PORTD = 0;   // Set segments all OFF.
    DDRD = 0xFF; // Set all bits to outputs.
	// Initialize display scanning.
    CurrentDigit = 0;
		
	while(1) {
        DecimalDisplayPrep(988, DISP_MAX_INT_DIAG, DISP_MIN_INT, 2, 1<<1);
        for (int16_t i = 0; i<1000; i++) { // .5 seconds
			DisplayScanStep();
			_delay_ms(5);
        }
        BlankDisplay();
        _delay_ms(15000);
        
		// Main loop. Display counter.
		//for (int16_t i = -1999; i <= 19999; i++)
		//{
		//	DecimalDisplayPrep(i, DISP_MAX_INT_DIAG, DISP_MIN_INT, 2, 1<<2);
		//	for (int8_t t = 0; t < 99; t++) {
		//		DisplayScanStep();
		//		_delay_us(10);
		//	}
		//}
	}
}
