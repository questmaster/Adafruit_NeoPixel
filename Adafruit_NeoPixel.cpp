/*-------------------------------------------------------------------------
  Arduino library to control a wide variety of WS2811- and WS2812-based RGB
  LED devices such as Adafruit FLORA RGB Smart Pixels and NeoPixel strips.
  Currently handles 400 and 800 KHz bitstreams on 8, 12 and 16 MHz ATmega
  MCUs, with LEDs wired for various color orders.  Handles most output pins
  (possible exception with upper PORT registers on the Arduino Mega).

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries,
  contributions by PJRC, Michael Miller and other members of the open
  source community.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoPixel library.

  NeoPixel is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoPixel is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include "Adafruit_NeoPixel.h"
#include "string.h"
#include "stm32f4xx_hal.h"


// ---

#define __SAMD21G18A__
// 48MHz

#define INPUT false
#define OUTPUT true
void pinMode(int8_t pin, bool direction_out)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // works only for GPIOA!

	GPIO_InitTypeDef gpioDef = {
		1U << pin,
		direction_out ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT,
		GPIO_PULLUP,
		GPIO_SPEED_FREQ_HIGH,
		0u
	};
	HAL_GPIO_Init(GPIOA, &gpioDef);
}
#define LOW true
#define HIGH false
void digitalWrite(int8_t pin, bool level_low)
{
	HAL_GPIO_WritePin(GPIOA, 1U << pin, level_low? GPIO_PIN_RESET : GPIO_PIN_SET );
}

// ---

// Constructor when length, pin and type are known at compile-time:
Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t n, uint8_t p, neoPixelType t) :
  begun(false), brightness(0), pixels(NULL), endTime(0)
{
  updateType(t);
  updateLength(n);
  setPin(p);
}

// via Michael Vogt/neophob: empty constructor is used when strand length
// isn't known at compile-time; situations where program config might be
// read from internal flash memory or an SD card, or arrive via serial
// command.  If using this constructor, MUST follow up with updateType(),
// updateLength(), etc. to establish the strand type, length and pin number!
Adafruit_NeoPixel::Adafruit_NeoPixel() :
#ifdef NEO_KHZ400
  is800KHz(true),
#endif
  begun(false), numLEDs(0), numBytes(0), pin(-1), brightness(0), pixels(NULL),
  rOffset(1), gOffset(0), bOffset(2), wOffset(1), endTime(0)
{
}

Adafruit_NeoPixel::~Adafruit_NeoPixel() {
  if(pixels)   free(pixels);
  if(pin >= 0) pinMode(pin, INPUT);
}

void Adafruit_NeoPixel::begin(void) {
  if(pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  begun = true;
}

void Adafruit_NeoPixel::updateLength(uint16_t n) {
  if(pixels) free(pixels); // Free existing data (if any)

  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  numBytes = n * ((wOffset == rOffset) ? 3 : 4);
  if((pixels = (uint8_t *)malloc(numBytes))) {
    memset(pixels, 0, numBytes);
    numLEDs = n;
  } else {
    numLEDs = numBytes = 0;
  }
}

void Adafruit_NeoPixel::updateType(neoPixelType t) {
  bool oldThreeBytesPerPixel = (wOffset == rOffset); // false if RGBW

  wOffset = (t >> 6) & 0b11; // See notes in header file
  rOffset = (t >> 4) & 0b11; // regarding R/G/B/W offsets
  gOffset = (t >> 2) & 0b11;
  bOffset =  t       & 0b11;
#ifdef NEO_KHZ400
  is800KHz = (t < 256);      // 400 KHz flag is 1<<8
#endif

  // If bytes-per-pixel has changed (and pixel data was previously
  // allocated), re-allocate to new size.  Will clear any data.
  if(pixels) {
    bool newThreeBytesPerPixel = (wOffset == rOffset);
    if(newThreeBytesPerPixel != oldThreeBytesPerPixel) updateLength(numLEDs);
  }
}

void Adafruit_NeoPixel::show(void) {

  if(!pixels) return;

  // Data latch = 50+ microsecond pause in the output stream.  Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed.  This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while(!canShow());
  // endTime is a private member (rather than global var) so that mutliple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions.  It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT.  The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  __disable_irq();
  //noInterrupts(); // Need 100% focus on instruction timing

#if defined(__arm__)

// ARM MCUs -- Teensy 3.0, 3.1, LC, Arduino Due ---------------------------

#if defined(__MK20DX128__) || defined(__MK20DX256__) // Teensy 3.0 & 3.1
#define CYCLES_800_T0H  (F_CPU / 4000000)
#define CYCLES_800_T1H  (F_CPU / 1250000)
#define CYCLES_800      (F_CPU /  800000)
#define CYCLES_400_T0H  (F_CPU / 2000000)
#define CYCLES_400_T1H  (F_CPU /  833333)
#define CYCLES_400      (F_CPU /  400000)

  uint8_t          *p   = pixels,
                   *end = p + numBytes, pix, mask;
  volatile uint8_t *set = portSetRegister(pin),
                   *clr = portClearRegister(pin);
  uint32_t          cyc;

  ARM_DEMCR    |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    cyc = ARM_DWT_CYCCNT + CYCLES_800;
    while(p < end) {
      pix = *p++;
      for(mask = 0x80; mask; mask >>= 1) {
        while(ARM_DWT_CYCCNT - cyc < CYCLES_800);
        cyc  = ARM_DWT_CYCCNT;
        *set = 1;
        if(pix & mask) {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T1H);
        } else {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_800_T0H);
        }
        *clr = 1;
      }
    }
    while(ARM_DWT_CYCCNT - cyc < CYCLES_800);
#ifdef NEO_KHZ400
  } else { // 400 kHz bitstream
    cyc = ARM_DWT_CYCCNT + CYCLES_400;
    while(p < end) {
      pix = *p++;
      for(mask = 0x80; mask; mask >>= 1) {
        while(ARM_DWT_CYCCNT - cyc < CYCLES_400);
        cyc  = ARM_DWT_CYCCNT;
        *set = 1;
        if(pix & mask) {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_400_T1H);
        } else {
          while(ARM_DWT_CYCCNT - cyc < CYCLES_400_T0H);
        }
        *clr = 1;
      }
    }
    while(ARM_DWT_CYCCNT - cyc < CYCLES_400);
  }
#endif // NEO_KHZ400

#elif defined(__MKL26Z64__) // Teensy-LC

#if F_CPU == 48000000
  uint8_t          *p   = pixels,
		   pix, count, dly,
                   bitmask = digitalPinToBitMask(pin);
  volatile uint8_t *reg = portSetRegister(pin);
  uint32_t         num = numBytes;
  asm volatile(
	"L%=_begin:"				"\n\t"
	"ldrb	%[pix], [%[p], #0]"		"\n\t"
	"lsl	%[pix], #24"			"\n\t"
	"movs	%[count], #7"			"\n\t"
	"L%=_loop:"				"\n\t"
	"lsl	%[pix], #1"			"\n\t"
	"bcs	L%=_loop_one"			"\n\t"
	"L%=_loop_zero:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #4"			"\n\t"
	"L%=_loop_delay_T0H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T0H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #13"			"\n\t"
	"L%=_loop_delay_T0L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T0L"		"\n\t"
	"b	L%=_next"			"\n\t"
	"L%=_loop_one:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #13"			"\n\t"
	"L%=_loop_delay_T1H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T1H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #4"			"\n\t"
	"L%=_loop_delay_T1L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_loop_delay_T1L"		"\n\t"
	"nop"					"\n\t"
	"L%=_next:"				"\n\t"
	"sub	%[count], #1"			"\n\t"
	"bne	L%=_loop"			"\n\t"
	"lsl	%[pix], #1"			"\n\t"
	"bcs	L%=_last_one"			"\n\t"
	"L%=_last_zero:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #4"			"\n\t"
	"L%=_last_delay_T0H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T0H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #10"			"\n\t"
	"L%=_last_delay_T0L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T0L"		"\n\t"
	"b	L%=_repeat"			"\n\t"
	"L%=_last_one:"
	"strb	%[bitmask], [%[reg], #0]"	"\n\t"
	"movs	%[dly], #13"			"\n\t"
	"L%=_last_delay_T1H:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T1H"		"\n\t"
	"strb	%[bitmask], [%[reg], #4]"	"\n\t"
	"movs	%[dly], #1"			"\n\t"
	"L%=_last_delay_T1L:"			"\n\t"
	"sub	%[dly], #1"			"\n\t"
	"bne	L%=_last_delay_T1L"		"\n\t"
	"nop"					"\n\t"
	"L%=_repeat:"				"\n\t"
	"add	%[p], #1"			"\n\t"
	"sub	%[num], #1"			"\n\t"
	"bne	L%=_begin"			"\n\t"
	"L%=_done:"				"\n\t"
	: [p] "+r" (p),
	  [pix] "=&r" (pix),
	  [count] "=&r" (count),
	  [dly] "=&r" (dly),
	  [num] "+r" (num)
	: [bitmask] "r" (bitmask),
	  [reg] "r" (reg)
  );
#else
#error "Sorry, only 48 MHz is supported, please set Tools > CPU Speed to 48 MHz"
#endif // F_CPU == 48000000

#elif defined(__SAMD21G18A__) // Arduino Zero

  // Tried this with a timer/counter, couldn't quite get adequate
  // resolution.  So yay, you get a load of goofball NOPs...

  uint8_t  *ptr, *end, p, bitMask;
  uint32_t  pinMaskSet, pinMaskReset;

  pinMaskSet =  1ul << pin;
  pinMaskReset =  (1ul << pin) << 16U;
  ptr     =  pixels;
  end     =  ptr + numBytes;
  p       = *ptr++;
  bitMask =  0x80;

#define SET (GPIOA->BSRR = pinMaskSet)
#define RESET (GPIOA->BSRR = pinMaskReset)
//#define SET SET_BIT(GPIOA->ODR, pinMaskSet)
//#define RESET CLEAR_BIT(GPIOA->ODR, pinMaskSet)

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    for(;;) {
      SET;
      asm("nop; nop; nop; nop; nop; nop; nop; nop;");
      asm("nop; nop; nop; nop; nop; nop; nop; nop;");
      if(p & bitMask) {
          asm("nop; nop; nop; nop; nop; nop; nop; nop;"
              "nop; nop; nop; nop; nop; nop; nop; nop;"
              "nop; nop; nop; nop;");
          asm("nop; nop; nop; nop; nop; nop; nop; nop;"
              "nop; nop; nop; nop; nop; nop; nop; nop;"
              "nop; nop; nop; nop;");
        RESET;
      } else {
    	RESET;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
      }
      if(bitMask >>= 1) {
          asm("nop; nop; nop; nop; nop; nop; nop; nop; nop;");
          asm("nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    for(;;) {
      SET;
      asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
      if(p & bitMask) {
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop;");
        RESET;
      } else {
    	RESET;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop;");
      }
      asm("nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;"
          "nop; nop; nop; nop; nop; nop; nop; nop;");
      if(bitMask >>= 1) {
        asm("nop; nop; nop; nop; nop; nop; nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }
  }
#endif

#elif defined (ARDUINO_STM32_FEATHER) // FEATHER WICED (120MHz)

  // Tried this with a timer/counter, couldn't quite get adequate
  // resolution.  So yay, you get a load of goofball NOPs...

  uint8_t  *ptr, *end, p, bitMask;
  uint32_t  pinMask;

  pinMask =  BIT(PIN_MAP[pin].gpio_bit);
  ptr     =  pixels;
  end     =  ptr + numBytes;
  p       = *ptr++;
  bitMask =  0x80;

  volatile uint16_t *set = &(PIN_MAP[pin].gpio_device->regs->BSRRL);
  volatile uint16_t *clr = &(PIN_MAP[pin].gpio_device->regs->BSRRH);

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    for(;;) {
      if(p & bitMask) { // ONE
        // High 800ns
        *set = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop;");
        // Low 450ns
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop;");
      } else { // ZERO
        // High 400ns
        *set = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop;");
        // Low 850ns
        *clr = pinMask;
        asm("nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop; nop; nop; nop; nop;"
            "nop; nop; nop; nop;");
      }
      if(bitMask >>= 1) {
        // Move on to the next pixel
        asm("nop;");
      } else {
        if(ptr >= end) break;
        p       = *ptr++;
        bitMask = 0x80;
      }
    }
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    // ToDo!
  }
#endif

#else // Other ARM architecture -- Presumed Arduino Due

  #define SCALE      VARIANT_MCK / 2UL / 1000000UL
  #define INST       (2UL * F_CPU / VARIANT_MCK)
  #define TIME_800_0 ((int)(0.40 * SCALE + 0.5) - (5 * INST))
  #define TIME_800_1 ((int)(0.80 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_800 ((int)(1.25 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_0 ((int)(0.50 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_1 ((int)(1.20 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_400 ((int)(2.50 * SCALE + 0.5) - (5 * INST))

  int             pinMask, time0, time1, period, t;
  Pio            *port;
  volatile WoReg *portSet, *portClear, *timeValue, *timeReset;
  uint8_t        *p, *end, pix, mask;

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)TC3_IRQn);
  TC_Configure(TC1, 0,
    TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_Start(TC1, 0);

  pinMask   = g_APinDescription[pin].ulPin; // Don't 'optimize' these into
  port      = g_APinDescription[pin].pPort; // declarations above.  Want to
  portSet   = &(port->PIO_SODR);            // burn a few cycles after
  portClear = &(port->PIO_CODR);            // starting timer to minimize
  timeValue = &(TC1->TC_CHANNEL[0].TC_CV);  // the initial 'while'.
  timeReset = &(TC1->TC_CHANNEL[0].TC_CCR);
  p         =  pixels;
  end       =  p + numBytes;
  pix       = *p++;
  mask      = 0x80;

#ifdef NEO_KHZ400 // 800 KHz check needed only if 400 KHz support enabled
  if(is800KHz) {
#endif
    time0  = TIME_800_0;
    time1  = TIME_800_1;
    period = PERIOD_800;
#ifdef NEO_KHZ400
  } else { // 400 KHz bitstream
    time0  = TIME_400_0;
    time1  = TIME_400_1;
    period = PERIOD_400;
  }
#endif

  for(t = time0;; t = time0) {
    if(pix & mask) t = time1;
    while(*timeValue < period);
    *portSet   = pinMask;
    *timeReset = TC_CCR_CLKEN | TC_CCR_SWTRG;
    while(*timeValue < t);
    *portClear = pinMask;
    if(!(mask >>= 1)) {   // This 'inside-out' loop logic utilizes
      if(p >= end) break; // idle time to minimize inter-byte delays.
      pix = *p++;
      mask = 0x80;
    }
  }
  while(*timeValue < period); // Wait for last bit
  TC_Stop(TC1, 0);

#endif // end Due

// END ARM ----------------------------------------------------------------
#endif // __arm__

// END ARCHITECTURE SELECT ------------------------------------------------


  __enable_irq();
  //interrupts();
  endTime = micros(); // Save EOD time for latch on next call
}

// Set the output pin number
void Adafruit_NeoPixel::setPin(uint8_t p) {
  if(begun && (pin >= 0)) pinMode(pin, INPUT);
    pin = p;
    if(begun) {
      pinMode(p, OUTPUT);
      digitalWrite(p, LOW);
    }
}

// Set pixel color from separate R,G,B components:
void Adafruit_NeoPixel::setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b) {

  if(n < numLEDs) {
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    uint8_t *p;
    if(wOffset == rOffset) { // Is an RGB-type strip
      p = &pixels[n * 3];    // 3 bytes per pixel
    } else {                 // Is a WRGB-type strip
      p = &pixels[n * 4];    // 4 bytes per pixel
      p[wOffset] = 0;        // But only R,G,B passed -- set W to 0
    }
    p[rOffset] = r;          // R,G,B always stored
    p[gOffset] = g;
    p[bOffset] = b;
  }
}

void Adafruit_NeoPixel::setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {

  if(n < numLEDs) {
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
      w = (w * brightness) >> 8;
    }
    uint8_t *p;
    if(wOffset == rOffset) { // Is an RGB-type strip
      p = &pixels[n * 3];    // 3 bytes per pixel (ignore W)
    } else {                 // Is a WRGB-type strip
      p = &pixels[n * 4];    // 4 bytes per pixel
      p[wOffset] = w;        // Store W
    }
    p[rOffset] = r;          // Store R,G,B
    p[gOffset] = g;
    p[bOffset] = b;
  }
}

// Set pixel color from 'packed' 32-bit RGB color:
void Adafruit_NeoPixel::setPixelColor(uint16_t n, uint32_t c) {
  if(n < numLEDs) {
    uint8_t *p,
      r = (uint8_t)(c >> 16),
      g = (uint8_t)(c >>  8),
      b = (uint8_t)c;
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    if(wOffset == rOffset) {
      p = &pixels[n * 3];
    } else {
      p = &pixels[n * 4];
      uint8_t w = (uint8_t)(c >> 24);
      p[wOffset] = brightness ? ((w * brightness) >> 8) : w;
    }
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
  }
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t Adafruit_NeoPixel::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

// Convert separate R,G,B,W into packed 32-bit WRGB color.
// Packed format is always WRGB, regardless of LED strand color order.
uint32_t Adafruit_NeoPixel::Color(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  return ((uint32_t)w << 24) | ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

// Query color from previously-set pixel (returns packed 32-bit RGB value)
uint32_t Adafruit_NeoPixel::getPixelColor(uint16_t n) const {
  if(n >= numLEDs) return 0; // Out of bounds, return no color.

  uint8_t *p;

  if(wOffset == rOffset) { // Is RGB-type device
    p = &pixels[n * 3];
    if(brightness) {
      // Stored color was decimated by setBrightness().  Returned value
      // attempts to scale back to an approximation of the original 24-bit
      // value used when setting the pixel color, but there will always be
      // some error -- those bits are simply gone.  Issue is most
      // pronounced at low brightness levels.
      return (((uint32_t)(p[rOffset] << 8) / brightness) << 16) |
             (((uint32_t)(p[gOffset] << 8) / brightness) <<  8) |
             ( (uint32_t)(p[bOffset] << 8) / brightness       );
    } else {
      // No brightness adjustment has been made -- return 'raw' color
      return ((uint32_t)p[rOffset] << 16) |
             ((uint32_t)p[gOffset] <<  8) |
              (uint32_t)p[bOffset];
    }
  } else {                 // Is RGBW-type device
    p = &pixels[n * 4];
    if(brightness) { // Return scaled color
      return (((uint32_t)(p[wOffset] << 8) / brightness) << 24) |
             (((uint32_t)(p[rOffset] << 8) / brightness) << 16) |
             (((uint32_t)(p[gOffset] << 8) / brightness) <<  8) |
             ( (uint32_t)(p[bOffset] << 8) / brightness       );
    } else { // Return raw color
      return ((uint32_t)p[wOffset] << 24) |
             ((uint32_t)p[rOffset] << 16) |
             ((uint32_t)p[gOffset] <<  8) |
              (uint32_t)p[bOffset];
    }
  }
}

// Returns pointer to pixels[] array.  Pixel data is stored in device-
// native format and is not translated here.  Application will need to be
// aware of specific pixel data format and handle colors appropriately.
uint8_t *Adafruit_NeoPixel::getPixels(void) const {
  return pixels;
}

uint16_t Adafruit_NeoPixel::numPixels(void) const {
  return numLEDs;
}

// Adjust output brightness; 0=darkest (off), 255=brightest.  This does
// NOT immediately affect what's currently displayed on the LEDs.  The
// next call to show() will refresh the LEDs at this level.  However,
// this process is potentially "lossy," especially when increasing
// brightness.  The tight timing in the WS2811/WS2812 code means there
// aren't enough free cycles to perform this scaling on the fly as data
// is issued.  So we make a pass through the existing color data in RAM
// and scale it (subsequent graphics commands also work at this
// brightness level).  If there's a significant step up in brightness,
// the limited number of steps (quantization) in the old data will be
// quite visible in the re-scaled version.  For a non-destructive
// change, you'll need to re-render the full strip data.  C'est la vie.
void Adafruit_NeoPixel::setBrightness(uint8_t b) {
  // Stored brightness value is different than what's passed.
  // This simplifies the actual scaling math later, allowing a fast
  // 8x8-bit multiply and taking the MSB.  'brightness' is a uint8_t,
  // adding 1 here may (intentionally) roll over...so 0 = max brightness
  // (color values are interpreted literally; no scaling), 1 = min
  // brightness (off), 255 = just below max brightness.
  uint8_t newBrightness = b + 1;
  if(newBrightness != brightness) { // Compare against prior value
    // Brightness has changed -- re-scale existing data in RAM
    uint8_t  c,
            *ptr           = pixels,
             oldBrightness = brightness - 1; // De-wrap old brightness value
    uint16_t scale;
    if(oldBrightness == 0) scale = 0; // Avoid /0
    else if(b == 255) scale = 65535 / oldBrightness;
    else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
    for(uint16_t i=0; i<numBytes; i++) {
      c      = *ptr;
      *ptr++ = (c * scale) >> 8;
    }
    brightness = newBrightness;
  }
}

//Return the brightness value
uint8_t Adafruit_NeoPixel::getBrightness(void) const {
  return brightness - 1;
}

void Adafruit_NeoPixel::clear() {
  memset(pixels, 0, numBytes);
}













