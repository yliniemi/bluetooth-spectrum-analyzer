

 

/*
TODO
DONE 4096 (is it even possible?) IT IS POSSIBLE
DONE 128 bands might be possible too at 45 fps. It's actually possible at 115 fps.
- Change backgound dynamically
- Animated backgound
  - Perhaps GIFs

DONE Get samples based on the time the last loop took + 1 %
- Retain both channels so I can analyze them both separately if I wawnt to.
  I'll do the conversion to 16 bits on core 1 instead of core 0.
  This might cost me some microseconds but probably not much since they are just bitwise operations.
  - Print the channels mirrored and not mirrored and reversed.

- Hard code several different images. I can also flip them to get variety.
- Flip analyzer upside down. Maybe I can come up with an elegant solution to all these flips.
  Perhaps I can flip images in place or have four different funtions to flip the images dynamically while I do the conversion from 2D to snakewise panel.
- Slow the falling down
- Add peaks that fall down. Maybe they start slowly falling down right away or wait a while.

Suggestions from Facebook

Development proposal from a musician's point of view. Such a feature could be useful, that it would be possible to choose a frequency range from e.g. four different presets.
1 the whole range, 2 the bass, 3 the middle frequencies, 4 the upper frequencies, or even better if the range was self-definable.
In this way, it would be possible to observe a certain area centrally and with a more precise resolution.

That is a good and feasible proposal. At first, I was thinking about an arbitrary area, but it would have to count a lot.
If there are only four of those areas, you can hardcode them into the flash rom. Ram is saved and the microcontroller does not have to count for ten seconds.

Yes, the typical use of colors in these momentary peaks, which remain on for a while, seems to be in that direction.
Red if it goes over +-0dB, orange "critically close to 0dB", and something else otherwise.
But here the zero level seems to be at the top, i.e. inside the device, and then it would depend on the application and the implementation where the relative 0dB would be.
Then part of the dynamics of this device would be used to measure that extra headroom, right Antti Yliniemi?
  
I'm also doing the DC removal wrong. At the moment, I'm subtracting its average from the signal.
This can cause the first two beams to rise to haunt. This removal of DC must be planned sensibly so that it does not use too much memory.
https://ccrma.stanford.edu/~jos/fp/DC_Blocker_Software_Implementations.html

I can copy some funtionality from this product
https://www.youtube.com/watch?v=iZrWMv02tlA
*/

/*
    Bluetooth Speaker Spectrum Analyzer
    Copyright (C) 2022 Antti Yliniemi
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
I use these libraries:
https://github.com/hpwit/I2SClocklessLedDriver                // I use a heavily modified version of this library. Will be putting a pull request on his github repo after I've done more testing.
https://github.com/yliniemi/I2SClocklessLedDriver/tree/dev    // This is my current heavily modified version. Perhaps the changes end up in the hpwit's version some day
https://github.com/fakufaku/esp32-fft                         // This is the fastest FFT library I could find. Analyzing 4096 samples takes 5 milliseconds.
https://github.com/pschatzmann/ESP32-A2DP
*/

#include "BluetoothA2DPSink.h"
extern "C"
{
   #include "fft.h"
}

struct CRGB {
  union {
    struct {
            union {
                uint8_t r;
                uint8_t red;
            };
            union {
                uint8_t g;
                uint8_t green;
            };
            union {
                uint8_t b;
                uint8_t blue;
            };
        };
    uint8_t raw[3];
  };
};

struct FloatOffset
{
  float x;
  float y;
};

#include "constants.h"

 //#define STRESS_RAM

#define BISCUIT

enum MapMode
{
  RAINBOW,
  TEXTURES
};
MapMode mapMode = RAINBOW;

#define TILE_WIDTH 32
#define TILE_HEIGHT 32
#define ENABLE_LEDMAP
#define PANEL_WIDTH 16
#define PANEL_HEIGHT 16
#define NUM_PANELS_PER_ROW 7
#define NUM_PANELS_PER_COLUMN 2
#define SCREEN_WIDTH PANEL_WIDTH * NUM_PANELS_PER_ROW
#define SCREEN_HEIGHT PANEL_HEIGHT * NUM_PANELS_PER_COLUMN
#define ATX_POWER_ON 13
#define DEFAULT_SAMPLE_RATE 44100
#define HIGHEST_FREQUENCY 20000
#define START_SPECTRUM_AT_THIS_BIN 3
#define ADD_TO_DELTA 0.5

#ifdef BISCUIT
#define SIGNAL_STARTS_FROM_THE_BOTTOM
#define PANEL_WIDTH 16
#define PANEL_HEIGHT 24
#define NUM_PANELS_PER_ROW 8
#define NUM_PANELS_PER_COLUMN 1
#define SCREEN_WIDTH PANEL_WIDTH * NUM_PANELS_PER_ROW
#define SCREEN_HEIGHT PANEL_HEIGHT * NUM_PANELS_PER_COLUMN
#define ATX_POWER_ON 27
#endif

// TaskHandle_t task1, task2, task3;

// has to be between 512 - 2048
// more than 2048 and we run out of ram. less than 512 and we run out of time to draw it on the led panel
// 512 => 86 fps, 1024 => 43 fps, 2048 => 21 fps
#define SAMPLES 4096
// #define WAIT_UNTIL_DRAWING_DONE
#define SECONDS_BETWEEN_DEBUG 60
// #define USE_DOUBLE_BUFFERING
#define BANDS SCREEN_WIDTH
const float dynamicRange = 5.0;    // in bels, not decibels. bel is ten decibels. it's metric. bel is the base unit. long live the metric.
// #define PRINT_PLOT
#define DEBUG false
// #define DEBUG true
// #define PRINT_BANDS
// #define PRINT_CEILING
#define USE_SERIAL
// #define PRINT_OUTPUT
//#define PRINT_PEAKS
#define PRINT_FFT_TIME
#define PRINT_ALL_TIME
#define PRINT_FASTLED_TIME
#define PRINT_MAPLEDS_TIME
#define PRINT_RAM                   // This uses some resources that block the isr and take a long time
#ifdef STRESS_RAM
#define PRINT_RAM
#endif
// #define PRINT_INDEXES
// #define TEST_FULL_BUFFER
#define PRINT_SAMPLE_RATE
#define PRINT_LESSER_SAMPLES
#define PRINT_BUFFER_FULL
#define BRIGHTNESS_PIN 36 // GPIO36 pin connected to pot. (10k~100k)
#define BUTTON_PIN1    25 // GIOP26 pin connected to button
#define BUTTON_PIN2    26 // GIOP25 pin connected to button
#define BUTTON_PIN3    33 // GIOP33 pin connected to button
#define DEBOUNCE_TIME  200 // the debounce time in millisecond, increase this time if it still chatters
#define NUMBER_OF_STATES 8
// Kaiser windowing has the best reduction of side lobes and somewhat narrow main lobe. The other windows don't even hold a candle.

int maxCurrent = 15000;
// int maxBrightness = 32;

#define BUFFER_LENGTH 3072   // 2048 is also just fine. maybe 4096 has too much extra space
IRAM_ATTR int32_t bufferRing[BUFFER_LENGTH] = {};
volatile int writeIndex = 0;   // writeIndex will stay 4 behind readIndex. it can't go past
volatile int readIndex = 0;    // readIndex can be equal to writeIndex but cannot advance
volatile int bufferFull = 0;  
volatile int lesserSamples = 0;

IRAM_ATTR int32_t realRing[SAMPLES] = {};

int realRingIndex = 0;

const int numLeds = NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN * PANEL_WIDTH * PANEL_HEIGHT;

// uint8_t *leds = NULL;
CRGB *leds;

uint32_t loopMicros = 0;
uint32_t loopCycles = 0;

float deltaRatio = 0;
float rainbowChunk = 0.4;  
float rainbowOffset = 0;
//  rainbowDelta is a value that makes the rainbow change. it makes it faster or slower. 
// .... if (NEG(-))--> LEFT to RIGHT direction. 
// .... if (POS(+))<-- RIGHT to LEFT direction.
float rainbowDelta_1 = -0.00028; // RAINBOW CHANGING
float rainbowDelta_2 = -0.005;  // RAINBOW SWEEPING

static uint16_t ledMappingFunction(uint16_t hardwareLed);
#define __SOFTWARE_MAP
#include "I2SClocklessLedDriver.h"
I2SClocklessLedDriver driver;

char BTname[] = "SPECTRUM";
BluetoothA2DPSink a2dp_sink;

OffsetDisplay offd;
FloatOffset groundOffset;
FloatOffset skyOffset;

enum {bluetooth, microphone, artnet} programMode;

__attribute__((always_inline)) IRAM_ATTR static uint16_t ledMappingFunction(uint16_t hardwareLed)
{
  int x, y, moduloRow;
  // = MOD(FLOOR(A11 / 4), 12)
  x = (hardwareLed / PANEL_HEIGHT) % (PANEL_WIDTH * NUM_PANELS_PER_ROW);
  // = MOD(MOD(FLOOR(A11 / 4) + 1, 2) * MOD(A11, 4) + MOD(FLOOR(A11 / 4), 2) * MOD(95 - A11, 4) + FLOOR(A11 / 48) * 4, 96)
  // moduloRow = (hardwareLed / PANEL_HEIGHT) % 2;
  moduloRow = x % 2;
  // y = (hardwareLed / PANEL_HEIGHT) % 2;
  /*
  #ifdef BISCUIT
  if (y == 1)
  #else
  if (y == 0)
  #endif
  {
    y = (hardwareLed % PANEL_HEIGHT) + (hardwareLed / (PANEL_WIDTH * PANEL_HEIGHT * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT;
    // y = ((hardwareLed % PANEL_HEIGHT) + (hardwareLed / (PANEL_WIDTH * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT) % (PANEL_WIDTH * NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN);
  }
  else
  {
                                 y = (hardwareLed % PANEL_HEIGHT) + (hardwareLed / (PANEL_WIDTH * PANEL_HEIGHT * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT;
    y = ((PANEL_HEIGHT * 12345 - 1 - hardwareLed) % PANEL_HEIGHT) + (hardwareLed / (PANEL_WIDTH * PANEL_HEIGHT * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT;
    // y = (((PANEL_HEIGHT * 12345 - 1 - hardwareLed) % PANEL_HEIGHT) + (hardwareLed / (PANEL_WIDTH * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT) % (PANEL_WIDTH * NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN);
  }
  */
  #ifdef SIGNAL_STARTS_FROM_THE_BOTTOM
  // y = (hardwareLed % PANEL_HEIGHT) + (hardwareLed / (PANEL_WIDTH * PANEL_HEIGHT * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT;
  y = (moduloRow * hardwareLed + (1 - moduloRow) * (PANEL_HEIGHT * 12345 - 1 - hardwareLed)) % PANEL_HEIGHT + (hardwareLed / (PANEL_WIDTH * PANEL_HEIGHT * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT;
  #else
  y = ((1 - moduloRow) * hardwareLed + moduloRow * (PANEL_HEIGHT * 12345 - 1 - hardwareLed)) % PANEL_HEIGHT + (hardwareLed / (PANEL_WIDTH * PANEL_HEIGHT * NUM_PANELS_PER_ROW)) * PANEL_HEIGHT;
  #endif
  // y = (((hardwareLed / 16 + 1) % 2) * (hardwareLed % 16) + ((hardwareLed / 16) % 2) * ((16 * 123 - 1 - hardwareLed) % 16) + (hardwareLed / (16 * 7)) * 16 ) % (16 * 7 * 2);
  x = (x + driver._offsetDisplay.offsetx) % driver._offsetDisplay.panel_width;
  y = (y + driver._offsetDisplay.offsety) % driver._offsetDisplay.panel_height;
  // #ifdef BISCUIT
  // return x + y * driver._offsetDisplay.panel_width - 1;
  // #else
  return x + y * driver._offsetDisplay.panel_width;
  // #endif
}

float sqrtApprox(float number)
{
  union { float f; uint32_t u; } y = {number};
  y.u = 0x5F1FFFF9ul - (y.u >> 1);
  return number * 0.703952253f * y.f * (2.38924456f - number * y.f * y.f);
}
 
 //================= COLORS =====================
// BLACK
CRGB Peaks(float hue)
{
  CRGB color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  return color;
}
// BLUE
CRGB Blue(float hue)
{
  CRGB color;
  color.r = 0;
  color.g = 0;
  color.b = 255;
  return color;
}
// RAINBOW
CRGB redToBlue(float hue)
{
  CRGB color;
  color.r = std::max((float)255 * (1 - 2 * hue), (float)0) * 0.094 * 2.5;
  color.g = std::max(min(hue * 2 * 255, (1 - hue) * 2 * 255), (float)0) * 0.113 * 2.5;
  color.b = std::max((float)255 * (hue * 2 - 1), (float)0) * 0.080 * 2.5;
  return color;
}
// RAINBOW CHANGING
CRGB rainbow(float hue)
{
  CRGB color;
  if (hue < 0)
  {
    color.r = 64;
    color.g = 0;
    color.b = 0;
  }
  else if (hue <= 1.0 / 3)
  {
    color.r = 64 * (1 - hue * 3);
    color.g = 64 * (hue * 3);
    color.b = 0;
  }
  else if (hue <= 2.0 / 3)
  {
    color.r = 0;
    color.g = 64 * (1 - (hue - 1.0 / 3) * 3);
    color.b = 64 * ((hue - 1.0 / 3) * 3);
  }
  else if (hue <= 1.0)
  {
    color.r = 64 * ((hue - 2.0 / 3) * 3);;
    color.g = 0;
    color.b = 64 * (1 - (hue - 2.0 / 3) * 3);
  }
  else
  {
    color.r = 64;
    color.g = 0;
    color.b = 0;
  }
  return color;
}
//=============== END COLORS ====================

//=============== PATTERNS ======================
void mapLeds_rainbow(float* bands)
{
  for (int x = 0; x < BANDS; x++)
  {
    CRGB color = redToBlue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
	}
  }
}

void mapLeds_rainbow_upsidedown(float* bands)    
{
  for (int x = 0; x < BANDS; x++)
  {
    CRGB color = redToBlue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - y) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * 24 - y) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
    }
  }
}

void mapLeds_changing_rainbow(float* bands, float rainbowChunk, float rainbowOffset)
{
  for (int x = 0; x < BANDS; x++)
  {
	CRGB color = rainbow(fmodf((float)x / (float)BANDS * rainbowChunk * 0.038 + rainbowOffset, 1));  
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
	}
  }
}

void  mapLeds_sweeping_rainbow(float* bands, float rainbowChunk, float rainbowOffset)
{
  for (int x = 0; x < BANDS; x++)
  {
	CRGB color = rainbow(fmodf((float)x / (float)BANDS * rainbowChunk * 0.01 + rainbowOffset, 1));  
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
	}
  }
}

void mapLeds_Blue(float* bands)
{
  for (int x = 0; x < BANDS; x++)
  {
    CRGB color = Blue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
    }
  }
}

void mapLeds_Blue_upsidedown(float* bands) 
{
  for (int x = 0; x < BANDS; x++)
  {
    CRGB color = Blue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - y) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * 24 - y) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
    }
  }
}

void mapLeds_peaks(float* bands)
{
  for (int x = 0; x < BANDS; x++)
  {
    CRGB color = Peaks(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT%2 - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT%2 - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT%2 - 1 - y) + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
    }
  }
}

__attribute__((always_inline)) void mapLeds_textures(int x, FloatOffset groundOffset, FloatOffset skyOffset, float* bands)
{
    // CRGB color = redToBlue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      CRGB groundColor = ((CRGB*)yellow_sand)[((y + (int)groundOffset.y) % TILE_HEIGHT) * TILE_WIDTH + (x + (int)groundOffset.x) % TILE_WIDTH];
      groundColor.r = groundColor.r / 4;
      groundColor.g = groundColor.g / 32;
      groundColor.b = groundColor.b / 32;
      CRGB skyColor = ((CRGB*)dark_blue_water)[((y + (int)skyOffset.y) % TILE_HEIGHT) * TILE_WIDTH + (x + (int)skyOffset.x) % TILE_WIDTH];
      skyColor.r = skyColor.r / 64;
      skyColor.g = skyColor.g / 64;
      skyColor.b = skyColor.b / 64;
      /*
      uint8_t r = groundColor.r;
      groundColor.r = groundColor.g;
      groundColor.g = r;
      */
      if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0)
      {
        leds[x + SCREEN_WIDTH * y] = groundColor;
      }
      else if ((bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
      {
        float skyPortion = SCREEN_HEIGHT - 1 - y - bands[x] * SCREEN_HEIGHT;
        float groundPortion = 1 - skyPortion;
        leds[x + SCREEN_WIDTH * y].r = (float)groundColor.r * groundPortion + (float)skyColor.r * skyPortion;
        leds[x + SCREEN_WIDTH * y].g = (float)groundColor.g * groundPortion + (float)skyColor.g * skyPortion;
        leds[x + SCREEN_WIDTH * y].b = (float)groundColor.b * groundPortion + (float)skyColor.b * skyPortion;
      }
      else
      {
        leds[x + SCREEN_WIDTH * y] = skyColor;
      }
    }
}

void drawCharacter(const CRGB* character, float startingColumn, float offSet, float columnsPerFrame, int numberOfFrames, int frameWidth, int frameHeight)
{
   int frame = (int)((startingColumn + offSet) / columnsPerFrame) % numberOfFrames;
  // frame = 0;
  for (int x = (int)(startingColumn + 100) - 100; x < (int)startingColumn + frameWidth; x++)
  {
    if (x >= 0 && x < SCREEN_WIDTH)
    {
      for (int y = max(SCREEN_HEIGHT - frameHeight, 0); y < SCREEN_HEIGHT; y++)
      {
        int characterPixel = x - (int)(startingColumn + 100) + 100 + frame * frameWidth + (y - SCREEN_HEIGHT + frameHeight) * (numberOfFrames * frameWidth);
        // if (character[characterPixel].r != 0xFE && character[characterPixel].g != 0xFD && character[characterPixel].b != 0xFC)
        if ((*(uint32_t*)&(character[characterPixel]) & 0x00FFFFFF) != 0x00FCFDFE)    // I don't know why it's not 0xFEFDFC. It's because esp32 is little endian.
        {
          leds[x + y * SCREEN_WIDTH].r = character[characterPixel].r / 16;
          leds[x + y * SCREEN_WIDTH].g = character[characterPixel].g / 16;
          leds[x + y * SCREEN_WIDTH].b = character[characterPixel].b / 16;
        }
      }
    }
  }
  
}

void substractAverage(float* inputReal)
{
  float average = 0;
  for (int i = 0; i < SAMPLES; i++)
  {
    average += inputReal[i];
  }
  average /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++)
  {
    inputReal[i] = inputReal[i] - average;
  }
}

void doWindowing(float* inputReal)
{
  for (int i = 0; i < SAMPLES; i++)
  {
    inputReal[i] = inputReal[i] * windowingArray[i];
  }
}

void audio_data_callback(const uint8_t *data, uint32_t byteLen)
{
  int actualLen = byteLen / 4;
  if ((readIndex - writeIndex + BUFFER_LENGTH - 1) % BUFFER_LENGTH <= actualLen)
  {
    bufferFull++;
  }
  else
  {
    for (int i = 0; i < actualLen; i++)
    {
      // bufferRing[(writeIndex + i) % BUFFER_LENGTH] = *(int16_t*)&(data[i * 4]) / 2 + *(int16_t*)&(data[i * 4 + 2]) / 2;
      bufferRing[(writeIndex + i) % BUFFER_LENGTH] = ((int)*(int16_t*)&(data[i * 4]) + (int)*(int16_t*)&(data[i * 4 + 2]));
    }
  }
  writeIndex = (writeIndex + actualLen) % BUFFER_LENGTH;
  
  #ifdef PRINT_INDEXES
  Serial.print("Got ");
  Serial.print(actualLen);
  Serial.print(" data, writeIndex = ");
  Serial.println(writeIndex);
  #endif
}

void startAudio()
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = DEFAULT_SAMPLE_RATE, // updated automatically by A2DP
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),
//=============== OTHER COMMUNICATION FORMATS =============================	
//  .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
//  .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S_MSB),
//  .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S_PCM),
//  .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_MSB),
//  .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_PCM),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = true,
    .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
  };
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_i2s_port(I2S_NUM_1);
  a2dp_sink.set_stream_reader(audio_data_callback);
  
  #ifdef BISCUIT
  i2s_pin_config_t my_pin_config = {
      .bck_io_num = 14,
      .ws_io_num = 13,
      .data_out_num = 12,
      .data_in_num = I2S_PIN_NO_CHANGE };
  #else
  i2s_pin_config_t my_pin_config = {
      .bck_io_num = 4,
      .ws_io_num = 15,
      .data_out_num = 16,
      .data_in_num = I2S_PIN_NO_CHANGE };
  #endif
  a2dp_sink.set_pin_config(my_pin_config);
  // a2dp_sink.set_auto_reconnect(false);    // maybe this helps with my compatibility problem
  
  a2dp_sink.start(BTname);
  Serial.println(String("Started Bluetooth audio receiver with the name ") + BTname);
}

bool readBuffer(float* inputReal)
{
  int newSamples = min(min(SAMPLES, a2dp_sink.sample_rate() * (int)loopMicros / 990000), BUFFER_LENGTH);
  if ((writeIndex - readIndex + BUFFER_LENGTH) % BUFFER_LENGTH < newSamples / 2)   // i only ask for half as many samples to be present
  {
    // delay(1);
    return false;
  }
  if ((writeIndex - readIndex + BUFFER_LENGTH) % BUFFER_LENGTH < newSamples)
  {
    // delay(1);
    newSamples = (writeIndex - readIndex + BUFFER_LENGTH) % BUFFER_LENGTH;
    lesserSamples = newSamples;
  }
  
  {
    for (int i = 0; i < newSamples; i++)
    {
      realRing[realRingIndex] = (bufferRing[readIndex]);
      realRingIndex = (realRingIndex + 1) % SAMPLES;
      readIndex = (readIndex + 1) % BUFFER_LENGTH;
    }
    #ifdef PRINT_INDEXES
    Serial.println(String("readIndex = ") + readIndex + ", realRingIndex = " + realRingIndex);
    #endif
  }

  // DC-offset removal goes here. I don't yet know how. I need a few variables of ram.
  // But since I do overlapping fft, I need to do the DC removal again but the variables are continous.
  // I'm not going to save a history of them. Perhaps I need to calculate them again
  // by first calculating them to the point where I delete the circular buffer and save them for use the next loop.
  // only after that I'll read the buffer. I'll need to do the calculations like 7 times more than I should if I had an extra 16 kB.
  // I know the most probable amount of new samples I will use.
  // Only when I reach the head of the ring buffer I need to get variable amount of samples.
  // This only saves 15 % of calculating the DC-offset. Perhaps it's not worth it.

  // I could separate the channels at this point too. I can get them for free now since they are in IRAM.
  for (int i = 0; i < SAMPLES; i++)
  {
    inputReal[i] = (float)realRing[(realRingIndex + i) % SAMPLES];
  }
  return true;
}

void powerOfTwo(float* output)
{
  for (int index = 0; index < SAMPLES / 2; index++)
  {
    output[index] = output[index * 2] * output[index * 2] + output[index * 2 + 1] * output[index * 2 + 1];
  }
}

void zeroSmallBins(float* output)
{
  float biggest = 0;
  for (int i = 0; i < SAMPLES / 2; i++)
  {
    if (biggest < output[i]) biggest = output[i];
  }
  for (int i = 0; i < SAMPLES / 2; i++)
  {
    // we get rid of the unwanted frequency side lobes this way. kaiser 2 is quite eficient and we shouldn't see more than -60 dB on the side lobes
    // at the same time we make sure that we don't take logarithm out of zero. that would be -infinite
    output[i] = std::max(output[i] - biggest * 0.0000003, 0.0000000001);
    }
}

/*
void powTwoBands(float* output, float* bands)
{
  for (int i = 0; i < BANDS; i++)
  {
    bands[i] = 0;
    #if SAMPLES == 4096
    #if BANDS == 112
    for (int j = bins_4096_112[i]; j < bins_4096_112[i + 1]; j++)
    #endif
    #if BANDS == 64
    for (int j = bins_4096_64[i]; j < bins_4096_64[i + 1]; j++)
    #endif
    #if BANDS == 7
    for (int j = bins_4096_7[i]; j < bins_4096_7[i + 1]; j++)
    #endif
    #endif 
    #if SAMPLES == 2048
    for (int j = bins_2048_64[i]; j < bins_2048_64[i + 1]; j++)
    #endif
    
    {
      bands[i] += output[j];
    }
  }
}
*/

void logBands(float* bands, float* peakBands)
{
  for (int i = 0; i < BANDS; i++)
  {
    bands[i] = log10f(bands[i]);
    #ifdef PRINT_PEAKS
    if (peakBands[i] < bands[i]) peakBands[i] = bands[i];
    #endif
    #if BANDS == 7
    bands[i] = bands[i] - substract_7[i] + 6;
    #endif
    #if BANDS == 16
    bands[i] = bands[i] - substract_16[i] + 6;
    #endif
    #if BANDS == 32
    bands[i] = bands[i] - substract_32[i] + 6;
    #endif
    #if BANDS == 48
    bands[i] = bands[i] - substract_48[i] + 6;
    #endif
    #if BANDS == 64
    bands[i] = bands[i] - substract_64[i] + 6;
    #endif
    #if BANDS == 112
    bands[i] = bands[i] - substract_112[i] + 6;
    #endif
  }
}

void normalizeBands(float* bands)
{
  static float bandCeiling = 0.1;
  #ifdef PRINT_CEILING
  Serial.println(String("bandCeiling = ") + bandCeiling);
  #endif
  bandCeiling -= 0.00005;          // now it takes 200 seconds to come 10 dB down
  for (int i = 0; i < BANDS; i++)
  {
    if (bands[i] > bandCeiling) bandCeiling = bands[i];
  }
  for (int i = 0; i < BANDS; i++)
  {
    bands[i] = (bands[i] - bandCeiling + dynamicRange) / dynamicRange;
  }
}

  /* These will be combined to one function
  powTwoBands(output, bands);
  logBands(bands, peakBands);
  normalizeBands(bands);
  */
void powerBinsToBands(float* output, float* bands)
{
  int32_t startingPoint = START_SPECTRUM_AT_THIS_BIN;
  for (int32_t i = 0; i < BANDS; i++)
  {
    bands[i] = 0;
    int32_t delta = (startingPoint * deltaRatio) + ADD_TO_DELTA;
    if (delta <= 0) delta = 1;
    for (int32_t j = 0; j < delta; j++)
    {
      bands[i] += output[startingPoint + j];
    }
    startingPoint += delta;
    bands[i] = log10f(bands[i]);
    #ifdef PRINT_PEAKS
    if (peakBands[i] < bands[i]) peakBands[i] = bands[i];
    #endif
    // if (delta < 1) delta = 1;
    if (delta > 6) delta = 6;
    bands[i] -= substract_universal[delta - 1];
  }
  static float bandCeiling = -1000000;
  #ifdef PRINT_CEILING
  Serial.println(String("bandCeiling = ") + bandCeiling);
  #endif
  bandCeiling -= 0.00002;          // now it takes 300 seconds to come 10 dB down
  for (int i = 0; i < BANDS; i++)
  {
    if (bands[i] > bandCeiling) bandCeiling = bands[i];
  }
  for (int i = 0; i < BANDS; i++)
  {
    bands[i] = (bands[i] - bandCeiling + dynamicRange) / dynamicRange;
  }
}

float __attribute__ ((noinline)) checkDeltaRatio(float ratio, float startingPoint, int32_t rounds)
{
    for (int32_t i = 0; i < rounds; i++)
    {
        float delta = floorf((startingPoint * ratio) + ADD_TO_DELTA);
        if (delta <= 0) delta = 1;
        startingPoint += delta;
    }
    return startingPoint;
}

float __attribute__ ((noinline)) findDeltaRatio(int32_t maxDepth, float low, float high, float lowestValid, float startingPoint, int32_t endGoal, int32_t rounds)
{
    float middle = (low + high) * 0.5;
    float result = checkDeltaRatio(middle, startingPoint, rounds);
    // Serial.printf("%.17f : %f\n", middle, result);
    if (maxDepth <= 0) return lowestValid;    
    if (result > endGoal)
    {
        return findDeltaRatio(maxDepth - 1, low, middle, lowestValid, startingPoint, endGoal, rounds);
    }
    else
    {
        return findDeltaRatio(maxDepth - 1, middle, high, middle, startingPoint, endGoal, rounds);
    }
}

 bool charactersOn = false;
 
 //================##### BRIGHTNESS-(1) #####=======================
 uint8_t brightness = 255;
 float brightnessFloat = 0;
 //================#### BUTTON STUFF-(1) ####=======================
 // Variables will change:
 
 int lastSteadyState1 = LOW;       // the previous steady state from input pin
 int lastSteadyState2 = LOW;      // the previous steady state from input pin2
 int lastSteadyState3 = LOW;      // the previous steady state from input pin3
 int lastFlickerableState1 = LOW;  // the previous flickerable state from input pin
 int lastFlickerableState2 = LOW; // the previous flickerable state from input pin2
 int lastFlickerableState3 = LOW; // the previous flickerable state from the input pin3
 int currentState1;               // the current reading from input pin1
 int currentState2;               // the current reading from input pin2
 int currentState3;               // the current reading from input pin3
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
 unsigned long lastDebounceTime1 = 0;  // the last time the output pin was toggled
 unsigned long lastDebounceTime2 = 0; // the last time the output pin was toggled
 unsigned long lastDebounceTime3 = 0; // the last time the output pin was toggled
uint32_t buttonPushCounter = UINT32_MAX / 2;   // counter for the number of button presses

 //==============#### END BUTTON STUFF-(1) ####=========

 //======================<<<< DEMO_1 >>>>===============================
 // We are using hardware timer in ESP32. The timer calls onTimer function every (10) seconds.
 //  The timer can be stopped with BUTTON_PIN1 attached to PIN 26 (GIOP26)
  hw_timer_t * timer = NULL;   //H/W timer defining (Pointer to the Structure)
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//Defining Inerrupt function with IRAM_ATTR for faster access
void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
 // charactersOn ^= true;
  buttonPushCounter++; // function that uses timer
  portEXIT_CRITICAL_ISR(&timerMux); 
}
 //=====================>>>> END DEMO_1 <<<<============================
 
void setup()
{
  delay( 1000 ); // power-up safety delay
 #ifdef USE_SERIAL
  Serial.begin(115200);
  #endif
  //==============#### BUTTON STUFF-(2) ####============== 
  // initialize the pushbutton pin as a pull-up input
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
  pinMode(BUTTON_PIN3, INPUT_PULLUP);
  //===========#### END BUTTON STUFF-(2) ####=============
  pinMode(BRIGHTNESS_PIN, INPUT);

  
  programMode = bluetooth;
  Serial.printf("programMode = %d\n\r", programMode);
  Serial.println("Just booted up");
  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  delay(100);
  int32_t lastBin = min(SAMPLES / 2 - 1, HIGHEST_FREQUENCY * SAMPLES / DEFAULT_SAMPLE_RATE);
  deltaRatio = findDeltaRatio(25, 0, 0.2, 0, START_SPECTRUM_AT_THIS_BIN, lastBin, BANDS);
  Serial.printf("deltaRatio = %f\n", deltaRatio);
  Serial.printf("numLeds = %d\n", numLeds);
  Serial.printf("SCREEN_WIDTH = %d\n", SCREEN_WIDTH);
  Serial.printf("SCREEN_HEIGHT = %d\n", SCREEN_HEIGHT);
  
  #ifdef BISCUIT
  int pins[] = {15, 4, 16, 17, 5, 18, 19, 21};              // esp32 dev kit v1
  #else
  int pins[] = {32, 33, 25, 26, 27, 14, 12, 23, 22, 21, 19, 18, 5, 17};              // esp32 dev kit v1
  #endif
  #ifndef WAIT_UNTIL_DRAWING_DONE
  driver.__displayMode = NO_WAIT;
  #endif
  
  #ifdef STRESS_RAM
  leds = (CRGB*)calloc(numLeds * 2, sizeof(CRGB));      // testing if I have enough ram for a 112 x 64 screen
  driver.initled((uint8_t*)leds, pins, NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN, PANEL_WIDTH * PANEL_HEIGHT * 2, ORDER_GRB);    // simulating screen twice as big and half as fast
  #else
  leds = (CRGB*)calloc(numLeds, sizeof(CRGB));
  driver.initled((uint8_t*)leds, pins, NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN, PANEL_WIDTH * PANEL_HEIGHT, ORDER_GRB);
  #endif
  // These two lines have to be after driver.initled()
  driver._offsetDisplay.panel_width=SCREEN_WIDTH;
  driver._offsetDisplay.panel_height=SCREEN_HEIGHT;
  
  offd = driver.getDefaultOffset();
  offd.panel_width=SCREEN_WIDTH;
  offd.panel_height=SCREEN_HEIGHT;

  Serial.printf("driver._offsetDisplay.panel_width = %d\n", driver._offsetDisplay.panel_width);
  Serial.printf("driver._offsetDisplay.panel_height = %d\n", driver._offsetDisplay.panel_height);
  
  groundOffset.x = 0;
  groundOffset.y = 0;
  skyOffset.x = 0;
  skyOffset.y = 0;
  
  // driver.setMapLed(mapLedsOnPanel);
  
  startAudio();
  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  delay(100);
  
  pinMode(ATX_POWER_ON, OUTPUT);
  digitalWrite(ATX_POWER_ON, LOW);
  // gpio_set_direction((gpio_num_t)13, GPIO_MODE_OUTPUT);
  // gpio_set_level((gpio_num_t)13, 0);
  
  Serial.printf("ledMappingFunction(0) = %d\n\r", ledMappingFunction(0));
  Serial.printf("ledMappingFunction(1790) = %d\n\r", ledMappingFunction(1790));
 //====================<<<< DEMO_2 >>>>==============================
  // Initilise the timer.
  // Parameter 1 is the timer we want to use. Valid: 0, 1, 2, 3 (total 4 timers)
  // Parameter 2 is the prescaler. The ESP32 default clock is at 80MhZ. The value "80" will
  // divide the clock by 80, giving us 1,000,000 ticks per second.
  // Parameter 3 is true means this counter will count up, instead of down (false).
  timer = timerBegin(0, 80, true);  
  
  // Attach the timer to the interrupt service routine named "onTimer".
  // The 3rd parameter is set to "true" to indicate that we want to use the "edge" type (instead of "flat").
  timerAttachInterrupt(timer, &onTimer, true);
  
  // This is where we indicate the frequency of the interrupts.
  // The value "1000000" (because of the prescaler we set in timerBegin) will produce
  // one interrupt every second.
  // The 3rd parameter is true so that the counter reloads when it fires an interrupt, and so we
  // can get periodic interrupts (instead of a single interrupt). 
  timerAlarmWrite(timer, 10000000, true);  // 10000000 is 10sec
  timerAlarmEnable(timer);   // Enable Timer with interrupt.
 //==================>>>> END DEMO_2 <<<<============================
}

void loopBluetooth()
{
  
  static float* inputReal = (float*)calloc(SAMPLES, sizeof(float));
  static float* output = (float*)calloc(SAMPLES, sizeof(float));
  static fft_config_t* fftReal = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, inputReal, output);
  static float* bands = (float*)calloc(BANDS, sizeof(float));
  #ifdef PRINT_PEAKS
  static float* peakBands = (float*)calloc(BANDS, sizeof(float));
  #else
  static float* peakBands;
  #endif
  
  //bool state;
  //bool charactersOn = false;
 
  static unsigned int beebBoob = 178;
  static uint32_t previousMicros = 0;
  static uint32_t previousCycles = 0;
  uint32_t newMicros = micros();
  uint32_t newCycles = xthal_get_ccount();
  static uint32_t previousDebugMillis = 0;
  loopMicros = newMicros - previousMicros;
  loopCycles = newCycles - previousCycles;
  previousMicros = newMicros;
  previousCycles = newCycles;
  if (previousDebugMillis == 0)
  {
    previousDebugMillis = 1;
    Serial.print("ESP.getFreeHeap() = ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
    Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  }
  
  while (true)
  {
    if (readBuffer(inputReal)) break;
    else
    {
      vTaskDelay(1);
      previousMicros = micros();
      previousCycles = xthal_get_ccount();
    }
  }


  #ifdef PRINT_FFT_TIME
  uint32_t fftMicros = micros();
  uint32_t fftCycles = xthal_get_ccount();
  #endif
  
  // substractAverage(inputReal);
  doWindowing(inputReal);
  fft_execute(fftReal);
  powerOfTwo(output);   // if we end up doing log() of the output anyways there is no need to do costly sqrt() because it's the same as dividing log() by 2
  // sqrtBins();     // we don't actually need this since we are dealing with the power of the signal and not the amplitude
  zeroSmallBins(output);  // we do this because there are plenty of of reflections in the surrounding bins

  /* These will be combined to one function
  powTwoBands(output, bands);
  logBands(bands, peakBands);
  normalizeBands(bands);
  */
  powerBinsToBands(output, bands);
  
  groundOffset.x += 0.03;
  groundOffset.y += 0.5;
  skyOffset.x += 0.23;
  skyOffset.y += 0.05;
  if (groundOffset.x < 0) groundOffset.x += TILE_WIDTH;
  if (groundOffset.y < 0) groundOffset.y += TILE_HEIGHT;
  if (skyOffset.x < 0) skyOffset.x += TILE_WIDTH;
  if (skyOffset.y < 0) skyOffset.y += TILE_HEIGHT;
  
  #ifdef PRINT_FFT_TIME
  fftMicros = micros() - fftMicros;
  fftCycles = xthal_get_ccount() - fftCycles;
  #endif
  
  #ifndef WAIT_UNTIL_DRAWING_DONE
  while (driver.isDisplaying == true) vTaskDelay(1);
  #endif
  
  #ifdef PRINT_RAM
  // I had to move these commands here because they bothered I2SCloclessLedDriver
  // They do something that blocks the interrupts or something
  // I was getting pretty garbled results just because of this
  // I had to wait until I2SCloclessLedDriver was done drawing the leds
  static int heap_caps_get_largest_free_block_8bit_min = 1000000000;
  static int heap_caps_get_largest_free_block_32bit_min = 1000000000;
  int newValue = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  if (newValue < heap_caps_get_largest_free_block_8bit_min) heap_caps_get_largest_free_block_8bit_min = newValue;
  newValue = heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
  if (newValue < heap_caps_get_largest_free_block_32bit_min) heap_caps_get_largest_free_block_32bit_min = newValue;
  #endif
  
  uint32_t mapLedsMicros = micros();
  uint32_t mapLedsCycles = xthal_get_ccount();
  
  // mapLeds();
  // mapLeds_mario();
  // driver.ledToDisplay will be needed next

 //============##### BRIGHTNESS-(2) #####============================
  brightnessFloat = 0.99 * brightnessFloat + 0.01 * analogRead(BRIGHTNESS_PIN) / 4095 * 255;
  brightness = brightnessFloat;
  driver.setBrightness(brightness);
 //============#### BUTTON STUFF-(3) ####============================  
  currentState1 = digitalRead(BUTTON_PIN1);
  currentState2 = digitalRead(BUTTON_PIN2);
 //================ BUTTON (1) ======================================
  if (currentState1 != lastFlickerableState1)
  {
    lastDebounceTime1 = millis();
    lastFlickerableState1 = currentState1;
  }
  if ((millis() - lastDebounceTime1) > DEBOUNCE_TIME)
  {
    if (lastSteadyState1 == HIGH && currentState1 == LOW)
    {
 //=============== (DEMO STOP) ======================================	
	  // If timer is still running
      if (timer) {
      // Stop and free timer
         timerEnd(timer);
         timer = NULL;
	}
 //================================================================== 	
      buttonPushCounter++; // PATTERNS changing up
    } 
    lastSteadyState1 = currentState1;
  }
 //================ BUTTON (2) ======================================
  if (currentState2 != lastFlickerableState2)
  {
    lastDebounceTime2 = millis();
    lastFlickerableState2 = currentState2;
  }
  if ((millis() - lastDebounceTime2) > DEBOUNCE_TIME)
  {
    if (lastSteadyState2 == HIGH && currentState2 == LOW)
    {
 //=============== (DEMO STOP) ======================================	
	  // If timer is still running
      if (timer) {
      // Stop and free timer
         timerEnd(timer);
         timer = NULL;
	}
 //================================================================== 
      buttonPushCounter--; // PATTERNS changing down
    }
    lastSteadyState2 = currentState2;
  }
 //==========#### END BUTTON STUFF-(3) ####==========================   

 //=========\\\\\ PATTERNS \\\\\=====================================
  switch (buttonPushCounter % NUMBER_OF_STATES)
  {
    case 0:  // WHITE PEAKS ONLY
      mapLeds_peaks(bands);  
      //  charactersOn ^= true;
      break;   
	
	case 1:  // RAINBOW NORMAL
      mapLeds_rainbow(bands);
      break;
      //	charactersOn ^= true;
 
    case 2:  // RAINBOW CHANGING
      rainbowOffset = fmodf(rainbowOffset + rainbowDelta_1, 1) + 1;
      mapLeds_changing_rainbow(bands, rainbowChunk, rainbowOffset);
      //  charactersOn ^= true;
      break;
	  
    case 3:  // RAINBOW SWEEPING
      rainbowOffset = fmodf(rainbowOffset + rainbowDelta_2, 1) + 1;
      mapLeds_sweeping_rainbow(bands, rainbowChunk, rainbowOffset);
      //  charactersOn ^= true;
      break;
	
    case 4:  // TEXTURES
      for (int x = 0; x < BANDS; x++)
      {
        mapLeds_textures(x, groundOffset, skyOffset, bands);
      }
      //	charactersOn ^= true;  
      break;
      
    case 5:  // BLUE
      mapLeds_Blue(bands);
      //	charactersOn ^= true;
      break;
      
	case 6:  // BLUE UPSIDEDOWN
	  mapLeds_Blue_upsidedown(bands);
      //	charactersOn ^= true;
      break; 
	  
    case 7:  // RAINBOW UPSIDEDOWN
	  mapLeds_rainbow_upsidedown(bands);	
      //	charactersOn ^= true;
      break; 
  }  
 //=========///// END PATTERNS  /////================================
  if (charactersOn == true)  
  {
    // drawCharacter(CRGB* character, float startingColumn, float offset, int columnsPerFrame, int numberOfFrames, int frameWidth, int frameHeight)
    static uint32_t characterNumber = 0;
    static int frameWidth = 32;
    static int frameHeight = 32;
    static int numberOfFrames = 8;
    static const CRGB* currentCharacter = (const CRGB*)tuxes_walking;
    static float columnsPerFrame = 1.5;
    static float offset = 100.4 + ((double)esp_random() / (double)4194304);
    static float startingColumn = 0 - frameWidth;
    // [frameHeight * 2 * (characterNumber % 11) * 3 * frameWidth * numberOfFrames]
    drawCharacter(currentCharacter, startingColumn, offset, columnsPerFrame, numberOfFrames, frameHeight, frameWidth);
    startingColumn += 0.2;
    if (startingColumn > SCREEN_WIDTH + frameWidth + 4)
    {
      characterNumber = (characterNumber + 1) % 24;
      {
        currentCharacter = (const CRGB*)tuxes_walking + characterNumber * frameHeight * frameWidth * numberOfFrames;
        frameWidth = 32;
        frameHeight = 32;
        numberOfFrames = 8;
      }
      offset = 100.4 + (float)esp_random() / 1000000.4;
      //offset = 100.4 + ((double)esp_random() / (double)4194304);
      if ((characterNumber % 2) == 0)
      {
        columnsPerFrame = 1.5;
      }
      else
      {
        columnsPerFrame = 2;
      }
      if (characterNumber == 22)
      {
        currentCharacter = (const CRGB*)guido_walking;
        frameWidth = 32;
        frameHeight = 32;
        numberOfFrames = 8;
        columnsPerFrame = 2; 
      }
      if (characterNumber == 23)
      {
        currentCharacter = (const CRGB*)squirrel_walking;
        frameWidth = 20;
        frameHeight = 20;
        numberOfFrames = 6;
        columnsPerFrame = 2;
      }
	   // startingColumn = 0 - frameWidth - 4 - (esp_random() / 2097152);
       randomSeed(millis());
       startingColumn = 0 - frameWidth - 4 - random(100, 1000);	  // time passed between characters showing
      
      // Serial.printf("Color of the first pixel is %u\n\r", (*(uint32_t*)&(currentCharacter[0])));
      // it should be 16711164;
      // it's actually 4277992958 because the cpu is little endian
      // that means that the last byte of the number is stored in the beginning
      // all the bytes in a number are in reverse order
      // back in the day when the bit width of the memory lane was less than the bit width of the cpu
      // it allowed the cpu to start the calculations one clock cycle before it got the whole number
    }
  }
  mapLedsMicros = micros() - mapLedsMicros;
  mapLedsCycles = xthal_get_ccount() - mapLedsCycles;
  
  uint32_t showPixelsMicros = micros();
  uint32_t showPixelsCycles = xthal_get_ccount();
  
  // driver.showPixels((uint8_t *)leds);
  // driver.showPixels(offd);
  // leds[8 * SCREEN_WIDTH + 51].r = 255;
  // leds[8 * SCREEN_WIDTH + 51].g = 255;
  // leds[8 * SCREEN_WIDTH + 51].b = 255;
  driver.showPixels(NO_WAIT);

  showPixelsMicros = micros() - showPixelsMicros;
  showPixelsCycles = xthal_get_ccount() - showPixelsCycles;
  
  #ifdef PRINT_PLOT
  plot();
  #endif
  #ifdef PRINT_BANDS
  printBands();
  #endif
  
  if (millis() - previousDebugMillis > SECONDS_BETWEEN_DEBUG * 1000)
  {
    previousDebugMillis = millis();
    Serial.printf("driver._offsetDisplay.panel_width = %d\n", driver._offsetDisplay.panel_width);
    Serial.printf("driver._offsetDisplay.panel_height = %d\n", driver._offsetDisplay.panel_height);

    #ifdef PRINT_FFT_TIME
    Serial.print("FFT took ");
    Serial.print(fftMicros);
    Serial.print(" ");
    Serial.print((fftCycles) / 240);
    Serial.println(" μs");
    #endif
    #ifdef PRINT_RAM
    Serial.print("ESP.getFreeHeap() = ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
    Serial.println(heap_caps_get_largest_free_block_8bit_min);
    Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
    Serial.println(heap_caps_get_largest_free_block_32bit_min);
    heap_caps_get_largest_free_block_8bit_min = 1000000000;
    heap_caps_get_largest_free_block_32bit_min = 1000000000;
    #endif
    #ifdef PRINT_FASTLED_TIME
    Serial.print("driver.showPixels() took ");
    Serial.print(showPixelsMicros);
    Serial.print(" ");
    Serial.print((showPixelsCycles) / 240);
    Serial.println(" μs ");
    #endif
    #ifdef PRINT_MAPLEDS_TIME
    Serial.print("MapLeds took ");
    Serial.print(mapLedsMicros);
    Serial.print(" ");
    Serial.print((mapLedsCycles) / 240);
    Serial.println(" μs ");
    #endif
    #ifdef PRINT_SAMPLE_RATE
    Serial.print("a2dp_sink.sample_rate() = ");
    Serial.println(a2dp_sink.sample_rate());
    #endif
    #ifdef PRINT_ALL_TIME
    Serial.print("everything took ");
    Serial.print(loopMicros);
    Serial.print(" ");
    Serial.print((loopCycles) / 240);
    Serial.println(" μs");
    #endif
    #ifdef PRINT_PEAKS
    Serial.print("Peak bands: ");
    for (int i = 0; i < BANDS; i++)
    {
      Serial.printf("%f, ", peakBands[i] - 6.0);
    }
    Serial.println();
    #endif
    #ifdef PRINT_LESSER_SAMPLES
    if (lesserSamples > 0)
    {
      Serial.print("lesserSamples = ");
      Serial.println(lesserSamples);
      lesserSamples = 0;
    }
    #endif
    #ifdef PRINT_BUFFER_FULL
    if (bufferFull > 0)
    {
      Serial.print("BUFFER FULL. DISCARDING DATA. BUFFER FULL. DISCARDING DATA. BUFFER FULL. bufferFull = ");
      Serial.println(bufferFull);
      bufferFull = 0;
    }
    #endif
  }
  #ifdef TEST_FULL_BUFFER
  delay(150);
  #endif
}

void loopMicrophone()
{
  
}

void loopArtnet()
{
  
}

void loop()
{
   loopBluetooth();  // This is the Loop where the magic happens.
   
  //==========####BUTTON STUFF-(4)####=============================
  //============= BUTTON (3) ======================================
  // read the state of the switch/button:
  currentState3 = digitalRead(BUTTON_PIN3);
  // If the switch/button changed, due to noise or pressing:
  if (currentState3 != lastFlickerableState3)
  {
    // reset the debouncing timer
    lastDebounceTime3 = millis();
    // save the the last flickerable state
    lastFlickerableState3 = currentState3;
  }
  if ((millis() - lastDebounceTime3) > DEBOUNCE_TIME)
  {   
    if(lastSteadyState3 == HIGH && currentState3 == LOW)
      charactersOn ^= true;  //"The button is pressed"
    else if(lastSteadyState3 == LOW && currentState3 == HIGH)
      charactersOn ^= false; //"The button is released"
    // save the the last steady state
    lastSteadyState3 = currentState3;
  }
  //========#### END BUTTON STUFF-(4)####==========================  
}
