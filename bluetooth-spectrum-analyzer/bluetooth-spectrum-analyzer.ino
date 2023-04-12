

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

Kehitysehdotus muusikon näkökulmasta. Semmoinen ominaisuus voisi olla hyödyllinen, että olisi mahdollista valita taajuusalue esim. neljästä eri presetistä.
1 koko alue, 2 bassot, 3 keskitaajuudet, 4 ylätaajuudet, tai vielä parempi, jos alue olisi itse määriteltävissä.
Näin pystyisi tarkkailemaan tiettyä aluetta keskitetysti ja tarkemmalla resoluutiolla.

Tuo on hyvä ja toteutettavissa oleva ehdotus. Aluksi mietin mielivaltaista aluetta, mutta sinne pitäis laskea tosi paljon.
Jos niitä alueita on vain neljä, niin ne voi hardkoodata flash romiin. Ram säästyy ja mikrokontrollerin ei tartte laskea kymmenen sekuntia.

Joo, tuon suuntainen se tyypillinen värien käyttö näissä hetkellissä huipussa mitkä jää vähäksi aikaa päälle taitaa olla.
Punainen jos menee +-0dB yli, oranssi "kriittisen lähellä 0dB", ja jotain muuta sitten muuten.
Mut tässähän nollataso taitaa olla tuolla ylälaidassa, siis laitteen sisäisenä, ja riippuisi sitten sovelluskohteesta ja toteutuksesta mihin kohtaa se suhteellinen 0dB asettuisi.
Silloin tän laitteen dynamiikasta osa käytettäisiin tuon ylimääräisen headroomin mittaamiseen, vai kuinka Antti Yliniemi?
  
Teen myös DC:n poistamisen väärin. Tällä hetkellä miinustan signaalista sen keskiarvon.
Siitä voi aiheutua että kaksi ekaa palkkia nousee kummittelemaan. Tämä DC:n poisto pitää suunnitella järkevästi ettei se käytä liikaa muistia.
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
extern "C" {
   #include "fft.h"
}

/*
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
*/
struct FloatOffset
{
  float x;
  float y;
};


#define TILE_WIDTH 32
#define TILE_HEIGHT 32
#define ENABLE_LEDMAP
#define PANEL_WIDTH 16
#define PANEL_HEIGHT 16
#define NUM_PANELS_PER_ROW 8
#define NUM_PANELS_PER_COLUMN 4
#define SCREEN_WIDTH PANEL_WIDTH * NUM_PANELS_PER_ROW
#define SCREEN_HEIGHT PANEL_HEIGHT * NUM_PANELS_PER_COLUMN

// TaskHandle_t task1, task2, task3;

// has to be between 512 - 2048
// more than 2048 and we run out of ram. less than 512 and we run out of time to draw it on the led panel
// 512 => 86 fps, 1024 => 43 fps, 2048 => 21 fps
#define SAMPLES 2048
#define WAIT_UNTIL_DRAWING_DONE
#define SECONDS_BETWEEN_DEBUG 600

// #define USE_DOUBLE_BUFFERING
#define BANDS 128
const float dynamicRange = 5.0;    // in bels, not decibels. bel is ten decibels. it's metric. bel is the base unit. long live the metric.
// #define PRINT_PLOT
#define DEBUG false
// #define DEBUG true
// #define PRINT_BANDS
// #define PRINT_CEILING
#define USE_SERIAL
// #define PRINT_OUTPUT
// #define PRINT_PEAKS
#define PRINT_FFT_TIME
#define PRINT_ALL_TIME
#define PRINT_FASTLED_TIME
#define PRINT_MAPLEDS_TIME
// #define PRINT_RAM                   // This uses some resources that block the isr and take a long time
// #define PRINT_INDEXES
#define TEST_FULL_BUFFER
#define PRINT_SAMPLE_RATE
#define PRINT_LESSER_SAMPLES
#define PRINT_BUFFER_FULL

// Kaiser windowing has the best reduction of side lobes and somewhat narrow main lobe. The other windows don't even hold a candle.

int maxCurrent = 15000;
// int maxBrightness = 32;

#define BUFFER_LENGTH 1700   // 2048 is also just fine. maybe 4096 has too much extra space
IRAM_ATTR int32_t bufferRing[BUFFER_LENGTH] = {};
volatile int writeIndex = 0;   // writeIndex will stay 4 behind readIndex. it can't go past
volatile int readIndex = 0;    // readIndex can be equal to writeIndex but cannot advance
volatile int bufferFull = 0;  
volatile int lesserSamples = 0;

int32_t realRing[SAMPLES] = {}; //IRAM_ATTR 

int realRingIndex = 0;

const int numLeds = NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN * PANEL_WIDTH * PANEL_HEIGHT;



//#define ESP_VIRTUAL_DRIVER_82_2 1
#define NBIS2SERIALPINS 4 
#define NUM_LEDS_PER_STRIP  256
#define NUM_STRIPS (NBIS2SERIALPINS * 8)
#define USE_FASTLED
#define COLOR_GRB
#define NUM_LEDS (NUM_LEDS_PER_STRIP *NUM_STRIPS)
#define __HARDWARE_MAP
#define BR 40
#define NBPANNEL 48

#include "pixelslib.h"
#include "I2SClocklessVirtualLedDriver.h"
#include "artnetESP32V2.h"

#include "constants.h"
//TwoWire d(0);
#define LATCH_PIN 27 //13
#define CLOCK_PIN 26 //27
int Pins[16]={14,12,13,25,33,32}; //12,14
 I2SClocklessVirtualLedDriver driver;
// uint8_t *leds = NULL;
CRGB *leds;
//CRGB leds[NUM_LEDS];

uint32_t loopMicros;
uint32_t loopCycles;
     static float bands[BANDS] ;
     static float *inputReal;//[SAMPLES];
      static  float *output;//[SAMPLES] ;
      static fft_config_t* fftReal;
       static float peakBands[BANDS];

//#include "I2SClocklessLedDriver.h"
//I2SClocklessLedDriver driver;

char BTname[] = "triumvirate";
BluetoothA2DPSink a2dp_sink;

OffsetDisplay offd;
FloatOffset groundOffset;
FloatOffset skyOffset;

enum {bluetooth, microphone, artnet} programMode;

float sqrtApprox(float number)
{
  union { float f; uint32_t u; } y = {number};
  y.u = 0x5F1FFFF9ul - (y.u >> 1);
  return number * 0.703952253f * y.f * (2.38924456f - number * y.f * y.f);
}

CRGB redToBlue(float hue)
{
  CRGB color;
  color.r = std::max((float)255 * (1 - 2 * hue), (float)0) * 0.094 * 2.5;
  color.g = std::max(min(hue * 2 * 255, (1 - hue) * 2 * 255), (float)0) * 0.113 * 2.5;
  color.b = std::max((float)255 * (hue * 2 - 1), (float)0) * 0.080 * 2.5;
  return color;
}

/*
void mapLeds()
{
  for (int i = 0; i < BANDS; i++)
  {
    for (int j = 0; j < 16; j++)
    {
      if ((bands[i] * 16 - j) >= 0) leds[ledMap[i * 16 + j]] = templateLeds[i * 16 + j];
      else if ((bands[i] * 16 - j) > -1)
      {
        leds[ledMap[i * 16 + j]].r = ((float)templateLeds[i * 16 + j].r + 1.0) * (bands[i] * 16.0 - j + 1.0);   // I add one to the color so that if it's small it doesn't disappear first. now if the color is one, it will disappear in the middle.
        leds[ledMap[i * 16 + j]].g = ((float)templateLeds[i * 16 + j].g + 1.0) * (bands[i] * 16.0 - j + 1.0);
        leds[ledMap[i * 16 + j]].b = ((float)templateLeds[i * 16 + j].b + 1.0) * (bands[i] * 16.0 - j + 1.0);
      }
      else leds[ledMap[i * 16 + j]] = {0, 0, 0};
    }
  }
}
*/

/*
void mapLeds()
{
  for (int x = 0; x < SCREEN_WIDTH; x++)
  {
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
        leds[x + y * SCREEN_WIDTH].r = 4;
    }
  }
}
*/

void mapLeds(float* _bands)
{
  for (int x = 0; x < BANDS; x++)
  {
    CRGB color = redToBlue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      //if ((_bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0) leds[x + SCREEN_WIDTH * y] = color;
            if ((_bands[x] * SCREEN_HEIGHT +y) >= 0) leds[x + SCREEN_WIDTH * y] = color;
     // else if ((_bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
     else if ((_bands[x] * SCREEN_HEIGHT + y) > -1)
      {
        /*
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (_bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (_bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (_bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y) + 0.999);
        */
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (_bands[x] * SCREEN_HEIGHT + y + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (_bands[x] * SCREEN_HEIGHT + y + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (_bands[x] * SCREEN_HEIGHT + y + 0.999);        
      }
      else leds[x + SCREEN_WIDTH * y] = 0x0;
    }
  }
}

uint16_t mapfunction(uint16_t pos)
{
    int panelnumber=pos/256;
    int datainpanel= pos%256;
  int yp=panelnumber/8;
  int Xp=panelnumber%8;
  int Y=yp;
  int X=Xp;

  int x=  datainpanel%16;
   int y=  datainpanel/16;

    if(y%2==0)
      {
        Y=Y*16+y;
        X=X*16+x;
      }
    else
      {
        Y=Y*16+y;
        X=X*16+16-x-1;
      }

      return Y*16*8+X;
}

void mapLeds_textures(int x, FloatOffset groundOffset, FloatOffset skyOffset, float* _bands)
{
    // CRGB color = redToBlue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      CRGB groundColor = ((CRGB*)yellow_sand)[((y + (int)groundOffset.y) % TILE_HEIGHT) * TILE_WIDTH + (x + (int)groundOffset.x) % TILE_WIDTH];
      groundColor.r = groundColor.r ;/// 4;
      groundColor.g = groundColor.g ;/// 64;
      groundColor.b = groundColor.b ;;// 64;
      CRGB skyColor = ((CRGB*)dark_blue_water)[((y + (int)skyOffset.y) % TILE_HEIGHT) * TILE_WIDTH + (x + (int)skyOffset.x) % TILE_WIDTH];
      skyColor.r = skyColor.r ;/// 64;
      skyColor.g = skyColor.g ;/// 64;
      skyColor.b = skyColor.b ;/// 64;
      /*
      uint8_t r = groundColor.r;
      groundColor.r = groundColor.g;
      groundColor.g = r;
      */
      if ((_bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) >= 0)
      {
        leds[x + SCREEN_WIDTH * (SCREEN_HEIGHT - 1 - y)] = groundColor;
      }
      else if ((_bands[x] * SCREEN_HEIGHT - (SCREEN_HEIGHT - 1 - y)) > -1)
      {
        float skyPortion = SCREEN_HEIGHT - 1 - y - _bands[x] * SCREEN_HEIGHT;
        float groundPortion = 1 - skyPortion;
        leds[x + SCREEN_WIDTH * (SCREEN_HEIGHT - 1 - y)].r = (float)groundColor.r * groundPortion + (float)skyColor.r * skyPortion;
        leds[x + SCREEN_WIDTH * (SCREEN_HEIGHT - 1 - y)].g = (float)groundColor.g * groundPortion + (float)skyColor.g * skyPortion;
        leds[x + SCREEN_WIDTH * (SCREEN_HEIGHT - 1 - y)].b = (float)groundColor.b * groundPortion + (float)skyColor.b * skyPortion;
      }
      else
      {
        leds[x + SCREEN_WIDTH * (SCREEN_HEIGHT - 1 - y)] = skyColor;
      }
    }
}

void drawCharacter(const CRGB* character, float startingColumn, float offSet, float columnsPerFrame, int numberOfFrames, int frameWidth, int frameHeight)
{
  // int frame = (int)((startingColumn + offSet) / columnsPerFrame) % numberOfFrames;
  int frame = (int)((startingColumn + offSet) / columnsPerFrame) % numberOfFrames;
  // frame = 0;
  for (int x = (int)(startingColumn + 100) - 100; x < (int)startingColumn + frameWidth; x++)
  {
    if (x >= 0 && x < SCREEN_WIDTH)
    {
      for (int y = SCREEN_HEIGHT - frameHeight; y < SCREEN_HEIGHT; y++)
      {
        int characterPixel = x - (int)(startingColumn + 100) + 100 + frame * frameWidth + (y - SCREEN_HEIGHT + frameHeight) * (numberOfFrames * frameWidth);
        // if (character[characterPixel].r != 0xFE && character[characterPixel].g != 0xFD && character[characterPixel].b != 0xFC)
        if ((*(uint32_t*)&(character[characterPixel]) & 0x00FFFFFF) != 0x00FCFDFE)    // I don't know why it's not 0xFEFDFC. It's because esp32 is little endian.
        {
          leds[x + (SCREEN_HEIGHT - 1 - y) * SCREEN_WIDTH].r = character[characterPixel].r / 4;
          leds[x + (SCREEN_HEIGHT - 1 - y) * SCREEN_WIDTH].g = character[characterPixel].g / 4;
          leds[x + (SCREEN_HEIGHT - 1 - y) * SCREEN_WIDTH].b = character[characterPixel].b / 4;
        }
      }
    }
  }
  
}



/*
void mapLeds()     // upside down
{
  for (int x = 0; x < BANDS; x++)
  {
    CRGB color = redToBlue(((float)x / (BANDS - 1)) * sqrtApprox((float)x / (BANDS - 1)));
    for (int y = 0; y < SCREEN_HEIGHT; y++)
    {
      if ((bands[x] * SCREEN_HEIGHT - y) >= 0) leds[x + SCREEN_WIDTH * y] = color;
      else if ((bands[x] * 32 - y) > -1)
      {
        leds[x + SCREEN_WIDTH * y].r = ((float)color.r + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
        leds[x + SCREEN_WIDTH * y].g = ((float)color.g + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
        leds[x + SCREEN_WIDTH * y].b = ((float)color.b + 0.999) * (bands[x] * SCREEN_HEIGHT - y + 0.999);
      }
      else leds[x + SCREEN_WIDTH * y] = {0, 0, 0};
    }
  }
}
*/

void doWindowing(float* _inputReal)
{
  // first we calculate the average of the signal so we can remove any dc offset
  float average = 0;
  for (int i = 0; i < SAMPLES; i++)
  {
    average += _inputReal[i];
  }
  average /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++)
  {
    _inputReal[i] = (_inputReal[i] - average) * windowingArray[i];
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
    .sample_rate = 44100, // updated automatically by A2DP
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
  };
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_i2s_port(I2S_NUM_1);
  a2dp_sink.set_stream_reader(audio_data_callback);
  
  i2s_pin_config_t my_pin_config = {
      .bck_io_num = 4,
      .ws_io_num = 15,
      .data_out_num = 17,
      .data_in_num = I2S_PIN_NO_CHANGE };
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

void powerOfTwo(float* _output)
{
  for (int index = 0; index < SAMPLES / 2; index++)
  {
    _output[index] = _output[index * 2] * _output[index * 2] + _output[index * 2 + 1] * _output[index * 2 + 1];
  }
}

void powTwoBands(float* _output, float* _bands)
{
  for (int i = 0; i < BANDS; i++)
  {
    _bands[i] = 0;
    #if SAMPLES == 4096
    #if BANDS == 128
    for (int j = bins_4096_128[i]; j < bins_4096_128[i + 1]; j++)
    #endif
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
    for (int j = bins_4096_112[i]/2; j < bins_4096_112[i + 1]/2; j++)
    #endif 
    {
     // i=i+1-1;
      _bands[i]+= _output[j];
    }
  }
}

void logBands(float* _bands, float* _peakBands)
{
  for (int i = 0; i < BANDS; i++)
  {
    _bands[i] = log10f(_bands[i]);
    
    #ifdef PRINT_PEAKS
    if (_peakBands[i] < _bands[i]) _peakBands[i] = _bands[i];
    #endif
    
    #if SAMPLES == 4096
    #if BANDS == 128
    int delta = bins_4096_128[i + 1] - bins_4096_128[i];
    #endif
    #endif 
    #if SAMPLES == 2048
    #if BANDS == 128
    int delta = bins_2048_128[i + 1] - bins_2048_128[i];
    #endif
    #endif 
    
    if (delta < 1) delta = 1;
    if (delta > 6) delta = 6;
    bands[i] = bands[i] - substract_universal[delta - 1] + 6;
    
  }
}

void normalizeBands(float* _bands)
{
  static float bandCeiling = 0.1;
  #ifdef PRINT_CEILING
  Serial.println(String("bandCeiling = ") + bandCeiling);
  #endif
  bandCeiling -= 0.00005;          // now it takes 200 seconds to come 10 dB down
  for (int i = 0; i < BANDS; i++)
  {
    if (_bands[i] > bandCeiling) bandCeiling = _bands[i];
  }
  for (int i = 0; i < BANDS; i++)
  {
    _bands[i] = (_bands[i] - bandCeiling + dynamicRange) / dynamicRange;
  }
}

void zeroSmallBins(float* _output)
{
  float biggest = 0;
  for (int i = 0; i < SAMPLES / 2; i++)
  {
    if (biggest < _output[i]) biggest = _output[i];
  }
  for (int i = 0; i < SAMPLES / 2; i++)
  {
    // we get rid of the unwanted frequency side lobes this way. kaiser 2 is quite eficient and we shouldn't see more than -60 dB on the side lobes
    // at the same time we make sure that we don't take logarithm out of zero. that would be -infinite
    _output[i] = std::max(_output[i] - biggest * 0.0000003, 0.0000000001);
    }
}

void setup()
{
  #ifdef USE_SERIAL
  Serial.begin(115200);
  #endif
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
  
  Serial.printf("numLeds = %d\n", numLeds);
  Serial.printf("SCREEN_WIDTH = %d\n", SCREEN_WIDTH);
  Serial.printf("SCREEN_HEIGHT = %d\n", SCREEN_HEIGHT);
  inputReal = (float*)calloc(SAMPLES, sizeof(float));
    if(!inputReal)
  {
    Serial.printf("unbale to allocatte memory inpout\n");

    return;
  }
  
   output = (float*)calloc(SAMPLES, sizeof(float));
     if(!output)
  {
    Serial.printf("unbale to allocatte memory output \n");
    free(inputReal);
    return;
  }
    
  //int pins[] = {32, 33, 25, 26, 27, 14, 12, 23, 22, 21, 19, 18, 5, 17};              // esp32 dev kit v1
  #ifndef WAIT_UNTIL_DRAWING_DONE
  driver.__displayMode = NO_WAIT;
  #endif
 leds = (CRGB*)calloc(numLeds, sizeof(CRGB));
  //driver.initled((uint8_t*)leds, pins, NUM_PANELS_PER_ROW * NUM_PANELS_PER_COLUMN, PANEL_WIDTH * PANEL_HEIGHT, ORDER_GRB);
  // These two lines have to be after driver.initled()
  
    #ifdef _LEDMAPPING
   driver.setMapLed(&mapfunction);
   #endif
 // driver.initled((uint8_t *)leds, Pins, CLOCK_PIN, LATCH_PIN);
    driver.initled((uint8_t *)leds, Pins, CLOCK_PIN, LATCH_PIN);
    driver.setBrightness(30);
    memset(leds,10,NUM_LEDS*3);
    driver.showPixels();
    fftReal = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, inputReal, output);
    /*

  driver._offsetDisplay.panel_width=SCREEN_WIDTH;
  driver._offsetDisplay.panel_height=SCREEN_HEIGHT;
  
  offd = driver.getDefaultOffset();
  offd.panel_width=SCREEN_WIDTH;
  offd.panel_height=SCREEN_HEIGHT;

  Serial.printf("driver._offsetDisplay.panel_width = %d\n", driver._offsetDisplay.panel_width);
  Serial.printf("driver._offsetDisplay.panel_height = %d\n", driver._offsetDisplay.panel_height);
  */
  groundOffset.x = 0;
  groundOffset.y = 0;
  skyOffset.x = 0;
  skyOffset.y = 0;
  
  startAudio();
  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  delay(100);
  
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  // gpio_set_direction((gpio_num_t)13, GPIO_MODE_OUTPUT);
  // gpio_set_level((gpio_num_t)13, 0);
  
}

int off=0;
void loopBluetooth()
{


  //Serial.printf("\n******************\non start loop:%d\n******************\n",off);
  off++;
  /*
  inputReal = (float*)calloc(SAMPLES, sizeof(float));
    if(!inputReal)
  {
    Serial.printf("unbale to allocatte memory inpout\n");

    return;
  }
  
   output = (float*)calloc(SAMPLES, sizeof(float));
     if(!output)
  {
    Serial.printf("unbale to allocatte memory output \n");
    free(inputReal);
    return;
  }*/
   /*
  bands = (float*)calloc(BANDS, sizeof(float));
  if(!bands)
  {
    Serial.printf("unbale to allocatte memory bands\n");
     free(inputReal);
     free(output);
         return;
  }*/
  /*
  #ifdef PRINT_PEAKS
  static float* peakBands = (float*)calloc(BANDS, sizeof(float));
  #else
  static float* peakBands;
  #endif */
  
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
  
 // Serial.println("windowing");
  doWindowing(inputReal);
  
  //Serial.println("fftreeal");
  fft_execute(fftReal);

  //Serial.println("powerpftwo");
  powerOfTwo(output);   // if we end up doing log() of the output anyways there is no need to do costly sqrt() because it's the same as dividing log() by 2
  // sqrtBins();     // we don't actually need this since we are dealing with the power of the signal and not the amplitude

  //Serial.println("small zero");
  zeroSmallBins(output);  // we do this because there are plenty of of reflections in the surrounding bins

  //Serial.println("powtobands");
  powTwoBands(output, bands);

  //Serial.println("lobabnds");
  logBands(bands, peakBands);
  
  //Serial.println("norm bands");
  normalizeBands(bands);
    //Serial.println("on continue bands");
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
  driver.showPixels();
  #endif
  
  #ifdef PRINT_RAM
  // I had to move these commands here because they bothered I2SCloclessLedDriver
  // They do something that blocks the interruots or something
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
  for (int x = 0; x < SCREEN_WIDTH; x++)
  {
    mapLeds_textures(x, groundOffset, skyOffset, bands);
  }
  
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
    // offset = 100.4 + (float)esp_random() / 1000000.4;
    offset = 100.4 + ((double)esp_random() / (double)4194304);
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
    startingColumn = 0 - frameWidth - 4;
    // Serial.printf("Color of the first pixel is %u\n\r", (*(uint32_t*)&(currentCharacter[0])));
    // it should be 16711164;
    // it's actually 4277992958 because the cpu is little endian
    // that means that the last byte of the number is stored in the beginning
    // all the bytes in a number are in reverse order
    // back in the day when the bit width of the memory lane was less than the bit width of the cpu
    // it allowed the cpu to start the calculations one clock cycle before it got the whole number
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
  driver.showPixels();

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
  //delay(150);
  #endif
 // free( bands );
 // free(inputReal);
  //free(output);
}

void loopMicrophone()
{
  
}

void loopArtnet()
{
  
}

void loop()
{
  switch(programMode)
  {
    case bluetooth:
      while(true) loopBluetooth();
      break;
    case microphone:
      while(true) loopMicrophone();
      break;
    case artnet:
      while(true) loopArtnet();
      break;
  }
}
