

/*
TODO
- 4096 (is it even possible?) IT IS POSSIBLE
- 128 bands might be possible too at 45 fps
- Change backgound dynamically
- Animated backgound
  - Perhaps GIFs

- Get samples based on the time the last loop took + 1 %
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


#define FASTLED_ESP32_I2S true
// #define I2S_DEVICE 0

// #include "WiFi.h"
#define FASTLED_RMT_MEM_BLOCKS 2      // setting this to 1 gives me 8 parallel rmt outputs but will cause glitching
// #define FASTLED_ESP32_I2S true
// #define I2S_DEVICE 1
#include "FastLED.h"
FASTLED_USING_NAMESPACE
#include "BluetoothA2DPSink.h"
#include "FFT.h"
// #include "arduinoFFT.h"
// #include "I2SClocklessLedDriver.h"
// I2SClocklessLedDriver driver;

#include "constants.h"

char BTname[] = "alsonot4096";
BluetoothA2DPSink a2dp_sink;
// TaskHandle_t task1, task2, task3;

// has to be between 512 - 2048
// more than 2048 and we run out of ram. less than 512 and we run out of time to draw it on the led panel
// 512 => 86 fps, 1024 => 43 fps, 2048 => 21 fps
#define SAMPLES 4096
int newSamples = 650;
int frameRate = 70;
#define BANDS 64
// const float dynamicRange = (float)DYNAMIC_RANGE / (float)5;    // because we don't do sqrt() our logarithms are twice as large. That is why we divide by 10 and miltiply by 2
const float dynamicRange = 5.0;    // in bels, not decibels. bel is ten decibels. it's metric. bel is the base unit. long live the metric.
// #define PRINT_PLOT
#define DEBUG false
// #define PRINT_BANDS
// #define PRINT_CEILING
#define USE_SERIAL
#define PRINT_OUTPUT
// #define PRINT_PEAKS
#define PRINT_FFT_TIME
#define PRINT_ALL_TIME
#define PRINT_FASTLED_TIME
#define PRINT_RAM
// #define PRINT_INDEXES
// #define TEST_FULL_BUFFER
#define PRINT_SAMPLE_RATE
#define PRINT_LESSER_SAMPLES
#define PRINT_BUFFER_FULL


// int16_t binIntervals[65] = {};
float bands[64] = {};
#ifdef PRINT_PEAKS
float peakBands[64] = {};
#endif
// Hann, Blackman or Flat top is best for separating distant frequencies. Others suck at that and will make the spectrum more flat.
// Hann has the smallest initial spread, Blackman comes after that and Flat top has the widest spread
// I will probably go with Hann which is just a cosine that has been lifted above zero

// This is old information. Kaiser has the best reduction of side lobes and somewhat narrow main lobe. The other windows don't even hold a candle.

int maxCurrent = 2500;
int maxBrightness = 32;

#define BUFFER_LENGTH 4092   // 3000 is also just fine. maybe 4096 has too much extra space
// uint8_t bufferRing[BUFFER_LENGTH] = {};
IRAM_ATTR int32_t bufferRing[BUFFER_LENGTH] = {};
// int16_t bufferRing[BUFFER_LENGTH] = {};
volatile int writeIndex = 0;   // writeIndex will stay 4 behind readIndex. it can't go past
volatile int readIndex = 0;    // readIndex can be equal to writeIndex but cannot advance
volatile int bufferFull = 0;
volatile int lesserSamples = 0;

float inputReal[SAMPLES] = {};
float output[SAMPLES] = {};
// float windowingArray[SAMPLES] = {};
// float realRing[SAMPLES] = {};

IRAM_ATTR int32_t realRing[SAMPLES] = {};
// int32_t realRing[SAMPLES] = {};

int realRingIndex = 0;

const int numLeds = 2048;
// uint8_t *leds = NULL;
CRGB leds[numLeds];
// CRGB ledsTemp[numLeds];         // remove this later
// CRGB templateLeds[numLeds];
// uint16_t ledMap[numLeds] = {};
volatile unsigned int beebBoob = 178;    // this is how I keep track of how many frames have been shown


fft_config_t *fftReal = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, inputReal, output);


/*
void printLedMap()
{
  Serial.println();
  Serial.print("const uint16_t templateLeds[] PROGMEM = {");
  for (int i = 0; i < numLeds; i++)
  {
    if (i % 20 == 0) Serial.println();
    Serial.print(ledMap[i]);
    Serial.print(", ");
  }
  Serial.println("};");
}

void generateLedMap()
{
  for  (int panel = 0; panel < BANDS / 16; panel++)
  {
    for  (int i = 0; i < 16; i++)
    {
      for (int j = 0; j < 16; j++)
      {
        if (i % 2 != 0) ledMap[i * 16 + j + panel * 256] = 16 * (16 - 1 - i) + j + panel * 256;
        else ledMap[i * 16 + j + panel * 256] = 16 * (16 - 1 - i) + 15 - j + panel * 256;
      }
    }
  }
  printLedMap();
}

CRGB redToBlue(float hue)
{
  CRGB color;
  color.r = std::max((float)255 * (1 - 2 * hue), (float)0);
  color.g = std::max(std::min(hue * 2 * 255, (1 - hue) * 2 * 255), (float)0);
  color.b = std::max((float)255 * (hue * 2 - 1), (float)0);
  return color;
}

void printTemplateLeds()
{
  Serial.println();
  Serial.print("const CRGB templateLeds[] PROGMEM = {");
  for (int i = 0; i < numLeds; i++)
  {
    if (i % 10 == 0) Serial.println();
    Serial.print("{");
    Serial.print(templateLeds[i][0]);
    Serial.print(", ");
    Serial.print(templateLeds[i][1]);
    Serial.print(", ");
    Serial.print(templateLeds[i][2]);
    Serial.print("}");
    Serial.print(", ");
  }
  Serial.println("};");
}

void generateTemplateHSV()
{
  for  (int i = 0; i < BANDS; i++)
  {
    for (int j = 0; j < 16; j++)
    {
      // Serial.println(String("CHSV ") + (i * 160 / 31) + " " + (255 - 128 * j / 15) + " 16");
      // CRGB color = CHSV(i * 160 / 31, 255, 255);
      // CRGB color = CHSV(powf(i, 1.5) / powf(31, 1.5) * 160.0, 255, 255);
      CRGB color = redToBlue(powf((float)i / (BANDS - 1), 1.6));
      templateLeds[i * 16 + j].r = color.r / 11;
      templateLeds[i * 16 + j].g = color.g / 9;
      templateLeds[i * 16 + j].b = color.b / 17;
    }
  }
  printTemplateLeds();
}
*/

void mapLeds()
{
  for (int i = 0; i < BANDS; i++)
  {
    for (int j = 0; j < 16; j++)
    {
      // if ((bands[i] * 16 - j) > 0) leds[(ledMap[i * 16 + j]) * 3 + 1] = 1;
      // else leds[(ledMap[i * 16 + j]) * 3 + 1] = 0;
      if ((bands[i] * 16 - j) >= 0) leds[ledMap[i * 16 + j]] = templateLeds[i * 16 + j];
      else if ((bands[i] * 16 - j) > -1)
      {
        leds[ledMap[i * 16 + j]][0] = ((float)templateLeds[i * 16 + j][0] + 1.0) * (bands[i] * 16.0 - j + 1.0);   // I add one to the color so that if it's small it doesn't disappear first. now if the color is one, it will disappear in the middle.
        leds[ledMap[i * 16 + j]][1] = ((float)templateLeds[i * 16 + j][1] + 1.0) * (bands[i] * 16.0 - j + 1.0);
        leds[ledMap[i * 16 + j]][2] = ((float)templateLeds[i * 16 + j][2] + 1.0) * (bands[i] * 16.0 - j + 1.0);
      }
      else leds[ledMap[i * 16 + j]] = CRGB::Black;
    }
  }
}

/*
void generateBinIntervals()
{
  for (int i = 0; i < BANDS + 1; i++)
  {
    if (SAMPLES == 4096 && BANDS == 64) binIntervals[i] = bins_4096_64[i];
    if (SAMPLES == 4096 && BANDS == 48) binIntervals[i] = bins_4096_48[i];
    if (SAMPLES == 4096 && BANDS == 32) binIntervals[i] = bins_4096_32[i];
    if (SAMPLES == 4096 && BANDS == 16) binIntervals[i] = bins_4096_16[i];
    
    if (SAMPLES == 2048 && BANDS == 64) binIntervals[i] = bins_2048_64[i];
    if (SAMPLES == 2048 && BANDS == 48) binIntervals[i] = bins_2048_48[i];
    if (SAMPLES == 2048 && BANDS == 32) binIntervals[i] = bins_2048_32[i];
    if (SAMPLES == 2048 && BANDS == 16) binIntervals[i] = bins_2048_16[i];
  }
  if (binIntervals[BANDS] > SAMPLES / 2) binIntervals[BANDS] = SAMPLES / 2;
  Serial.print("Last binInterval = ");
  Serial.println(binIntervals[BANDS]);
}
*/

/*
void createWindowing(float a, float multiplier)
{
  float temp = 0;
  Serial.print("const float windowingArray[] PROGMEM = {");
  for (int i = 0; i < SAMPLES; i++)
  {
    temp = ((float)a - (1.0 - a) * (cos(TWO_PI * (float)i / (float)(SAMPLES - 1)))) * multiplier;    // a = 0.5 (Hann), a = 0.54 (Hamming)
    if (a == -1) temp = (0.44959 - (0.49364 * (cos(TWO_PI * (float)i / (float)(SAMPLES - 1)))) + (0.05677 * (cos(2 * TWO_PI * (float)i / (float)(SAMPLES - 1))))) * multiplier;    // Blackman
    if (a == -2) temp = (0.42323 - (0.49755 * (cos(TWO_PI * (float)i / (float)(SAMPLES - 1)))) + (0.07922 * (cos(2 * TWO_PI * (float)i / (float)(SAMPLES - 1))))) * multiplier;    // Blackman
    if (a == -3) temp = 1 * multiplier;     // -3 equals no windowing
    if (a == -4) temp = (0.21557895
        - 0.416631580 * cos(1 * TWO_PI * (float)i / (float)(SAMPLES - 1))
        + 0.277263158 * cos(2 * TWO_PI * (float)i / (float)(SAMPLES - 1))
        - 0.083578947 * cos(3 * TWO_PI * (float)i / (float)(SAMPLES - 1))
        + 0.006947368 * cos(4 * TWO_PI * (float)i / (float)(SAMPLES - 1))) * multiplier;    // flat top
    if (a == -5) temp = (0.355768
        - 0.487396 * cos(1 * TWO_PI * (float)i / (float)(SAMPLES - 1))
        + 0.144232 * cos(2 * TWO_PI * (float)i / (float)(SAMPLES - 1))
        - 0.012604 * cos(3 * TWO_PI * (float)i / (float)(SAMPLES - 1))
        + 0.0 * cos(4 * TWO_PI * (float)i / (float)(SAMPLES - 1))) * multiplier;    // nuttal
    if (i % 10 == 0) Serial.println();
    Serial.print(temp, 10);
    Serial.print(", ");
  }
  Serial.println("};");
}
*/

void doWindowing()
{
  // first we calculate the average of the signal so we can remove any dc offset
  float average = 0;
  for (int i = 0; i < SAMPLES; i++)
  {
    average += inputReal[i];
  }
  average /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++)
  {
    inputReal[i] = (inputReal[i] - average) * windowingArray[i];
  }
}


void audio_data_callback(const uint8_t *data, uint32_t byteLen)
{
  int actualLen = byteLen / 4;
  // int free = (mHead - mTail + mSize - 1)%mSize;
  // if ((readIndex - writeIndex + BUFFER_LENGTH - 1) % BUFFER_LENGTH <= BUFFER_LENGTH / 4)
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
  /*
  else if (writeIndex + actualLen <= BUFFER_LENGTH)
  {
    // memcpy(&bufferRing[writeIndex], &data[0], len);
    for (int i = i; i < actualLen; i++)
    {
      // bufferRing[writeIndex + i] = (int16_t)(((int)*(int16_t*)&(data[readIndex * 4]) + (int)*(int16_t*)&(data[readIndex * 4 + 2])));
      bufferRing[writeIndex + i] = *(int16_t*)&(data[readIndex * 4]) / 2 + *(int16_t*)&(data[readIndex * 4 + 2]);
    }
    writeIndex = (writeIndex + actualLen) % (BUFFER_LENGTH);
  }
  else
  {
    int written = BUFFER_LENGTH - writeIndex;
    for (int i = i; i < written; i++)
    {
      // bufferRing[writeIndex + i] = (int16_t)(((int)*(int16_t*)&(data[readIndex * 4]) + (int)*(int16_t*)&(data[readIndex * 4 + 2])));
      bufferRing[writeIndex + i] = *(int16_t*)&(data[readIndex * 4]) / 2 + *(int16_t*)&(data[readIndex * 4 + 2]);
    }
    for (int i = i; i < actualLen - written; i++)
    {
      bufferRing[i] = *(int16_t*)&(data[(written + i) * 4]) / 2 + *(int16_t*)&(data[(written + i) * 4 + 2]);
    }
    // memcpy(&bufferRing[0], &data[written], len - written);
    writeIndex = (writeIndex + actualLen) % (BUFFER_LENGTH);
  }
  */
  #ifdef PRINT_INDEXES
  Serial.print("Got ");
  Serial.print(actualLen);
  Serial.print(" data, writeIndex = ");
  Serial.println(writeIndex);
  #endif
}

/* This used the float. I had to switch to int16_t to save 16 kB
void audio_data_callback(const uint8_t *data, uint32_t len)
{
  // int free = (mHead - mTail + mSize - 1)%mSize;
  // if ((readIndex - writeIndex + BUFFER_LENGTH - 1) % BUFFER_LENGTH <= BUFFER_LENGTH / 4)
  if ((readIndex - writeIndex + BUFFER_LENGTH - 1) % BUFFER_LENGTH <= len)
  {
    bufferFull++;
  }
  else if (writeIndex + len <= BUFFER_LENGTH)
  {
    memcpy(&bufferRing[writeIndex], &data[0], len);
    writeIndex = (writeIndex + len) % BUFFER_LENGTH;
  }
  else
  {
    int written = BUFFER_LENGTH - writeIndex;
    memcpy(&bufferRing[writeIndex], &data[0], written);
    memcpy(&bufferRing[0], &data[written], len - written);
    writeIndex = (writeIndex + len) % BUFFER_LENGTH;
  }
  #ifdef PRINT_INDEXES
  Serial.println(String("Got ") + len + " data, writeIndex = " + writeIndex);
  #endif
}
*/

void startAudio()
{
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000, // updated automatically by A2DP
    .bits_per_sample = (i2s_bits_per_sample_t)16,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = true,
    .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
  };
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_i2s_port(I2S_NUM_1);
  a2dp_sink.set_stream_reader(audio_data_callback);
  a2dp_sink.start(BTname);
  Serial.println(String("Started Bluetooth audio receiver with the name ") + BTname);
}

bool readBuffer()
{
  int newSamples = std::min(SAMPLES, a2dp_sink.i2s_config.sample_rate / frameRate);
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

/* This used the float. I had to switch to int16_t to save 16 kB
bool readBuffer()
{
  int newSamples = std::min(SAMPLES, a2dp_sink.i2s_config.sample_rate / frameRate);
  if ((writeIndex - readIndex + BUFFER_LENGTH) % BUFFER_LENGTH < newSamples * 2)   // newSamples should be multiplied by four but by multiplying it by 2 i only ask for half as many samples to be present
  {
    // delay(1);
    return false;
  }
  if ((writeIndex - readIndex + BUFFER_LENGTH) % BUFFER_LENGTH < newSamples * 4)
  {
    // delay(1);
    newSamples = ((writeIndex - readIndex + BUFFER_LENGTH) % BUFFER_LENGTH) / 4;
    lesserSamples = newSamples;
  }
  
  {
    for (int i = 0; i < newSamples; i++)
    {
      realRing[realRingIndex] = (float)*(int16_t*)&(bufferRing[readIndex]) + (float)*(int16_t*)&(bufferRing[readIndex + 2]);
      realRingIndex = (realRingIndex + 1) % SAMPLES;
      readIndex = (readIndex + 4) % BUFFER_LENGTH;
    }
    #ifdef PRINT_INDEXES
    Serial.println(String("readIndex = ") + readIndex + ", realRingIndex = " + realRingIndex);
    #endif
  }

  memcpy(&inputReal[0], &realRing[realRingIndex], (SAMPLES - realRingIndex) * sizeof(float));
  memcpy(&inputReal[SAMPLES - realRingIndex], &realRing[0], (realRingIndex) * sizeof(float));
  return true;
}
*/

void powerOfTwo()
{
  for (int index = 0; index < SAMPLES / 2; index++)
  {
    // Serial.println(String(index) + ": " + output[index * 2]);
    output[index] = output[index * 2] * output[index * 2] + output[index * 2 + 1] * output[index * 2 + 1];
    // output[index] = output[index * 2] * output[index * 2]; // + output[index * 2 + 1] * output[index * 2 + 1];
    // outputPowTwo[index] = output[index * 2];
  }
}

float sqrtApprox(float number)
{
  union { float f; uint32_t u; } y = {number};
  y.u = 0x5F1FFFF9ul - (y.u >> 1);
  return number * 0.703952253f * y.f * (2.38924456f - number * y.f * y.f);
}

void powTwoBands()
{
  for (int i = 0; i < BANDS; i++)
  {
    bands[i] = 0;
    #if SAMPLES == 4096
    for (int j = bins_4096_64[i]; j < bins_4096_64[i + 1]; j++)
    #endif 
    #if SAMPLES == 2048
    for (int j = bins_2048_64[i]; j < bins_2048_64[i + 1]; j++)
    #endif 
    {
      bands[i] += output[j];
    }
  }
}

void logBands()
{
  for (int i = 0; i < BANDS; i++)
  {
    bands[i] = log10f(bands[i]);
    #ifdef PRINT_PEAKS
    if (peakBands[i] < bands[i]) peakBands[i] = bands[i];
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
  }
}

void normalizeBands()
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

void zeroSmallBins()
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

void setup()
{
  #ifdef USE_SERIAL
  Serial.begin(115200);
  #endif
  Serial.println("Just booted up");
  // Serial.println(String(testSqrt(1000) * 1000000) + " is the maximum error of sqrtApprox");
  
  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  delay(100);

  // with 0.543478261 it's Hamming windowing. with 0.5 it's Hann windowing
  // we also get free scaling because we can scale the windowing array in advance
  // createWindowing(25.0/46.0, 1.0/256.0);  // 1.0/32.0 is the lowest sane minimum   // doesn't need to be a power of two. just a habit
  // createWindowing(-1, 1.0/256.0);  // a = 1 creates Blackman window -61 dB
  // createWindowing(-4, 1.0/256.0);  // a = -4 creates flat top window -67 dB
  // createWindowing(-2, 1.0/256.0);  // a = -1 creates no windowing
  // Reducing side lobes is quite a problem since in the last bands I add like a hundred of them together. That exarbates the problem by 20 dB. That's not neglible.
  
  // createWindowing(-2, 1.0/256.0);  // a = 0.5 Hann windowing <= this one is my current preference because the side lobes very far away will be quite reduced
  // delay(1000);
  // generateBinIntervals();
  
  // generateTemplateHSV();
  
  // Serial.println("1");
  // leds = (uint8_t *)malloc(sizeof(uint8_t) * numLeds * 3);
  for (int i = 0; i < sizeof(uint8_t) * numLeds * 3; i++)
  {
    leds[i] = 0;
  }
  // Serial.println("2");
  
  // generateLedMap();
  
  // Serial.println("3");
  // driver.initled((uint8_t*)leds, pins, 2, 256, ORDER_GRB);
  FastLED.addLeds<NEOPIXEL, 33>(leds, 0*256, 256);
  #if BANDS > 16
  FastLED.addLeds<NEOPIXEL, 32>(leds, 1*256, 256);
  #if BANDS > 32
  FastLED.addLeds<NEOPIXEL, 12>(leds, 2*256, 256);
  #if BANDS > 48
  FastLED.addLeds<NEOPIXEL, 14>(leds, 3*256, 256);
  /*
  FastLED.addLeds<NEOPIXEL, 16>(leds, 3*256, 256);
  FastLED.addLeds<NEOPIXEL, 17>(leds, 3*256, 256);
  FastLED.addLeds<NEOPIXEL, 18>(leds, 3*256, 256);
  FastLED.addLeds<NEOPIXEL, 19>(leds, 3*256, 256);
  */
  #endif
  #endif
  #endif
  FastLED.setMaxPowerInVoltsAndMilliamps(5, maxCurrent);

  startAudio();

}

void loop()
{
  static unsigned int beebBoob = 178;
  beebBoob++;
  for (;;)
  {
    if (readBuffer()) break;
    else delayMicroseconds(100);
  }

  // debug("buffer read");
  #ifdef PRINT_FFT_TIME
  unsigned int fftMicros = micros();
  #endif
  
  doWindowing();
  // debug("windowing done");
  fft_execute(fftReal);
  #ifdef PRINT_PLOT
  plot();
  #endif
  // plotInput();
  // debug("fftReal executed");
  powerOfTwo();   // if we end up doing log() of the output anyways there is no need to do costly sqrt() because it's the same as dividing log() by 2
  // sqrtBins();     // we don't actually need this since we are dealing with the power of the signal and not the amplitude
  // Serial.println(String("sqrt took ") + (micros() - sqrtMicros) + " μs");
  zeroSmallBins();  // we do this because there are plenty of of reflections in the surrounding bins
  // debug("powerOfTwo");
  // I'll have to get rid of powTwoBands and actually do sqrt() on every bin since I'm goin to add them all up
  powTwoBands();
  logBands();
  normalizeBands();
  // superNormalizeBands();      // maybe we'll do something like this later. now i need full info for debugging and testing

  #ifdef PRINT_BANDS
  printBands();
  #endif
  // printOutput();
  // logFFT();     // logarithm is super costly. that's why we only do it once per band instead of once per bin in the logBands()
  // debug("logFFT");
  // if (PLOTTER) plot();
  // Serial.println("5");
  mapLeds();
  // Serial.println("6");
  // driver.setBrightness(limitCurrent(leds, numLeds, maxCurrent, maxBrightness));
  // Serial.println("7");
  #ifdef PRINT_FFT_TIME
  if (beebBoob % 179 == 0)
  {
    Serial.print("FFT took ");
    Serial.print(micros() - fftMicros);
    Serial.println(" μs");
  }
  #endif
  #ifdef PRINT_RAM
  int heap_caps_get_largest_free_block_8bit_min = 1000000000;
  int heap_caps_get_largest_free_block_32bit_min = 1000000000;
  int newValue = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  if (newValue < heap_caps_get_largest_free_block_8bit_min) heap_caps_get_largest_free_block_8bit_min = newValue;
  newValue = heap_caps_get_largest_free_block(MALLOC_CAP_32BIT);
  if (newValue < heap_caps_get_largest_free_block_32bit_min) heap_caps_get_largest_free_block_32bit_min = newValue;
  if (beebBoob % 179 == 0)
  {
    Serial.print("ESP.getFreeHeap() = ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
    Serial.println(heap_caps_get_largest_free_block_8bit_min);
    Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
    Serial.println(heap_caps_get_largest_free_block_32bit_min);
    heap_caps_get_largest_free_block_8bit_min = 1000000000;
    heap_caps_get_largest_free_block_32bit_min = 1000000000;
  }
  #endif
  // printLeds();
  // driver.showPixels((uint8_t*)leds);
  unsigned int fastledMicros = micros();
  FastLED.show();
  #ifdef PRINT_FASTLED_TIME
  if (beebBoob % 179 == 0)
  {
    Serial.print("FastLED took ");
    Serial.print(micros() - fastledMicros);
    Serial.println(" μs");
  }
  #endif
  #ifdef PRINT_SAMPLE_RATE
  if (beebBoob % 179 == 0)
  {
    Serial.print("a2dp_sink.i2s_config.sample_rate = ");
    Serial.println(a2dp_sink.i2s_config.sample_rate);
  }
  #endif
  #ifdef PRINT_ALL_TIME
  static unsigned int previousMicros;
  unsigned int newMicros = micros();
  if (beebBoob % 179 == 0)
  {
    Serial.print("everything took ");
    Serial.print(newMicros - previousMicros);
    Serial.print(" μs, FFT and FastLED took ");
    Serial.print(newMicros - fftMicros);
    Serial.println(" μs");
  }
  #endif
  // Serial.println(String("readIndex = ") + readIndex);
  #ifdef PRINT_PEAKS
  if (beebBoob % 179 == 0)
  {
    Serial.print("Peak bands: ");
    for (int i = 0; i < BANDS; i++)
    {
      Serial.print(peakBands[i] - 6);
      Serial.print(", ");
    }
    Serial.println();
  }
  #endif
  #ifdef PRINT_LESSER_SAMPLES
  if ((beebBoob % 179 == 0) && (lesserSamples > 0))
  {
    Serial.print("lesserSamples = ");
    Serial.println(lesserSamples);
    lesserSamples = 0;
  }
  #endif
  #ifdef PRINT_BUFFER_FULL
  if ((beebBoob % 7727 == 0) && (bufferFull > 0))
  {
    Serial.print("BUFFER FULL. DISCARDING DATA. BUFFER FULL. DISCARDING DATA. BUFFER FULL. bufferFull = ");
    Serial.println(bufferFull);
    /*
    newSamples += 10;
    Serial.print("newSamples = ");
    Serial.println(newSamples);
    */
    bufferFull = 0;
  }
  #endif
  #ifdef PRINT_ALL_TIME
  previousMicros = newMicros;
  #endif
  // delayMicroseconds(100);
  #ifdef TEST_FULL_BUFFER
  delay(100);
  #endif
}
