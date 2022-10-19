

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
https://github.com/fakufaku/esp32-fft                         // This is the fastest FFT library I could find. Analyzing 4096 samples takes 5 milliseconds.
https://github.com/pschatzmann/ESP32-A2DP
*/

#include "BluetoothA2DPSink.h"
#include "FFT.h"

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

#include "constants.h"

#include "I2SClocklessLedDriver.h"
I2SClocklessLedDriver driver;

char BTname[] = "4096";
BluetoothA2DPSink a2dp_sink;
// TaskHandle_t task1, task2, task3;

// has to be between 512 - 2048
// more than 2048 and we run out of ram. less than 512 and we run out of time to draw it on the led panel
// 512 => 86 fps, 1024 => 43 fps, 2048 => 21 fps
#define SAMPLES 4096
#define NUM_PANELS_X 4
#define NUM_PANELS_Y 1
#define WAIT
int newSamples = 650;
int frameRate = 115;           // here for historical reasons. Will be depricated soon.
#define SECONDS_BETWEEN_DEBUG 15
int framesBetweenDebug = SECONDS_BETWEEN_DEBUG * frameRate;
// #define USE_DOUBLE_BUFFERING
#define BANDS 64
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
#define PRINT_MAPLEDS_TIME
#define PRINT_RAM                   // This uses some resources that block the isr and take a long time
// #define PRINT_INDEXES
// #define TEST_FULL_BUFFER
#define PRINT_SAMPLE_RATE
#define PRINT_LESSER_SAMPLES
#define PRINT_BUFFER_FULL


float bands[64] = {};
#ifdef PRINT_PEAKS
float peakBands[64] = {};
#endif

// Kaiser windowing has the best reduction of side lobes and somewhat narrow main lobe. The other windows don't even hold a candle.

int maxCurrent = 2500;
int maxBrightness = 32;

#define BUFFER_LENGTH 2048   // 2048 is also just fine. maybe 4096 has too much extra space
IRAM_ATTR int32_t bufferRing[BUFFER_LENGTH] = {};
volatile int writeIndex = 0;   // writeIndex will stay 4 behind readIndex. it can't go past
volatile int readIndex = 0;    // readIndex can be equal to writeIndex but cannot advance
volatile int bufferFull = 0;  
volatile int lesserSamples = 0;

float inputReal[SAMPLES] = {};
float output[SAMPLES] = {};

IRAM_ATTR int32_t realRing[SAMPLES] = {};

int realRingIndex = 0;
const int panelSize = 256;

#ifdef USE_DOUBLE_BUFFERING
const int numLeds = panelSize * NUM_PANELS_X * NUM_PANELS_Y * 2;
#else
const int numLeds = panelSize * NUM_PANELS_X * NUM_PANELS_Y;
#endif

// uint8_t *leds = NULL;
CRGB *leds;

fft_config_t *fftReal = fft_init(SAMPLES, FFT_REAL, FFT_FORWARD, inputReal, output);

uint32_t loopMicros;
uint32_t loopCycles;


CRGB redToBlue(float hue)
{
  CRGB color;
  color.r = std::max((float)255 * (1 - 2 * hue), (float)0);
  color.g = std::max(min(hue * 2 * 255, (1 - hue) * 2 * 255), (float)0);
  color.b = std::max((float)255 * (hue * 2 - 1), (float)0);
  return color;
}

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
    .use_apll = true,
    .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
  };
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.set_i2s_port(I2S_NUM_1);
  a2dp_sink.set_stream_reader(audio_data_callback);
  /*
  i2s_pin_config_t my_pin_config = {
      .bck_io_num = 15,
      .ws_io_num = 2,
      .data_out_num = 4,
      .data_in_num = I2S_PIN_NO_CHANGE };
  a2dp_sink.set_pin_config(my_pin_config);
  */
  a2dp_sink.start(BTname);
  Serial.println(String("Started Bluetooth audio receiver with the name ") + BTname);
}

bool readBuffer()
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

void powerOfTwo()
{
  for (int index = 0; index < SAMPLES / 2; index++)
  {
    output[index] = output[index * 2] * output[index * 2] + output[index * 2 + 1] * output[index * 2 + 1];
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
    #if BANDS == 64
    for (int j = bins_4096_64[i]; j < bins_4096_64[i + 1]; j++)
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
  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  delay(100);
  
  leds = (CRGB *)calloc(numLeds * 3, sizeof(uint8_t));
  
  // int pins[] = {32, 33, 25, 26, 27, 12, 13, 14, 15, 16, 17, 5, 18, 19, 21, 22, 23, 4, 0, 2, 1, 3};
  int pins[] = {33, 32, 12, 14, 27, 0, 13, 0, 15, 16, 17, 5, 18, 19, 21, 22, 23, 4, 0, 2, 1, 3};              // esp32 dev kit v1
  #ifndef WAIT
  driver.__displayMode = NO_WAIT;
  #endif
  driver.initled((uint8_t*)leds, pins, 7, panelSize, ORDER_GRB);
  
  startAudio();
  Serial.print("ESP.getFreeHeap() = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  Serial.print("heap_caps_get_largest_free_block(MALLOC_CAP_32BIT) = ");
  Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_32BIT));
  delay(100);
}

void loop()
{
  static unsigned int beebBoob = 178;
  static uint32_t previousMicros = 0;
  static uint32_t previousCycles = 0;
  uint32_t newMicros = micros();;
  uint32_t newCycles = xthal_get_ccount();
  loopMicros = newMicros - previousMicros;
  loopCycles = newCycles - previousCycles;
  previousMicros = newMicros;
  previousCycles = newCycles;
  beebBoob++;
  while (true)
  {
    if (readBuffer()) break;
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
  
  doWindowing();
  fft_execute(fftReal);
  powerOfTwo();   // if we end up doing log() of the output anyways there is no need to do costly sqrt() because it's the same as dividing log() by 2
  // sqrtBins();     // we don't actually need this since we are dealing with the power of the signal and not the amplitude
  zeroSmallBins();  // we do this because there are plenty of of reflections in the surrounding bins
  powTwoBands();
  logBands();
  normalizeBands();
  
  #ifdef PRINT_FFT_TIME
  fftMicros = micros() - fftMicros;
  fftCycles = xthal_get_ccount() - fftCycles;
  #endif
  #ifndef WAIT
  driver.waitUntilDone();
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
  
  mapLeds();

  mapLedsMicros = micros() - mapLedsMicros;
  mapLedsCycles = xthal_get_ccount() - mapLedsCycles;
  
  uint32_t showPixelsMicros = micros();
  uint32_t showPixelsCycles = xthal_get_ccount();
  
  driver.showPixels((uint8_t *)leds);

  showPixelsMicros = micros() - showPixelsMicros;
  showPixelsCycles = xthal_get_ccount() - showPixelsCycles;
  
  #ifdef PRINT_PLOT
  plot();
  #endif
  #ifdef PRINT_BANDS
  printBands();
  #endif
  if (beebBoob % framesBetweenDebug == 0)
  {
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
      Serial.print(peakBands[i] - 6);
      Serial.print(", ");
    }
    Serial.println();
  #endif
  #ifdef PRINT_LESSER_SAMPLES
  if ((beebBoob % framesBetweenDebug == 0) && (lesserSamples > 0))
    Serial.print("lesserSamples = ");
    Serial.println(lesserSamples);
    lesserSamples = 0;
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
