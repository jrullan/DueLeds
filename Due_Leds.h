/*
	This is a modified version of the Adafruit_NeoPixel library
	to use exclusively on the Arduino Due for parallel strips manipulation.
	The proposed model is to control 8 simultaneous strips each on its own
	pin by manipulating a port directly using the REG_PIOx_ODSR instruction.
	
	1. All references to __ARM__ have been removed since this code only applies
	to the DUE.		
	
	2. Instead of using a pin, it will use a port number (D for example) and a mask
	for the selected pins.
	
	By: José Rullán
	Date: 2014-09-30
*/

#ifndef DUE_LEDS_H
#define DUE_LEDS_H
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif

//#include "Adafruit_GFX.h"
#include "Adafruit_NeoPixel2.h"
#include "Adafruit_NeoMatrix2.h"
#include "glcdfont.c"
#include "sine32.h"

//	Defines the structure of a color
typedef struct rgbColor{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

// 	Defines an image meta-data
typedef struct image{
	uint8_t brightness;
  uint8_t width;
  uint8_t height;  
  int16_t x;
  int16_t y;
  const struct rgbColor *image;
};

// 	Defines the structure of a sine wave
typedef struct sineWave{
	boolean dir;
	int8_t crestHeight;
	int8_t size;
  int8_t numberOfCycles;
  int8_t yOffset;  
  int8_t xOffset;
  int8_t yDelta;
  int8_t xDelta;
  int8_t hDelta;
  int8_t phase;				//phase in degrees, need conversion to radians
  uint8_t colorIndex;
};

#include "images.h"

class Due_Leds{
	public:
//Methods
		Due_Leds(uint16_t n, uint8_t s, uint8_t r, uint8_t c);
		~Due_Leds();
		
		void
			begin(Tc *t, uint32_t c), // Timer object and channel
			display(void),						// Displays all leds (Simultaneous version)
			display2(void),						// Displays all leds (Sequential version)
			clear(void),							// Clears the screen
			showImage(image img, int16_t xOffset, int16_t yOffset, uint8_t mode=0),
			moveImageLeft(image *img, uint8_t inc),
			moveImageRight(image *img, uint8_t inc),
			moveImageUp(image *img, uint8_t inc),
			moveImageDown(image *img, uint8_t inc),
			//drawChar(int16_t x, int16_t y, unsigned char c,uint16_t color, uint16_t bg, uint8_t size), 	//From Adafruit_GFX, draws a character based on the font defined in glcdfont.h
			setLed(uint32_t n, struct rgbColor color),
			setPixel(int16_t x, int16_t y, struct rgbColor color, uint8_t mode=0),		//Sets a pixel in the matrix
			fillLeds(uint32_t startLed, uint32_t endLed, struct rgbColor color);
		
		// Defined for Adafruit_GFX:
		void drawPixel(int16_t x, int16_t y, uint16_t color);

		uint32_t
			getColor32(uint8_t r, uint8_t g, uint8_t b),
			getLedPos(uint8_t r, uint8_t c);
			
		uint32_t 
			copyPixelRight(uint8_t x, uint8_t y, uint8_t increment),
			copyPixelLeft(uint8_t x, uint8_t y, uint8_t decrement),
			movePixelLeft(uint8_t x, uint8_t y, uint8_t decrement);

//Attributes
		const uint8_t
			cols, 				//Total matrix columns
			rows,					//Total matrix rows
			sCols,				//Sub matrices columns
			sRows,				//Sub matrices rows
			strips;  			//Num of strips
		uint8_t brightness;
		boolean poked;				//Variable to enable/disable the noInterrupts during display
		const uint16_t numLeds;
		struct rgbColor getPixelColorN(int n);
		struct rgbColor getPixelColor(uint8_t r, uint8_t c);
		Adafruit_NeoMatrix2 *matrix;
		uint8_t *leds;
		uint8_t *dataBytes;
		

	private:
		//Methods
		void
			defineColors(),
			stopTimer(),
			startTimer(),
			transposeValues(uint8_t *d);
		uint8_t
			getMatrixNumber(uint32_t ledNumber),
			getRowNumber(uint32_t ledNumber),
			getColNumber(uint32_t ledNumber);
			
		int
			allocateMemory(void),
			rotateRight(int v, int s);
		
		//Attributes
		Tc *tC;			
		int 
			channel,
		 //*colors,
			memoryAllocated,
			wsDelay; //Wait for setup in the WS2812B
};
#endif


//Template to filter by architecture
#ifdef __arm__
#else
 #error "CPU SPEED NOT SUPPORTED"
#endif // end Architecture select
