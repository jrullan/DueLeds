 #include "Due_Leds.h"
 #include "arm_ass_instructions.h"

//Constructor and Destructor
//=====================================================================
Due_Leds::Due_Leds(uint16_t n, uint8_t s, uint8_t r, uint8_t c): 
numLeds(n),strips(s),rows(r),cols(c),sCols(c),sRows(r/s)
{}

Due_Leds::~Due_Leds(){
	if(dataBytes) free(dataBytes);
	if(leds) free(leds);
	
	delete matrix;
}


// Public Methods
//=====================================================================
/* Begin Method
 * 
 *	Timer Counter explanations:
 * 	The processor has three Timer Counters (TC0,TC1,TC2)
 * 	but each one has three channels. What we use is one of the channels
 * 	so if we want to use the first channel of TC1 then we are
 * 	actually using channel TC3.
 * 	TC0:
 * 		TC0, TC1, TC2
 * 	TC1:
 * 		TC3, TC4, TC5
 * 	TC2:
 * 		TC6, TC7, TC8
 * 	The Timer Counters are powered by the PMC, so it is necessary to 
 * 	enable it for the specific timer channel.
 * 	Timer_Clock1 = Master clock / 2
 */
void Due_Leds::begin(Tc *pTc, uint32_t c){
	int irqN;
	poked = false;
	brightness = 255;
	channel = c;
	tC = pTc;

	if(tC == TC0){ 
		irqN = TC0_IRQn;
	}
	if(tC == TC1){
		irqN = TC3_IRQn;
	}
	if(tC == TC2){
		irqN = TC6_IRQn;
	}	 

	// 	Enable clock for peripherals and configure Timer/Counter to wave
	//	generator with a 42MHz clock
	pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irqN); 
  TC_Configure(tC, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP | TC_CMR_TCCLKS_TIMER_CLOCK1);
  
  //	Set PORTD's pins (0-7) to output write enable
  REG_PIOD_PER = 0xFF;
  REG_PIOD_OER = 0xFF;
  REG_PIOD_OWER = 0xFF;
  
  //	Allocate memory for data structures required
  memoryAllocated = allocateMemory();
  
  // Instances of member classes
  matrix = new Adafruit_NeoMatrix2(120,64,leds,6,NEO_MATRIX_TOP+NEO_MATRIX_LEFT+NEO_MATRIX_ROWS+NEO_MATRIX_PROGRESSIVE,NEO_GRB+NEO_KHZ800);
  
  //matrix->myDueLeds = this; //Pass pointer to matrix, so it can use this class setPixel method
  //Note myDueLeds is really declared in NeoPixel class from which
  //NeoMatrix inherits.
  
}

/* Display LEDs Method
 * 
 *	This method will send data to the Leds simultaneously to the 8 strips 
 */
void Due_Leds::display(){
  #define SCALE      VARIANT_MCK / 2UL / 1000000UL //42
  #define INST       (2UL * F_CPU / VARIANT_MCK) //2
  #define TIME_800_0 ((int)(0.25 * SCALE + 0.5) - (5 * INST)) 
  #define TIME_800_1 ((int)(0.80 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_800 ((int)(1.25 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_0 ((int)(0.50 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_1 ((int)(1.20 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_400 ((int)(2.50 * SCALE + 0.5) - (5 * INST))
  int times, period, time0, time1, pinMask, numBytes,pos;
  volatile WoReg *timeValue, *timeReset;
  volatile uint8_t *dataByte, *ptr;
  
  
  startTimer();
  transposeValues(dataBytes);
	
  //Show Leds
  while((micros()-wsDelay)<50L);

  timeValue = &(tC->TC_CHANNEL[channel].TC_CV);
  timeReset = &(tC->TC_CHANNEL[channel].TC_CCR);
  dataByte = dataBytes; //Points to the array of transposed bytes
  
  REG_PIOD_ODSR = 0x00;
  times = 24*numLeds;
  period = 43;//PERIOD_800;
  time0 = 1; // 0 value = 1-13
  time1 = 19; // 1 value = 18-31
  pinMask = 0xFF;
  
  if(!poked) noInterrupts(); //Added to be able to listen to bluetooth messages
  do{
		while(*timeValue < period);
		REG_PIOD_ODSR = 0xFF;
		*timeReset = TC_CCR_CLKEN | TC_CCR_SWTRG; //this approach saves lots of cycles (5) vs. calling the subroutine
		while(*timeValue < time0); // 2

			REG_PIOD_ODSR = *dataByte++ & pinMask;
			while(*timeValue < time1);

		REG_PIOD_ODSR = 0x00; // 4
	}while(times--);
	interrupts();
	
	while(*timeValue < period);
	wsDelay = micros();
	stopTimer();
	
}


/* Display LEDs Method
 * 
 *	This method will send data to the Leds sequentially
 */
void Due_Leds::display2(){
  #define SCALE      VARIANT_MCK / 2UL / 1000000UL //42
  #define INST       (2UL * F_CPU / VARIANT_MCK) //2
  #define TIME_800_0 ((int)(0.25 * SCALE + 0.5) - (5 * INST)) 
  #define TIME_800_1 ((int)(0.80 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_800 ((int)(1.25 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_0 ((int)(0.50 * SCALE + 0.5) - (5 * INST))
  #define TIME_400_1 ((int)(1.20 * SCALE + 0.5) - (5 * INST))
  #define PERIOD_400 ((int)(2.50 * SCALE + 0.5) - (5 * INST))
 
  volatile WoReg *timeValue, *timeReset;
  volatile uint8_t *ptr0,*ptr1,*ptr2,*ptr3,*ptr4,*ptr5,*ptr6,*ptr7;
  
  
  startTimer();
	
  //Show Leds
  while((micros()-wsDelay)<50L);

  timeValue = &(tC->TC_CHANNEL[channel].TC_CV);
  timeReset = &(tC->TC_CHANNEL[channel].TC_CCR);
  
  //Pointers to each strip:
  ptr0 = leds;
  ptr1 = ptr0 + (numLeds * 4);
  ptr2 = ptr1 + (numLeds * 4);
  ptr3 = ptr2 + (numLeds * 4);
  ptr4 = ptr3 + (numLeds * 4);
  ptr5 = ptr4 + (numLeds * 4);
  ptr6 = ptr5 + (numLeds * 4);
  ptr7 = ptr6 + (numLeds * 4);
  
  //dataByte = dataBytes; //Points to the array of transposed bytes
  
  int bits = 24;
  REG_PIOD_ODSR = 0x00;
  int times = bits*numLeds;
  const int period = 43;//PERIOD_800;
  const int time0 = 1; // 0 value = 1-13
  const int time1 = 19; // 1 value = 18-31
  const int pinMask = 0xFF;
  
  if(!poked) noInterrupts(); //Added to be able to listen to bluetooth messages
  do{
		while(*timeValue < period);
		REG_PIOD_ODSR = 0xFF; //-- All ON
		*timeReset = TC_CCR_CLKEN | TC_CCR_SWTRG; //this approach saves lots of cycles (5) vs. calling the subroutine
		while(*timeValue < time0); // 2
			//Set each pin level corresponding to the bit value
			
			
			
			
			while(*timeValue < time1);

		REG_PIOD_ODSR = 0x00; // 4 -- All OFF
	}while(times--);
	interrupts();
	
	while(*timeValue < period);
	wsDelay = micros();
	stopTimer();
	
}


/*	Clear Leds
 * 	This function clear all leds.
 * 	It uses a pointer to int(32 bits) to reduce the loops by four
 */
void Due_Leds::clear(){
  int i,*p;
  p=(int *)leds;
  for(i=numLeds*strips;i!=0;i--)
  {
    p[numLeds*strips-i] = 0;
  }
}


//Images---------------------------------------------------------------
/* 	displayImage
 * 	This function displays an image at the position specified 
 */
void Due_Leds::showImage(image img, int16_t xOffset, int16_t yOffset, uint8_t mode){
  uint8_t x=0;
  uint8_t y=0;
  uint8_t brightness = img.brightness;
  struct rgbColor pixel;

  for(int16_t i=0;i<img.width*img.height;i++)
  {
    x =  i - (i/img.width)*img.width;
    y = (i/img.width);
    pixel = img.image[i];
    if(mode == 0) setPixel(x+xOffset,y+yOffset,{pixel.r*brightness/255, pixel.g*brightness/255, pixel.b*brightness/255});
    if(mode == 1) setPixel(x+xOffset,y+yOffset,{pixel.r*brightness/255, pixel.g*brightness/255, pixel.b*brightness/255},1);
  }   
}

void Due_Leds::moveImageLeft(image *img, uint8_t inc){
  img->x = (img->x)-inc;
  if(img->x < 0) img->x = cols-1;
}

void Due_Leds::moveImageRight(image *img, uint8_t inc){
  img->x = (img->x)+inc;
  if(img->x >= cols) img->x = 0;
}

void Due_Leds::moveImageDown(image *img, uint8_t inc){
  img->y = (img->y)+inc;
  if(img->y >= rows) img->y = 0;
}

void Due_Leds::moveImageUp(image *img, uint8_t inc){
  img->y = (img->y)-inc;
  if(img->y < 0) img->y = rows-1;
}
//---------------------------------------------------------------------


// Text---------------------------------------------------------------
// Draw a character
// Characters in glcdfont.c are 5x7 pixels (or 6x8?)
/*void Due_Leds::drawChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t color, uint16_t bg, uint8_t size) {

  if((x >= _width)            || // Clip right
     (y >= _height)           || // Clip bottom
     ((x + 6 * size - 1) < 0) || // Clip left
     ((y + 8 * size - 1) < 0))   // Clip top
    return;

  for (int8_t i=0; i<6; i++ ) {
    uint8_t line;
    
    if (i == 5) 
      line = 0x0;
    else 
      line = pgm_read_byte(font+(c*5)+i); // get character font data from glcdfont.c
    
    for (int8_t j = 0; j<8; j++) {
      if (line & 0x1) {
				
        if (size == 1) // default size
          drawPixel(x+i, y+j, color);
        else {  // big size
          fillRect(x+(i*size), y+(j*size), size, size, color);
        } 
        
      } else if (bg != color) {
				
        if (size == 1) // default size
          drawPixel(x+i, y+j, bg);
        else {  // big size
          fillRect(x+i*size, y+j*size, size, size, bg);
        }
        
      }
      line >>= 1;
    }
  }
}
*/

//--------------------------------------------------------------------

/* 	setLed
 * 	This function sets the color of a Led in the Leds array
 * 	Location of a led can be found by numLeds * strip# + position of led
 */
void Due_Leds::setLed(uint32_t n, struct rgbColor rgb){
	uint32_t p = n*4;
	leds[p] = rgb.g;
	leds[p+1] = rgb.r;
	leds[p+2] = rgb.b;
}

/* 	setPixel
 * 	This function sets the color of a Led in the matrix
 */
void Due_Leds::setPixel(int16_t x, int16_t y, struct rgbColor rgb, uint8_t mode){
	if(x >= cols) x=x-cols; 	//Wrap around horizontal in Fwd
	if(x < 0) x=cols-(-1*x);	//Wrap around horizontal in Rev
	if(y>=rows) y=y-rows; 		//Wrap around vertical in Fwd
	if(y<0) y=rows-(-1*y);		//Wrap around vertical in Rev

	rgb.r = rgb.r*brightness/255;
	rgb.g = rgb.g*brightness/255;
	rgb.b = rgb.b*brightness/255;		

	uint32_t p = getLedPos(x,y);
	rgb.r = __LSR(__RBIT(pgm_read_byte(&gammaCorr[rgb.r])),24);//__ROR(__RBIT(rgb.r),24);
	rgb.g = __LSR(__RBIT(pgm_read_byte(&gammaCorr[rgb.g])),24);//__ROR(__RBIT(rgb.g),24);//(__RBIT(rgb.g)>>24);
	rgb.b = __LSR(__RBIT(pgm_read_byte(&gammaCorr[rgb.b])),24);//__ROR(__RBIT(rgb.g),24);//(__RBIT(rgb.b)>>24);
		
	if(mode == 0){ // Mode 0 - Write blacks
		leds[p] = rgb.g;
		leds[p+1] = rgb.r;
		leds[p+2] = rgb.b;
	}
	
	if(mode == 1){ // Mode 1 - Do not write blacks (transparent bkg)
		if((rgb.r > 0) || (rgb.g > 0) || (rgb.b > 0)){
			leds[p] = rgb.g;
			leds[p+1] = rgb.r;
			leds[p+2] = rgb.b;
		}
	}
}

/* 	movePixelLeft
 * 	This function moves a pixel to the left position
 *  and wraps around the display
 *  Returns the new position of the Pixel
 */
uint32_t Due_Leds::movePixelLeft(uint8_t x, uint8_t y, uint8_t increment){
	int ledPos = copyPixelLeft(x,y,increment);
	setPixel(x,y,{0,0,0});
	return ledPos;
}

/* 	copyPixelRight
 * 	This function copies a pixel to the right position
 *  and wraps around the display
 */
uint32_t Due_Leds::copyPixelRight(uint8_t x, uint8_t y, uint8_t increment){
	if(x >= cols-increment){
		setPixel(increment-(cols-x),y,getPixelColor(x,y));
		return getLedPos(increment-(cols-x),y);
	}
	setPixel(x+increment,y,getPixelColor(x,y));
	return getLedPos(x+increment,y);
}

/* 	copyPixelLeft
 * 	This function copies a pixel to the left position
 *  and wraps around the display
 */
uint32_t Due_Leds::copyPixelLeft(uint8_t x, uint8_t y, uint8_t decrement){
	if(x <= decrement-2){
		setPixel(x+cols-decrement,y,getPixelColor(x,y));
		return getLedPos(x+cols-decrement,y);
	}
	setPixel(x+1-decrement-1,y,getPixelColor(x,y));
	return getLedPos(x+1-decrement-1,y);
}

/* 	fillLeds
 * 	This function sets the color of a group of continuous leds
 * 	Location of a led can be found by numLeds * strip# + position of led
 */
void Due_Leds::fillLeds(uint32_t startLed, uint32_t endLed, struct rgbColor rgb){
	int i,*p;
  p=(int *)leds;
  for(i=endLed;i!=startLed;i--)
  {
    p[endLed-i + startLed] = getColor32(rgb.r, rgb.g, rgb.b);
  }
}

/*	getColor32
 * 	Converts a component based color: r, g, b into a
 * 	32 bits integer equivalent
 */
uint32_t Due_Leds::getColor32(uint8_t r, uint8_t g, uint8_t b){
	uint32_t blue,red,green;
	blue = b;
	red = r;
	green = g;
	return  (blue<<16) + (red<<8) + (green);
	
}

/*	getPixelColor
 * 	returns a struct rgbColor value from the Pixel data
 *  of the nth led
 */
struct rgbColor Due_Leds::getPixelColor(uint8_t x, uint8_t y){
	struct rgbColor color;
	int ledPos = getLedPos(x,y);
	color.g = leds[ledPos];
	color.r = leds[ledPos+1];
	color.b = leds[ledPos+2];
	return  color;
}

/*	getPixelColorN
 * 	returns a struct rgbColor value from the Pixel data
 *  of the nth led
 */
struct rgbColor Due_Leds::getPixelColorN(int n){
	struct rgbColor color;
	color.g = leds[n*4];
	color.r = leds[n*4+1];
	color.b = leds[n*4+2];
	return  color;
}


// Private Methods
//=====================================================================

/*	getMatrixNumber
 * 	This function returns the corresponding matrix number of a led byte 0
 *  position in the leds array
 */
uint8_t Due_Leds::getMatrixNumber(uint32_t ledNumber){
	return ledNumber/numLeds;
}

/*	getRowNumber
 * 	This function returns the corresponding row number of a led byte 0
 *  position in the leds array
 */
uint8_t Due_Leds::getRowNumber(uint32_t ledNumber){
	uint16_t nPos = ledNumber/4;
	return nPos/cols;//(ledNumber-getMatrixNumber(ledNumber)*numLeds)/cols;
}

/*	getColNumber
 * 	This function returns the corresponding col number of a led byte 0
 *  position in the leds array
 */
uint8_t Due_Leds::getColNumber(uint32_t ledNumber){
	uint16_t nPos = ledNumber/4;
	uint8_t col = (nPos/cols);
	col = nPos-col*cols;	
	return col;//ledNumber-(getMatrixNumber(ledNumber)*numLeds)-(getRowNumber(ledNumber)*cols);
}

/*	getLedPos
 * 	This function calculates the led byte 0
 *  position in the leds array  
 * 	based on its absolute row,col coordinates
 */
uint32_t Due_Leds::getLedPos(uint8_t x, uint8_t y){
	//if(strips > 1) 
	return (y*cols+x)*4;
}

/*	allocateMemory
 * 
 *  Allocated 4 bytes per led for each strip.
 * 	This way the transposition becomes easier and faster
 * 	than using 3 bytes. 
 */
int Due_Leds::allocateMemory(){
	int i,j,result,numBytes;
	result = 1;
	numBytes = numLeds*4*8; // 4 bytes per leds in matrix x 8 matrices
	
	//Reserve 960x4x8 = 30,720 bytes
	if(leds = (uint8_t *)malloc(numBytes)){ 
		memset(leds,0,numBytes);
	}else{
		return 0;
	}
	
	//Reserve 960x(24+8 not used) = 30,720 bytes
  if(dataBytes = (uint8_t *)malloc(numBytes)){ 
		memset(dataBytes,0,numBytes);
	}else{
		return 0;
	}
}

/*	rotateRight
 *  This function is for testing the __ROR implementation
 */
int Due_Leds::rotateRight(int v, int s){
	return __ROR(v,s);
}

/*	startTimer
 * 
 */
void Due_Leds::startTimer(){
	TC_Start(tC, channel);
}

/*	stopTimer
 * 
 */
void Due_Leds::stopTimer(){
	TC_Stop(tC,channel);
}

/* Transpose the matrices into the dataBytes array
 * 
 *	 
 */
void Due_Leds::transposeValues(uint8_t *d){
	int b,m,n;
	uint8_t *outBytes;
	int *s0,*s1,*s2,*s3,*s4,*s5,*s6,*s7;
	int db0,db1,db2,db3,db4,db5,db6,db7;

	outBytes = d;
	n = numLeds;	
	s0 = (int *)leds;
	s1 = s0 + n;
	s2 = s1 + n;
	s3 = s2 + n;
	s4 = s3 + n;
	s5 = s4 + n;
	s6 = s5 + n;
	s7 = s6 + n;

	//Optimizations to perform:
	//	1) Count to zero in the for loop - DONE
	//	2) Use only one pointer for the strip and calculate the value
	//		of the next pointer by summing 4 x numLeds to the address
	//		of the base pointer - PENDING ---? 
	//	3) Unroll the 24 loops - DONE Using spreadsheet :D
	//	4) Calculate the value of the bits to shift in the ROR instruction -DONE
	
	do{
		/*
		for(b=24;b!=0;b--){ //Optimize to for(b=24;b!=0;b--) 32+(24-b)-s = 56-b-s
			m = 1 << (24-b);
			*outBytes = __ROR((*s0 & m),56-b-0); //strip 0 data		
			*outBytes = *outBytes + __ROR((*s1 & m),56-b-1); //strip 1 data
			*outBytes = *outBytes + __ROR((*s2 & m),56-b-2); //strip 2 data
			*outBytes = *outBytes + __ROR((*s3 & m),56-b-3); //strip 3 data
			*outBytes = *outBytes + __ROR((*s4 & m),56-b-4); //strip 4 data
			*outBytes = *outBytes + __ROR((*s5 & m),56-b-5); //strip 5 data
			*outBytes = *outBytes + __ROR((*s6 & m),56-b-6); //strip 6 data
			*outBytes = *outBytes + __ROR((*s7 & m),56-b-7); //strip 7 data
			outBytes++;
		}		
		*/
		db0 = *s0;
		db1 = *s1;
		db2 = *s2;
		db3 = *s3;
		db4 = *s4;
		db5 = *s5;
		db6 = *s6;
		db7 = *s7;
		
		//b=0
		m = 1;
		*outBytes = __ROR((db0 & m),32);
		*outBytes = *outBytes + __ROR((db1 & m),31);
		*outBytes = *outBytes + __ROR((db2 & m),30);
		*outBytes = *outBytes + __ROR((db3 & m),29);
		*outBytes = *outBytes + __ROR((db4 & m),28);
		*outBytes = *outBytes + __ROR((db5 & m),27);
		*outBytes = *outBytes + __ROR((db6 & m),26);
		*outBytes = *outBytes + __ROR((db7 & m),25);
		outBytes++;

		//b=1
		m = 2;
		*outBytes = __ROR((db0 & m),33);
		*outBytes = *outBytes + __ROR((db1 & m),32);
		*outBytes = *outBytes + __ROR((db2 & m),31);
		*outBytes = *outBytes + __ROR((db3 & m),30);
		*outBytes = *outBytes + __ROR((db4 & m),29);
		*outBytes = *outBytes + __ROR((db5 & m),28);
		*outBytes = *outBytes + __ROR((db6 & m),27);
		*outBytes = *outBytes + __ROR((db7 & m),26);
		outBytes++;

		//b=2
		m = 4;
		*outBytes = __ROR((db0 & m),34);
		*outBytes = *outBytes + __ROR((db1 & m),33);
		*outBytes = *outBytes + __ROR((db2 & m),32);
		*outBytes = *outBytes + __ROR((db3 & m),31);
		*outBytes = *outBytes + __ROR((db4 & m),30);
		*outBytes = *outBytes + __ROR((db5 & m),29);
		*outBytes = *outBytes + __ROR((db6 & m),28);
		*outBytes = *outBytes + __ROR((db7 & m),27);
		outBytes++;

		//b=3
		m = 8;
		*outBytes = __ROR((db0 & m),35);
		*outBytes = *outBytes + __ROR((db1 & m),34);
		*outBytes = *outBytes + __ROR((db2 & m),33);
		*outBytes = *outBytes + __ROR((db3 & m),32);
		*outBytes = *outBytes + __ROR((db4 & m),31);
		*outBytes = *outBytes + __ROR((db5 & m),30);
		*outBytes = *outBytes + __ROR((db6 & m),29);
		*outBytes = *outBytes + __ROR((db7 & m),28);
		outBytes++;

		//b=4
		m = 16;
		*outBytes = __ROR((db0 & m),36);
		*outBytes = *outBytes + __ROR((db1 & m),35);
		*outBytes = *outBytes + __ROR((db2 & m),34);
		*outBytes = *outBytes + __ROR((db3 & m),33);
		*outBytes = *outBytes + __ROR((db4 & m),32);
		*outBytes = *outBytes + __ROR((db5 & m),31);
		*outBytes = *outBytes + __ROR((db6 & m),30);
		*outBytes = *outBytes + __ROR((db7 & m),29);
		outBytes++;

		//b=5
		m = 32;
		*outBytes = __ROR((db0 & m),37);
		*outBytes = *outBytes + __ROR((db1 & m),36);
		*outBytes = *outBytes + __ROR((db2 & m),35);
		*outBytes = *outBytes + __ROR((db3 & m),34);
		*outBytes = *outBytes + __ROR((db4 & m),33);
		*outBytes = *outBytes + __ROR((db5 & m),32);
		*outBytes = *outBytes + __ROR((db6 & m),31);
		*outBytes = *outBytes + __ROR((db7 & m),30);
		outBytes++;

		//b=6
		m = 64;
		*outBytes = __ROR((db0 & m),38);
		*outBytes = *outBytes + __ROR((db1 & m),37);
		*outBytes = *outBytes + __ROR((db2 & m),36);
		*outBytes = *outBytes + __ROR((db3 & m),35);
		*outBytes = *outBytes + __ROR((db4 & m),34);
		*outBytes = *outBytes + __ROR((db5 & m),33);
		*outBytes = *outBytes + __ROR((db6 & m),32);
		*outBytes = *outBytes + __ROR((db7 & m),31);
		outBytes++;

		//b=7
		m = 128;
		*outBytes = __ROR((db0 & m),39);
		*outBytes = *outBytes + __ROR((db1 & m),38);
		*outBytes = *outBytes + __ROR((db2 & m),37);
		*outBytes = *outBytes + __ROR((db3 & m),36);
		*outBytes = *outBytes + __ROR((db4 & m),35);
		*outBytes = *outBytes + __ROR((db5 & m),34);
		*outBytes = *outBytes + __ROR((db6 & m),33);
		*outBytes = *outBytes + __ROR((db7 & m),32);
		outBytes++;

		//b=8
		m = 256;
		*outBytes = __ROR((db0 & m),40);
		*outBytes = *outBytes + __ROR((db1 & m),39);
		*outBytes = *outBytes + __ROR((db2 & m),38);
		*outBytes = *outBytes + __ROR((db3 & m),37);
		*outBytes = *outBytes + __ROR((db4 & m),36);
		*outBytes = *outBytes + __ROR((db5 & m),35);
		*outBytes = *outBytes + __ROR((db6 & m),34);
		*outBytes = *outBytes + __ROR((db7 & m),33);
		outBytes++;

		//b=9
		m = 512;
		*outBytes = __ROR((db0 & m),41);
		*outBytes = *outBytes + __ROR((db1 & m),40);
		*outBytes = *outBytes + __ROR((db2 & m),39);
		*outBytes = *outBytes + __ROR((db3 & m),38);
		*outBytes = *outBytes + __ROR((db4 & m),37);
		*outBytes = *outBytes + __ROR((db5 & m),36);
		*outBytes = *outBytes + __ROR((db6 & m),35);
		*outBytes = *outBytes + __ROR((db7 & m),34);
		outBytes++;

		//b=10
		m = 1024;
		*outBytes = __ROR((db0 & m),42);
		*outBytes = *outBytes + __ROR((db1 & m),41);
		*outBytes = *outBytes + __ROR((db2 & m),40);
		*outBytes = *outBytes + __ROR((db3 & m),39);
		*outBytes = *outBytes + __ROR((db4 & m),38);
		*outBytes = *outBytes + __ROR((db5 & m),37);
		*outBytes = *outBytes + __ROR((db6 & m),36);
		*outBytes = *outBytes + __ROR((db7 & m),35);
		outBytes++;

		//b=11
		m = 2048;
		*outBytes = __ROR((db0 & m),43);
		*outBytes = *outBytes + __ROR((db1 & m),42);
		*outBytes = *outBytes + __ROR((db2 & m),41);
		*outBytes = *outBytes + __ROR((db3 & m),40);
		*outBytes = *outBytes + __ROR((db4 & m),39);
		*outBytes = *outBytes + __ROR((db5 & m),38);
		*outBytes = *outBytes + __ROR((db6 & m),37);
		*outBytes = *outBytes + __ROR((db7 & m),36);
		outBytes++;

		//b=12
		m = 4096;
		*outBytes = __ROR((db0 & m),44);
		*outBytes = *outBytes + __ROR((db1 & m),43);
		*outBytes = *outBytes + __ROR((db2 & m),42);
		*outBytes = *outBytes + __ROR((db3 & m),41);
		*outBytes = *outBytes + __ROR((db4 & m),40);
		*outBytes = *outBytes + __ROR((db5 & m),39);
		*outBytes = *outBytes + __ROR((db6 & m),38);
		*outBytes = *outBytes + __ROR((db7 & m),37);
		outBytes++;

		//b=13
		m = 8192;
		*outBytes = __ROR((db0 & m),45);
		*outBytes = *outBytes + __ROR((db1 & m),44);
		*outBytes = *outBytes + __ROR((db2 & m),43);
		*outBytes = *outBytes + __ROR((db3 & m),42);
		*outBytes = *outBytes + __ROR((db4 & m),41);
		*outBytes = *outBytes + __ROR((db5 & m),40);
		*outBytes = *outBytes + __ROR((db6 & m),39);
		*outBytes = *outBytes + __ROR((db7 & m),38);
		outBytes++;

		//b=14
		m = 16384;
		*outBytes = __ROR((db0 & m),46);
		*outBytes = *outBytes + __ROR((db1 & m),45);
		*outBytes = *outBytes + __ROR((db2 & m),44);
		*outBytes = *outBytes + __ROR((db3 & m),43);
		*outBytes = *outBytes + __ROR((db4 & m),42);
		*outBytes = *outBytes + __ROR((db5 & m),41);
		*outBytes = *outBytes + __ROR((db6 & m),40);
		*outBytes = *outBytes + __ROR((db7 & m),39);
		outBytes++;

		//b=15
		m = 32768;
		*outBytes = __ROR((db0 & m),47);
		*outBytes = *outBytes + __ROR((db1 & m),46);
		*outBytes = *outBytes + __ROR((db2 & m),45);
		*outBytes = *outBytes + __ROR((db3 & m),44);
		*outBytes = *outBytes + __ROR((db4 & m),43);
		*outBytes = *outBytes + __ROR((db5 & m),42);
		*outBytes = *outBytes + __ROR((db6 & m),41);
		*outBytes = *outBytes + __ROR((db7 & m),40);
		outBytes++;

		//b=16
		m = 65536;
		*outBytes = __ROR((db0 & m),48);
		*outBytes = *outBytes + __ROR((db1 & m),47);
		*outBytes = *outBytes + __ROR((db2 & m),46);
		*outBytes = *outBytes + __ROR((db3 & m),45);
		*outBytes = *outBytes + __ROR((db4 & m),44);
		*outBytes = *outBytes + __ROR((db5 & m),43);
		*outBytes = *outBytes + __ROR((db6 & m),42);
		*outBytes = *outBytes + __ROR((db7 & m),41);
		outBytes++;

		//b=17
		m = 131072;
		*outBytes = __ROR((db0 & m),49);
		*outBytes = *outBytes + __ROR((db1 & m),48);
		*outBytes = *outBytes + __ROR((db2 & m),47);
		*outBytes = *outBytes + __ROR((db3 & m),46);
		*outBytes = *outBytes + __ROR((db4 & m),45);
		*outBytes = *outBytes + __ROR((db5 & m),44);
		*outBytes = *outBytes + __ROR((db6 & m),43);
		*outBytes = *outBytes + __ROR((db7 & m),42);
		outBytes++;

		//b=18
		m = 262144;
		*outBytes = __ROR((db0 & m),50);
		*outBytes = *outBytes + __ROR((db1 & m),49);
		*outBytes = *outBytes + __ROR((db2 & m),48);
		*outBytes = *outBytes + __ROR((db3 & m),47);
		*outBytes = *outBytes + __ROR((db4 & m),46);
		*outBytes = *outBytes + __ROR((db5 & m),45);
		*outBytes = *outBytes + __ROR((db6 & m),44);
		*outBytes = *outBytes + __ROR((db7 & m),43);
		outBytes++;

		//b=19
		m = 524288;
		*outBytes = __ROR((db0 & m),51);
		*outBytes = *outBytes + __ROR((db1 & m),50);
		*outBytes = *outBytes + __ROR((db2 & m),49);
		*outBytes = *outBytes + __ROR((db3 & m),48);
		*outBytes = *outBytes + __ROR((db4 & m),47);
		*outBytes = *outBytes + __ROR((db5 & m),46);
		*outBytes = *outBytes + __ROR((db6 & m),45);
		*outBytes = *outBytes + __ROR((db7 & m),44);
		outBytes++;

		//b=20
		m = 1048576;
		*outBytes = __ROR((db0 & m),52);
		*outBytes = *outBytes + __ROR((db1 & m),51);
		*outBytes = *outBytes + __ROR((db2 & m),50);
		*outBytes = *outBytes + __ROR((db3 & m),49);
		*outBytes = *outBytes + __ROR((db4 & m),48);
		*outBytes = *outBytes + __ROR((db5 & m),47);
		*outBytes = *outBytes + __ROR((db6 & m),46);
		*outBytes = *outBytes + __ROR((db7 & m),45);
		outBytes++;

		//b=21
		m = 2097152;
		*outBytes = __ROR((db0 & m),53);
		*outBytes = *outBytes + __ROR((db1 & m),52);
		*outBytes = *outBytes + __ROR((db2 & m),51);
		*outBytes = *outBytes + __ROR((db3 & m),50);
		*outBytes = *outBytes + __ROR((db4 & m),49);
		*outBytes = *outBytes + __ROR((db5 & m),48);
		*outBytes = *outBytes + __ROR((db6 & m),47);
		*outBytes = *outBytes + __ROR((db7 & m),46);
		outBytes++;

		//b=22
		m = 4194304;
		*outBytes = __ROR((db0 & m),54);
		*outBytes = *outBytes + __ROR((db1 & m),53);
		*outBytes = *outBytes + __ROR((db2 & m),52);
		*outBytes = *outBytes + __ROR((db3 & m),51);
		*outBytes = *outBytes + __ROR((db4 & m),50);
		*outBytes = *outBytes + __ROR((db5 & m),49);
		*outBytes = *outBytes + __ROR((db6 & m),48);
		*outBytes = *outBytes + __ROR((db7 & m),47);
		outBytes++;

		//b=23
		m = 8388608;
		*outBytes = __ROR((db0 & m),55);
		*outBytes = *outBytes + __ROR((db1 & m),54);
		*outBytes = *outBytes + __ROR((db2 & m),53);
		*outBytes = *outBytes + __ROR((db3 & m),52);
		*outBytes = *outBytes + __ROR((db4 & m),51);
		*outBytes = *outBytes + __ROR((db5 & m),50);
		*outBytes = *outBytes + __ROR((db6 & m),49);
		*outBytes = *outBytes + __ROR((db7 & m),48);
		outBytes++;

		
		s0 += 1;
		s1 += 1;
		s2 += 1;
		s3 += 1;
		s4 += 1;
		s5 += 1;
		s6 += 1;
		s7 += 1;
	}while(n--);
	

}
