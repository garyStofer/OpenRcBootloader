/*
 * Author - Mike Blandford
 *
 * Based on er9x by Erez Raviv <erezraviv@gmail.com>
 *
 * Based on th9x -> http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

// This version for ARM based ERSKY9X board
/*   For editing enable these defines so that the IDE can blank out code
#define PCBSKY
#define REVB
#define REVX
#define JR9303

// one of these three must be defined otherwise the old original ersky LCD is used
//#define ERC24064_1		// the 240 by 64 erc 
//#define ERC12864_2	 // the old 128x64 ERC 
//#define ERC12864_14	// The currently available 128x64 one -- as of 5/2024
*/
	

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef PCBSKY
#include "AT91SAM3S4.h"
#include "radio.h"
//#include "myeeprom.h"
#endif

#ifdef PCBX9D
 #include "radio.h"
#endif

#ifdef PCB9XT
 #include "radio.h"
 #include "mega64.h"
#endif

#include "lcd.h"
#include "drivers.h"
#include "logicio.h"

#include "font.lbm"
#define font_5x8_x20_x7f (font)

#ifndef PCBTARANIS
#include "font_dblsize.lbm"
#define font_10x16_x20_x7f (font_dblsize)
#endif

// Local data
uint8_t OptrexDisplay = 0 ;
uint8_t Lcd_lastPos ;
uint8_t DisplayBuf[DISPLAY_W*DISPLAY_H/8] ;
#define DISPLAY_END (DisplayBuf+sizeof(DisplayBuf))

#ifdef PCBX9D
#if defined(PCBX7) || defined (PCBXLITE) || defined(PCBX9LITE)
#define X9D_OFFSET		0
#else
#define X9D_OFFSET		11
#endif
#define DISPLAY_START (DisplayBuf + X9D_OFFSET)
#else
#define DISPLAY_START (DisplayBuf + 0)
#endif 



/// invers: 0 no 1=yes 2=blink
void lcd_putc(uint8_t x,uint8_t y,const char c )
{
  lcd_putcAtt(x,y,c,0);
}

uint8_t lcd_putcAtt(uint8_t x,uint8_t y,const char c,uint8_t mode)
{
	register int32_t i ;
	register uint8_t *p    = &DISPLAY_START[ y / 8 * DISPLAY_W + x ];
    //uint8_t *pmax = &displayBuf[ DISPLAY_H/8 * DISPLAY_W ];
	if ( c < 22 )		// Move to specific x position (c)*FW
	{
		x = c*FW ;
  	if(mode&DBLSIZE)
		{
			x += x ;
		}
		return x ;
	}
	x += FW ;
  register uint8_t    *q ;
	if( c < 0xC0 )
	{
		q = (uint8_t *) &font_5x8_x20_x7f[(c-0x20)*5] ;
	}
	else
	{
			q = (uint8_t *) &font_5x8_x20_x7f[0] ;
	}
  register bool   inv = (mode & INVERS) ? true : false ;
  
#ifndef PCBTARANIS
	if(mode&DBLSIZE)
  {
		if ( (c!=0x2E)) x+=FW; //check for decimal point
	/* each letter consists of ten top bytes followed by
	 * five bottom by ten bottom bytes (20 bytes per 
	 * char) */
	  unsigned char c_mapped ;
		if( c < 0xC0 )
		{
			if ( c == 'B' )
			{
				c_mapped = 1 ;
			}
			else if ( c == 'S' )
			{
				c_mapped = 2 ;
			}
			else if ( c == 'U' )
			{
				c_mapped = 3 ;
			}
			else
			{
				c_mapped = 0 ;
			}
			q = (uint8_t *) &font_10x16_x20_x7f[(c_mapped)*20] ;
//  	  q = (uint8_t *) &font_10x16_x20_x7f[(c-0x20)*10 + ((c-0x20)/16)*160];
		}
		else
		{
				q = (uint8_t *) &font_10x16_x20_x7f[0] ;
		}
    for( i=5 ; i>=0 ; i-- )
		{
			uint8_t b1 ;
			uint8_t b3 ;
			uint8_t b2 ;
			uint8_t b4 ;

			if( c < 0xC0 )
			{
	    	/*top byte*/
      	b1 = i>0 ? *q : 0;
	    	/*bottom byte*/
      	b3 = i>0 ? *(q+10) : 0;
	    	/*top byte*/
      	b2 = i>0 ? *(++q) : 0;
	    	/*bottom byte*/
      	b4 = i>0 ? *(q+10) : 0;
      	q++;
			}
			else
			{
	    	/*top byte*/
      	b1 = i>0 ? *q++ : 0 ;
	    	/*bottom byte*/
      	b3 = i>0 ? *q++ : 0 ;
	    	/*top byte*/
      	b2 = i>0 ? *q++ : 0 ;
	    	/*bottom byte*/
      	b4 = i>0 ? *q++ : 0 ;
			}
      if(inv)
			{
		    b1=~b1;
		    b2=~b2;
		    b3=~b3;
		    b4=~b4;
      }

      if(&p[DISPLAY_W+1] < DISPLAY_END)
			{
        p[0]=b1;
        p[1]=b2;
        p[DISPLAY_W] = b3;
        p[DISPLAY_W+1] = b4;
        p+=2;
      }
    }
  }
  else
#endif
  {
    uint8_t condense=0;

    if (mode & CONDENSED)
		{
      *p++ = inv ? ~0 : 0;
      condense=1;
    	x += FWNUM-FW ;
		}

    for( i=5 ; i!=0 ; i-- )
		{
      uint8_t b = *q++ ;
  	  if (condense && i==4)
			{
        /*condense the letter by skipping column 4 */
        continue;
      }
      if(p<DISPLAY_END) *p++ = inv ? ~b : b;
    }
    if(p<DISPLAY_END) *p++ = inv ? ~0 : 0;
  }
	return x ;
}

void lcd_putsnAtt(uint8_t x,uint8_t y, const char * s,uint8_t len,uint8_t mode)
{
	register char c ;
//	register uint8_t size ;
//	size = mode & DBLSIZE ;
  while(len!=0) {
    c = *s++ ;
		if ( c == 0 )
		{
			break ;			
		}

    x = lcd_putcAtt(x,y,c,mode);
    len--;
  }
}

void lcd_putsn_P(uint8_t x,uint8_t y,const char * s,uint8_t len)
{
  lcd_putsnAtt( x,y,s,len,0);
}

uint8_t lcd_putsAtt( uint8_t x, uint8_t y, const char *s, uint8_t mode )
{
  while(1)
	{
    char c = *s++ ;
    if(!c) break ;
    x = lcd_putcAtt(x,y,c,mode) ;
  }
  return x;
}

void lcd_puts_Pleft( uint8_t y, const char *s )
{
  lcd_putsAtt( 0, y, s, 0);
}

void lcd_puts_P( uint8_t x, uint8_t y, const char *s )
{
  lcd_putsAtt( x, y, s, 0);
}

#ifdef APP
// Puts sub-string from string options
// First byte of string is sub-string length
// idx is index into string (in length units)
// Output length characters
void lcd_putsAttIdx(uint8_t x,uint8_t y,const char * s,uint8_t idx,uint8_t att)
{
	uint8_t length ;
	length = *s++ ;

  lcd_putsnAtt(x,y,s+length*idx,length,att) ;
}
#endif

void lcd_outhex4(uint8_t x,uint8_t y,uint16_t val)
{
  register int i ;
  x+=FWNUM*4;
  for(i=0; i<4; i++)
  {
    x-=FWNUM;
    char c = val & 0xf;
    c = c>9 ? c+'A'-10 : c+'0';
    lcd_putcAtt(x,y,c,c>='A'?CONDENSED:0);
    val>>=4;
  }
}
uint8_t plotType = PLOT_XOR ;

void lcd_write_bits( uint8_t *p, uint8_t mask )
{
  if(p<DISPLAY_END)
	{
		uint8_t temp = *p ;
		if ( plotType != PLOT_XOR )
		{
			temp |= mask ;
		}
		if ( plotType != PLOT_BLACK )
		{
			temp ^= mask ;
		}
		*p = temp ;
	}
}

void lcd_plot( register uint8_t x, register uint8_t y )
{
  register uint8_t *p   = &DISPLAY_START[ y / 8 * DISPLAY_W + x ];
	lcd_write_bits( p, BITMASK(y%8) ) ;
}

void lcd_hlineStip( unsigned char x, unsigned char y, signed char w, uint8_t pat )
{
  if(w<0) {x+=w; w=-w;}
  register uint8_t *p  = &DISPLAY_START[ y / 8 * DISPLAY_W + x ];
  register uint8_t msk = BITMASK(y%8);
  while(w)
	{
    if ( p>=DISPLAY_END)
    {
      break ;			
    }
    if(pat&1)
		{
			lcd_write_bits( p, msk ) ;
      pat = (pat >> 1) | 0x80;
    }
		else
		{
      pat = pat >> 1;
    }
    w--;
    p++;
  }
}


// Reverse video 8 pixels high, w pixels wide
// Vertically on an 8 pixel high boundary
void lcd_char_inverse( uint8_t x, uint8_t y, uint8_t w, uint8_t blink )
{
//	if ( blink && BLINK_ON_PHASE )
//	{
//		return ;
//	}
	uint8_t end = x + w ;
  uint8_t *p = &DISPLAY_START[ y / 8 * DISPLAY_W + x ];

	while ( x < end )
	{
		*p++ ^= 0xFF ;
		x += 1 ;
	}
}

void lcd_hline( uint8_t x, uint8_t y, int8_t w )
{
  lcd_hlineStip(x,y,w,0xff);
}

void lcd_vline( uint8_t x, uint8_t y, int8_t h )
{
  if (h<0) { y+=h; h=-h; }
  register uint8_t *p  = &DISPLAY_START[ y / 8 * DISPLAY_W + x ];

  y &= 0x07 ;
	if ( y )
	{
    uint8_t msk = ~(BITMASK(y)-1) ;
    h -= 8-y ;
    if (h < 0)
      msk -= ~(BITMASK(8+h)-1) ;
		lcd_write_bits( p, msk ) ;
    p += DISPLAY_W ;
	}
    
  while( h >= 8 )
	{
		h -= 8 ;
		lcd_write_bits( p, 0xFF ) ;
    p += DISPLAY_W ;
  }
	if ( h > 0 )
	{
  	lcd_write_bits( p, (BITMASK(h)-1) ) ;
	}
}

#ifdef SIMU
bool lcd_refresh = true;
uint8_t lcd_buf[DISPLAY_W*DISPLAY_H/8];
#endif

#if PCBX9D
#ifndef REV9E
 #ifndef PCBX7
  #ifndef PCBXLITE
   #ifndef PCBX9LITE
const uint8_t arrows[] = {
10,64,80,
0xFF,0xF7,0xF3,0xF9,0x00,0x00,0xF9,0xF3,0xF7,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFC,0xFC,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xCF,0x87,0x03,0x49,0xCF,0xCF,0xCF,0xCF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0x3F,0x3F,0xFF,0xFF,0xFF,0xFF,
0xFF,0xEF,0xCF,0x9F,0x00,0x00,0x9F,0xCF,0xEF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,

0xFF,0xFF,0x3F,0x7F,0xFF,0x3F,0x8F,0xE3,0xF8,0xFF,
0xFF,0xFF,0xFF,0xFE,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xCF,0xCF,0xCF,0xCF,0x49,0x03,0x87,0xCF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0xFF,0xFF,0xFF,
0xFF,0x7F,0x7F,0xFF,0xFF,0xFF,0xFF,0x7F,0x7F,0xFF,
0xFF,0x9E,0x8C,0xC0,0xE1,0xE1,0xC0,0x8C,0x9E,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

} ;
static void lcd_bitmap( uint8_t i_x, uint8_t i_y, uint8_t *bitmap, uint8_t w, uint8_t h )
{
	uint32_t yb ;
	uint32_t x ;
	
  for( yb = 0; yb < h; yb++)
	{
    register uint8_t *p = &DISPLAY_START[ (i_y / 8 + yb) * DISPLAY_W + i_x ];
		if ( i_x > 211-X9D_OFFSET )
		{
			p -= DISPLAY_W ;		
		}
    for(x=0; x < w; x++)
		{
      register uint8_t b = *bitmap++ ;
      *p++ = b ;
    }
  }
}

void lcd_img( uint8_t i_x, uint8_t i_y, uint8_t *imgdat, uint8_t idx )
{
  register uint8_t *q = imgdat ;
  register uint8_t w    = *q++ ;
  register uint32_t hb   = (*q++ +7) / 8 ;
  register uint8_t sze1 = *q++ ;
//	uint32_t yb ;
//	uint32_t x ;

  q += idx * sze1 ;
  
	lcd_bitmap( i_x, i_y, q, w, hb ) ;
}
   #endif // nPCBX9LITE
  #endif // nPCBXLITE
 #endif // nPCBX7
#endif	// nREV9E
#endif

void lcd_clear()
{
  memset( DisplayBuf, 0, sizeof( DisplayBuf) ) ;
#if PCBX9D
#ifndef REV9E
 #ifndef PCBX7
  #ifndef PCBXLITE
   #ifndef PCBX9LITE
	lcd_img( 212-X9D_OFFSET, 0, (uint8_t *) arrows, 0 ) ;
	lcd_img( 212-X9D_OFFSET-10, 0, (uint8_t *) arrows, 1 ) ;
   #endif // nPCBX9LITE
  #endif // nPCBXLITE
 #endif // nPCBX7
#endif	// nREV9E
#endif // PCBX9D
}

// LCD i/o pins
// LCD_RES     PC27
// LCD_CS1     PC26
// LCD_E       PC12
// LCD_RnW     PC13
// LCD_A0      PC15
// LCD_D0      PC0
// LCD_D1      PC7
// LCD_D2      PC6
// LCD_D3      PC5
// LCD_D4      PC4
// LCD_D5      PC3
// LCD_D6      PC2
// LCD_D7      PC1

#define LCD_DATA	0x000000FFL
#define LCD_A0    0x00000080L
#define LCD_RnW   0x00002000L		// bit 13
#define LCD_E     0x00001000L
#define LCD_CS1   0x04000000L		// bit 26
#define LCD_RES   0x08000000L


uint8_t LcdLock ;
uint8_t LcdInputs ;



#ifdef PCBSKY


#if defined ERC12864_14
	#define PAGE_ADDR_OFFSET 0
	#define COL_ADDR_OFFSET 0 
	#define LCD_MAX_X 135
	#define LCD_CLEAR_NEEDED	
	const static uint8_t Lcd_init[] =
	{
		0xe2, // software reset command 
		0xae, //DON = 0: display OFF
		0x60, // display start line:0x40-0x7f erc12864-14 has funny line indexing
		0xa1, //ADC = 1: reverse direction(SEG132->SEG1)
		0xA6, //REV = 0: non-reverse display
		0xA4, //EON = 0: normal display. non-entire
		0xA3, // Select LCD : A2 = 1/9 bias, A3 = 1/7 bias for 1/65 duty
		0xC0, //SHL = 0: normal direction (COM0->COM63) 
		// 0xf8, // Booster ratio set command
		// 0x00, // Booster ratio set to 0, == 2x,3x,4x mode
		0x2F, //Control power circuit operation VC=VR=VF=1 :0x28-0x2f
		0x25, //Select int resistance ratio R2 R1 R0  : 0x20-0x27
		0x81, // Electronic volume mode set command
		0x20, // Electronic volume register set -- Brightness value (0-3f)

		0xAF  //DON = 1: display ON
	} ;
#endif	
#if defined ERC24064_1
/*	
		Display controller has actually 128 horizontal lines, addresses as 16 pages of 8 bits,
    but the attached LCD display has only 64 horizontal lines (8 pages), so they 
		wired the lcd lines to pages 4-12 leaving the first and last 4 pages worth or lines from the controller 
		dangling.  
		This could normally be compensated by setting the "Start Line Register" to shift the output to the 
		scan lines, but the UC1608 controller chip for some reason can only shift up to 64 lines out of the 128.
		Therefore the only way to get the contents displayed in the LCD area is to start writing the data 
	  starting at page 4 instead of 0 , hence the #define PAGE_ADDR_OFFSET below 

		Since only 128 columns of the 240 wide screen is used here the column start adress needs to be shifted by 
		(240-128)/2 to center the display area.

		The X axis is mirrored by default settings and needs to be inverted with the commnad below -- Sense is different 
		from all the other displays 
*/
	#define PAGE_ADDR_OFFSET 4  // for erc24064-1
	#define COL_ADDR_OFFSET 56  // for erc24064-1
	#define LCD_MAX_X 240
	#define LCD_CLEAR_NEEDED
	const static uint8_t Lcd_init[] =
	{
	  0xe2, //Initialize the internal functions -- not really needed using the reset line
		0xae, //DOF
		0x90, // no fixed lines
		0xc0 |0x4, 	// mirror , 4=X, 8=y
		0x40 |0x0,	// scan line start 0 -- but needs page offset of 4 for this device
		0x81,0x24,  //contrast setting, a good starting point -- gets overwritten later
		0xAF,  //DON = 1: display ON
		} ;
#endif

#if defined ERC12864_2
	#define PAGE_ADDR_OFFSET 0
	#define COL_ADDR_OFFSET 0
	#define LCD_MAX_X 135
	const static uint8_t Lcd_init[] =
	{
		0xe2, //Initialize the internal functions
		0xae, //DON = 0: display OFF
		0xa1, //ADC = 1: reverse direction(SEG132->SEG1)
		0xA6, //REV = 0: non-reverse display
		0xA4, //EON = 0: normal display. non-entire
		0xA3, // Select LCD bias=0
		0xC0, //SHL = 0: normal direction (COM1->COM64)
		0x2F, //Control power circuit operation VC=VR=VF=1
		0x27, //Select int resistance ratio R2 R1 R0 =5
		0x81, //Set reference voltage Mode
		0x2D, // 24 SV5 SV4 SV3 SV2 SV1 SV0 = 0x18
		0xAF  //DON = 1: display ON
	} ;
#endif	
#ifndef LCD_MAX_X // presumably the original er9x LCD
#warning "OLD er9x lcd is being used NOT aftermarket ERC type !!!!!"
	#define PAGE_ADDR_OFFSET 0
	#define COL_ADDR_OFFSET 0
	#define LCD_MAX_X 135
	const static uint8_t Lcd_init[] =
	{
		0xe2,		// Reset
		0xae,		// Display off
		0xa1,		// Reverse segment drive
		0xA6,		// Display normal (not inverse)
		0xA4,		// Display normal (not all on)
		0xA2,		// Bias low (A3 for high)
		0xC0,		// Scan Com0->Com63
		0x2F,		// Internal power mode
		0x25,		// Internal Vreg ratio high(ish)
		0x81,	0x22,		// Contrast
	#ifndef REVX
		0xAF		// Display on
	#endif
	} ;	
#endif

void lcd_init()
{
	register Pio *pioptr ;
	uint32_t i ;

 // read the inputs, and lock the LCD lines
	LcdInputs = PIOC->PIO_PDSR << 1 ; // 6 LEFT, 5 RIGHT, 4 DOWN, 3 UP ()
	LcdLock = 1 ;
	pioptr = PIOC ;

#ifdef REVX
	pioptr->PIO_PER = PIO_PC27 | PIO_PC12 | 0xFF ;		// Enable bits 27,26,13,12,7-0
#else
	pioptr->PIO_PER = PIO_PC27 | PIO_PC26 | PIO_PC13 | PIO_PC12 | 0xFF ;		// Enable bits 27,26,13,12,7-0
#endif // REVX


#ifdef REVX
	pioptr->PIO_CODR = LCD_E ;
	pioptr->PIO_CODR = LCD_RnW | LCD_CS1 ;	// No longer needed, used elsewhere
#else
	pioptr->PIO_CODR = LCD_E | LCD_RnW | LCD_CS1 ;
#endif // REVX

	pioptr->PIO_SODR = LCD_RES ;
	pioptr->PIO_OER = PIO_PC27 | PIO_PC26 | PIO_PC13 | PIO_PC12 | 0xFF ;		// Set bits 27,26,13,12,7-0 output
	pioptr->PIO_OWER = 0x000000FFL ;		// Allow write to ls 8 bits in ODSR


	configure_pins( LCD_A0, PIN_ENABLE | PIN_LOW | PIN_OUTPUT | PIN_PORTA | PIN_NO_PULLUP ) ;

	TC0->TC_CHANNEL[0].TC_CCR = 5 ;	// Enable clock and trigger it (may only need trigger)
	pioptr->PIO_CODR = LCD_RES ;		// Reset LCD

	while ( TC0->TC_CHANNEL[0].TC_CV < 200 )		// >10 uS, Value depends on MCK/8
		; // Wait


	TC0->TC_CHANNEL[0].TC_CCR = 5 ;	// Enable clock and trigger it (may only need trigger)
	pioptr->PIO_SODR = LCD_RES ;		// Remove LCD reset


	while ( TC0->TC_CHANNEL[0].TC_CV < 1500 )	//  1500 uS, Value depends on MCK/8
	{
		wdt_reset(); // Wait
	}

	// send the required initialization bytes
	for ( i = 0 ; i < sizeof(Lcd_init) ; i++ )
	  lcdSendCtl( Lcd_init[i] ) ;
	

// Clears the LCD display  --- all the postions -- Needed if the LCD does not fully clear it's memory
// upon Harware reset

#ifdef  LCD_CLEAR_NEEDED	
{ 
	uint8_t col_offset ;
	for ( uint8_t page_addr=0;page_addr <8; page_addr++)
	{

		col_offset = 0;

		lcdSendCtl( 0 |  (col_offset&0xf) ) ;	// address LCD hw column0
		lcdSendCtl(0x10 | ((col_offset & 0xf0) >>4 )); // send column-address-high commnad -- data part always 0
		lcdSendCtl( 0xB0 | (page_addr+PAGE_ADDR_OFFSET )); // send page addr command with ored in pageaddr and page addr offset
		
		PIOA->PIO_SODR = LCD_A0 ;			// Data
		for( uint8_t col_addr=0; col_addr< LCD_MAX_X; col_addr++ )	// fill all data memory of the LCD with 0 -- there are more than 128 col
		{
			
			pioptr->PIO_ODSR = 0;	// just clear it
			pioptr->PIO_SODR = LCD_E ;	// Start E pulse

			// Need a delay here (250nS)
			TC0->TC_CHANNEL[0].TC_CCR = 5 ;	

			while ( TC0->TC_CHANNEL[0].TC_CV < 500 )		// Value depends on MCK/8
			; 		// Wait for the lcd having received the byte
		
			pioptr->PIO_CODR = LCD_E ;			// End E pulse

		}
		PIOA->PIO_CODR = LCD_A0 ;			// commands 


	}
}
#endif


	pioptr->PIO_ODR = 0x0000003AL ;		// Set bits 1, 3, 4, 5 input
	pioptr->PIO_PUER = 0x0000003AL ;		// Set bits 1, 3, 4, 5 with pullups
	pioptr->PIO_ODSR = 0 ;							// Drive D0 low

	LcdLock = 0 ;
}


void lcdSetRefVolt(uint8_t val)
{
#ifndef SIMU
	register Pio *pioptr ;
	pioptr = PIOC ;

// read the inputs, and lock the LCD lines
	LcdInputs = PIOC->PIO_PDSR << 1 ; // 6 LEFT, 5 RIGHT, 4 DOWN, 3 UP ()
	LcdLock = 1 ;

	pioptr->PIO_OER = 0x0C00B0FFL ;		// Set bits 27,26,15,13,12,7-0 output

  lcdSendCtl(0x81);
	if ( val == 0 )
	{
		val = 0x22 ;		
	}
  lcdSendCtl(val);
	
	pioptr->PIO_ODR = 0x0000003AL ;		// Set bits 1, 3, 4, 5 input
	pioptr->PIO_PUER = 0x0000003AL ;		// Set bits 1, 3, 4, 5 with pullups
	pioptr->PIO_ODSR = 0 ;							// Drive D0 low
#endif // SIMU
	LcdLock = 0 ;
}



void lcdSendCtl(uint8_t val)
{
	register Pio *pioptr ;
	
	pioptr = PIOC ;
#ifndef REVX
	pioptr->PIO_CODR = LCD_CS1 ;		// Select LCD
#endif 
	PIOA->PIO_CODR = LCD_A0 ;
#ifndef REVX
	pioptr->PIO_CODR = LCD_RnW ;		// Write
#endif
	pioptr->PIO_ODSR = val ;

	pioptr->PIO_SODR = LCD_E ;			// Start E pulse
	// Need a delay here (250nS)
	TC0->TC_CHANNEL[0].TC_CCR = 5 ;	// Enable clock and trigger it (may only need trigger)
	while ( TC0->TC_CHANNEL[0].TC_CV < 9 )		// Value depends on MCK/2 (used 18MHz)
	{
		// Wait
	}
	pioptr->PIO_CODR = LCD_E ;			// End E pulse
	PIOA->PIO_SODR = LCD_A0 ;				// Data
}


#ifdef SIMU
void refreshDisplay()
{
  memcpy(lcd_buf, DisplayBuf, sizeof(DisplayBuf));
  lcd_refresh = true;
}
#else
void refreshDisplay()
{

	register Pio *pioptr ;
  register uint8_t *p=DisplayBuf; // pointer to THE display buffer -- will be used to itterate through buffer
	uint8_t lcd_page_addr;
	uint8_t column_start_lo; 
	register uint32_t ebit ;

	ebit = LCD_E ;

	pioptr = PIOA ;
	pioptr->PIO_PER = 0x00000080 ;		// Enable bit 7 (LCD-A0)
	pioptr->PIO_OER = 0x00000080 ;		// Set bit 7 output

// read the inputs, and lock the LCD lines
	LcdInputs = PIOC->PIO_PDSR << 1 ; // 6 LEFT, 5 RIGHT, 4 DOWN, 3 UP ()
	LcdLock = 1 ;

	pioptr = PIOC ;
	pioptr->PIO_OER = 0x0C0030FFL ;		// Set bits 27,26,15,13,12,7-0 output

  for( lcd_page_addr=0; lcd_page_addr < 8; lcd_page_addr++) // set the next page address and start column in the LCD
	{
			// not dealing with flipped display
#ifdef ERC12864_14	
			column_start_lo = COL_ADDR_OFFSET+3; 	
#elif defined ERC24064_1
			column_start_lo = COL_ADDR_OFFSET;
#else
			column_start_lo = COL_ADDR_OFFSET+4; 
#endif			
		lcdSendCtl( column_start_lo &0xf) ;
		lcdSendCtl(0x10 | ((column_start_lo & 0xf0 )>>4)) ; // send column-address-high commnad -- data part always 0
		lcdSendCtl( 0xB0 | (lcd_page_addr + PAGE_ADDR_OFFSET)); // send page addr command with ored in pageaddr
 
#if (!defined(REVX) )
		pioptr->PIO_CODR = LCD_CS1 ;		// Select LCD
#endif // nREVX 

		PIOA->PIO_SODR = LCD_A0 ;			// Data
#if (!defined(REVX) )
		pioptr->PIO_CODR = LCD_RnW ;		// Write
#endif // nREVX 

// note magic number 128 !!
		for( int line_cnt=0; line_cnt < 128; line_cnt++)	// transmitting one line to LCD before next page address needs to be set again
		{

			pioptr->PIO_ODSR = *p++ ;			// copy the display buffer data to the LCD	
			pioptr->PIO_SODR = ebit ;			// Start E pulse

			// Need a delay here (250nS)
			TC0->TC_CHANNEL[0].TC_CCR = 5 ;	// Enable clock and trigger it (may only need trigger)

			while ( TC0->TC_CHANNEL[0].TC_CV < 500 )		// Value depends on MCK/8
			; 		// Wait for the lcd having received the byte
		
			pioptr->PIO_CODR = ebit ;			// End E pulse
		} // end column for loop
	} // end page for loop

	pioptr->PIO_ODSR = 0xFF ;					// Drive lines high

	pioptr->PIO_PUER = 0x000000FEL ;	// Set bits 1, 3, 4, 5 with pullups
	pioptr->PIO_ODR = 0x000000FEL ;		// Set bits 1, 3, 4, 5 input
	// ????
	//	pioptr->PIO_PUER = 0x0000003AL ;	// Set bits 1, 3, 4, 5 with pullups
	//  pioptr->PIO_ODR = 0x0000003AL ;		// Set bits 1, 3, 4, 5 input
	// ????

	pioptr->PIO_ODSR = 0xFE ;					// Drive D0 low
	LcdLock = 0 ;

}

#endif // PCBSKY
#endif // PCBSKY

#ifdef PCBSKY
// This specially for the 9XR
extern void start_timer0( void ) ;
extern void stop_timer0( void ) ;

extern "C" void dispUSB( void ) ;
void dispUSB()
{
	// Put USB on display here?
	start_timer0() ;
	lcd_init() ;
	OptrexDisplay = 1 ;
	lcd_clear() ;
	refreshDisplay() ;
	OptrexDisplay = 0 ;
	lcd_clear() ;
	lcd_putcAtt( 48, 24, 'U', DBLSIZE ) ;
	lcd_putcAtt( 60, 24, 'S', DBLSIZE ) ;
	lcd_putcAtt( 72, 24, 'B', DBLSIZE ) ;
	refreshDisplay() ;
	stop_timer0() ;
}

#endif

#ifdef PCB9XT
void refreshDisplay()
{
	displayToM64() ;	
}

void lcdSetRefVolt(uint8_t val)
{
	M64Contrast = val ;
	M64SetContrast = 1 ;
}

void lcd_init()
{
}

void backlight_on()
{
}

void backlight_off()
{
}
#endif // PCB9XT

