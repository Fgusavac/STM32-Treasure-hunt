#include "LED_MATRIX.h"

const uint8_t data[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x02, 0x02
};

const uint8_t data2[] = {
    0x00, 0x00, 0x00, 0x40, 0x40, 0x7e, 0x02, 0x02
};

const uint8_t data3[] = {
    0x10, 0x10, 0x70, 0x40, 0x40, 0x7e, 0x02, 0x02
};

const uint8_t empty[] =
{
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//Initialise Dot Matrix display
void matrixInit (void)
{
	max7219_write(0x09, 0); //disbale decoding mode
	max7219_write(0x0a, 0x01); //3.32 for intensity
	max7219_write(0x0B, 0x07); // scan all 7 columbs
	max7219_write(0x0C, 0x01);// normal opperation (i think this is shutdown mode
	max7219_write(0x0F, 00); //disable the display test
}

//creates patterns of 5 to progress through
void pattern1 ()
{
	for (int i=0; i <=8; i++)
	  		  	{
	  		  	 max7219_write(i, data[i-1]);
	  		  	}
}

void pattern2 ()
{
	for (int i=0; i <=8; i++)
	  		  	{
	  		  	 max7219_write(i, data2[i-1]);
	  		  	}
}

void pattern3 ()
{
	for (int i=0; i <=8; i++)
	  		  	{
	  		  	 max7219_write(i, data3[i-1]);
	  		  	}
}

void empty_matrix()
{
	for (int i=0; i <=8; i++)
		  		  	{
		  		  	 max7219_write(i, empty[i-1]);
		  		  	}
}
