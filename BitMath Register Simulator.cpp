#include <stdio.h>
#include <stdint.h>

#define PINPOS(x)					( (x == 8) ? 0 :\
									  (x == 9) ? 1 :\
									  (x == 10) ? 2 :\
									  (x == 11) ? 3 :\
									  (x == 12) ? 4 :\
									  (x == 13) ? 5 :\
									  (x == 14) ? 6 :\
									  (x == 15) ? 7 : x )
									  
#define GPIO_MODE_PINPOSITION		( 4 * PINPOS(pinNumber) )
#define GPIO_MODE_BITPOS_0			( GPIO_MODE_PINPOSITION )
#define GPIO_MODE_BITPOS_1			( GPIO_MODE_BITPOS_0 + 1 )
#define GPIO_CNF_BITPOS_0			( GPIO_MODE_BITPOS_0 + 2 )
#define GPIO_CNF_BITPOS_1			( GPIO_MODE_BITPOS_0 + 3 )

void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
}

int main()
{
	uint8_t pinNumber = 8;
	
	uint32_t tempReg = 0xFFFFFFFF;
	tempReg &= ~( (1<<GPIO_MODE_BITPOS_1) | (1<<GPIO_MODE_BITPOS_0) );

	
	
	
	
	
	
	printBits(sizeof(tempReg), &tempReg);
}
