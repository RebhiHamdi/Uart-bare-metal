#include <stdint.h>

#define periph_base                            (0x40000000UL)

#define AHB1_OFFSET                           (0x00020000UL)
#define AHB1_BASE                             (periph_base + AHB1_OFFSET)

#define GPIOA_OFFSET                          (0x00000000UL)
#define GPIOA_BASE                            (AHB1_BASE + GPIOA_OFFSET)

#define RCC_OFFSET                            (0x00003800UL)
#define RCC_BASE                              (AHB1_BASE + RCC_OFFSET)


#define APB1_OFFSET                           (0x00000000UL)
#define APB1_BASE                             (periph_base + APB1_OFFSET)


#define USART2_OFFSET						  (0x00004400UL)
#define USART2_BASE                           (APB1_BASE + USART2_OFFSET)




#define GPIOA_EN			 			(1U << 0)
#define USART2_EN	    	  			(1U << 17)

#define CR1_TE                  		(1U << 3)
#define CR1_UE                  		(1U << 13)
#define CR1_RE                          (1U << 2)

#define SR_TXE                   		(1U << 7)
#define SR_RXNE      			        (1U << 5)

#define SYS_FREQ                  		16000000
#define Periph_CLK	           			SYS_FREQ
#define Uart_BaudRate	     	  		115200



typedef struct
{
  volatile uint32_t SR;         /*Address offset: 0x00 */
  volatile uint32_t DR;         /*Address offset: 0x04 */
  volatile uint32_t BRR;        /*Address offset: 0x08 */
  volatile uint32_t CR1;        /*Address offset: 0x0C */
  volatile uint32_t CR2;        /*Address offset: 0x10 */
  volatile uint32_t CR3;        /*Address offset: 0x14 */
  volatile uint32_t GTPR;       /*Address offset: 0x18 */
} USART_TypeDef;


typedef struct
{
  volatile uint32_t CR;            /*Address offset: 0x00 */
  volatile uint32_t PLLCFGR;       /*Address offset: 0x04 */
  volatile uint32_t CFGR;          /*Address offset: 0x08 */
  volatile uint32_t CIR;           /*Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;      /*Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;      /*Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;      /*Address offset: 0x18 */
  uint32_t      RESERVED0;
  volatile uint32_t APB1RSTR;      /*Address offset: 0x20 */
  volatile uint32_t APB2RSTR;      /*Address offset: 0x24 */
  uint32_t      RESERVED1[2];
  volatile uint32_t AHB1ENR;       /*Address offset: 0x30 */
  volatile uint32_t AHB2ENR;       /*Address offset: 0x34 */
  volatile uint32_t AHB3ENR;       /*Address offset: 0x38 */
  uint32_t      RESERVED2;
  volatile uint32_t APB1ENR;       /*Address offset: 0x40 */
  volatile uint32_t APB2ENR;       /*Address offset: 0x44 */
  uint32_t      RESERVED3[2];
  volatile uint32_t AHB1LPENR;     /*Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;     /*Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;     /*Address offset: 0x58 */
  uint32_t      RESERVED4;
  volatile uint32_t APB1LPENR;     /*Address offset: 0x60 */
  volatile uint32_t APB2LPENR;     /*Address offset: 0x64 */
  uint32_t      RESERVED5[2];
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  uint32_t      RESERVED6[2];
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
  uint32_t      RESERVED7[1];
  volatile uint32_t DCKCFGR;
} RCC_TypeDef;



typedef struct
{
  volatile uint32_t MODER;    /*Address offset: 0x00      */
  volatile uint32_t OTYPER;   /*Address offset: 0x04      */
  volatile uint32_t OSPEEDR;  /*Address offset: 0x08      */
  volatile uint32_t PUPDR;    /*Address offset: 0x0C      */
  volatile uint32_t IDR;      /*Address offset: 0x10      */
  volatile uint32_t ODR;      /*Address offset: 0x14      */
  volatile uint32_t BSRR;     /*Address offset: 0x18      */
  volatile uint32_t LCKR;     /*Address offset: 0x1C      */
  volatile uint32_t AFR[2];   /* Address offset: 0x20-0x24 */
} GPIO_TypeDef;



#define RCC          ( ( RCC_TypeDef* ) RCC_BASE )
#define GPIOA        ( ( GPIO_TypeDef* ) GPIOA_BASE )
#define USART2       ( ( USART_TypeDef* ) USART2_BASE )


char key;


static void uart_set_baudrate ( USART_TypeDef *USARTx,uint32_t Pheriphclk, uint32_t Baudrate );
static uint16_t compute_uart_bd ( uint32_t Pheriphclk, uint32_t Baudrate );
void usart2_rxtx_init(void);
void USART2_WRITE( int ch );
int __io_putchar(int ch);
char usart2_read(void);


int main(void) {
	usart2_rxtx_init();
	//char key;
		while(1) {

			key = usart2_read();

			switch (key) {
			    case '1':
			        printf("lundi\n\r");
			        break;
			    case '2':
			        printf("mardi\n\r");
			        break;
			    case '3':
			        printf("mercredi\n\r");
			        break;
			    case '4':
			        printf("jeudi\n\r");
			        break;
			    case '5':
			        printf("vendredi\n\r");
			        break;
			    case '6':
			        printf("samedi\n\r");
			        break;
			    case '7':
			        printf("dimanche\n\r");
			        break;
			    default :
			    	printf("Eror\n\r");
			}

 }
}


void usart2_rxtx_init(void) {

		/***************** Configuration UART gpio pin ***************************/

		/* 1- Enable clock access to gpioa */

			RCC -> AHB1ENR |=  GPIOA_EN;

		/* 2- Set PA2 mode to Alternate function */

			GPIOA -> MODER |= (1U << 5);
			GPIOA -> MODER &= ~(1U << 4);

		/* 3- Set PA2 alternate function type to AP07 */

			GPIOA -> AFR[0] |= (1U << 8);
			GPIOA -> AFR[0] |= (1U << 9);
			GPIOA -> AFR[0] |= (1U << 10);
			GPIOA -> AFR[0] &= ~(1U << 11);


		/* 4- Set PA3 mode to Alternate function */

			GPIOA -> MODER |= (1U << 7);
			GPIOA -> MODER &= ~(1U << 6);

		/* 5- Set PA3 alternate function type to AP07 */

			GPIOA -> AFR[0] |= (1U << 12);
			GPIOA -> AFR[0] |= (1U << 13);
			GPIOA -> AFR[0] |= (1U << 14);
			GPIOA -> AFR[0] &= ~(1U << 15);

		/**************** Configuratioin UART baudrate *************************/

	    /* 1- Enable clock access to uart2 */

			RCC -> APB1ENR |= USART2_EN;

		/* 2- Configure Uart baudrate */

			uart_set_baudrate(USART2, Periph_CLK, Uart_BaudRate);

		/* 3- Configure uart transfert direction */

			USART2 -> CR1 = (CR1_TE | CR1_RE);

		/* 4- Enable uart module */

			USART2 -> CR1 |= CR1_UE;


}

int __io_putchar(int ch) {

	USART2_WRITE(ch);
	return ch;

}

static void uart_set_baudrate (USART_TypeDef *USARTx, uint32_t Pheriphclk, uint32_t Baudrate ) {

	USARTx -> BRR = compute_uart_bd(Pheriphclk, Baudrate);

}



static uint16_t compute_uart_bd ( uint32_t Pheriphclk, uint32_t Baudrate ) {

	return ( ( Pheriphclk + ( Baudrate / 2U ) ) / Baudrate );

}


void USART2_WRITE( int ch ) {

	/* Make sure the transmit data is empty */

	   while (!( USART2 -> SR & SR_TXE )) {}

	/*write to transmit data register */

	    USART2 -> DR = (ch & 0xFF);
}


char usart2_read(void) {

	/* 1- Make sure the receive data register is not empty */

	while(!(USART2 -> SR & SR_RXNE)){}

	/* 2- Read Data */

	return USART2 -> DR;


}



