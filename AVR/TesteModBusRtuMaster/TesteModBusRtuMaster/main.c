/*
 * Teste ModBus Rtu Master
 *
 * Testado no ATmega32
 *
 * Created: 27/03/2022 09:29:55
 * Author : Kerschbaumer
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000 // frequencia de clock do microcontrolador necess�rio para a serial e para o delay
#include <util/delay.h>

#define E1 ((PINB&0b00000001)!=0)
#define E2 ((PINB&0b00000010)!=0)
#define E3 ((PINB&0b00000100)!=0)
#define E4 ((PINB&0b00001000)!=0)
#define E5 ((PINB&0b00010000)!=0)
#define E6 ((PIND&0b10000000)!=0)
#define E7 ((PIND&0b00000001)!=0)
#define E8 ((PIND&0b00000010)!=0)
#define E9 ((PIND&0b00000100)!=0)
#define E10 ((PIND&0b00001000)!=0)
#define E11 ((PIND&0b00010000)!=0)
#define E12 ((PIND&0b00100000)!=0)
#define E13 ((PIND&0b01000000)!=0)

// saidas
#define S1_H PORTC|=0b00000001
#define S1_L PORTC&=~0b00000001

#define S2_H PORTC|=0b00000010
#define S2_L PORTC&=~0b00000010

#define S3_H PORTC|=0b00000100
#define S3_L PORTC&=~0b00000100

#define S4_H PORTC|=0b00001000
#define S4_L PORTC&=~0b00001000

#define S5_H PORTC|=0b00010000
#define S5_L PORTC&=~0b00010000

#define S6_H PORTC|=0b00100000
#define S6_L PORTC&=~0b00100000

#define S7_H PORTC|=0b01000000
#define S7_L PORTC&=~0b01000000

#define S8_H PORTC|=0b10000000
#define S8_L PORTC&=~0b10000000

///////////////////////////////////////////////////////////////////////////////////////////////
//                               D E L A Y
///////////////////////////////////////////////////////////////////////////////////////////////
void delay(unsigned int del)
{
	while(del!=0)
	{
		del--;
		DDRC&=0x11111111;
	}
}

// vari�veis para teste da modBus
uint8_t bitData[100];
uint16_t wordData[50];

void iniciaTransmissao(uint8_t dado)
{
	//PORTD|=0b00000100; // Habilita a transmiss�o do driver 485 se necess�rio
	UDR = dado; // transmite o primeiro byte, os seguintes s�o transmitidos na interrup��o da serial
	UCSRB |= (1 << UDRIE); // habilita a interrup��o da serial
}

#include "ModBusMaster.h"

// configura��o da serial
#define USART_BAUDRATE 19200 // taxa de transmiss�o desejada
#define STOPBITS 2 // n�mero de bits de parada 1 ou 2
#define BAUD_PRESCALE ((F_CPU/ (USART_BAUDRATE * (long)16))-1) //calcula o valor do prescaler da usart
// fim da configura��o da serial

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Liga o temporizador usado na modBus com o intervalor ajustado para 1ms
//	Ajustar para o clock utilizado
//////////////////////////////////////////////////////////////////////////////////////////////////
void  inicia_timer_1ms()
{
	OCR2=125;			// Ajusta o valor de compara��o do timer 2 
	TCNT2=0;			// Zera a contagem do timer 2
	TCCR2=0b00001100;	// habilita o clock do timer 2 com prescaller
	TIMSK|=0b10000000;	// habilita a interrup��o do timer 2
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrup��o de recep��o de caractere
//////////////////////////////////////////////////////////////////////////////////////////////////
ISR(USART_RXC_vect)
{
	if(ModBus.status==aguardandoResposta) // est� aguardado dados
	{
		ModBus.rxbuf[ModBus.rxpt] = UDR; // recebe o byte
		if(ModBus.rxpt==ModBus.rxsize-1) // pacote completo
		{
			
			modBusTimeoutCounterStop();
			ModBus.status=respostaRecebida;
			ModBusProcess(); // processa a resposta
		}
		if(ModBus.rxpt<tam_buff_recep) ModBus.rxpt++; // incrementa o ponteiro de recep��o se o tamanho n�o chegou no limite
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrup��o de caractere transmitido
//////////////////////////////////////////////////////////////////////////////////////////////////
ISR(USART_UDRE_vect) // interrup��o de caractere transmitido
{
	if(ModBus.txpt>=ModBus.txsize) // se transmitiu o ultimo caractere do pacote
	{
		ModBus.rxpt=0;
		modBusTimeoutCounterStart();
		ModBus.status=aguardandoResposta;
		UCSRB &= ~(1 << UDRIE); // desabilita a interrup��o e transmiss�o
		//PORTD&=~0b00000100; // Habilita a recep��o do driver 485 se necess�rio
	}
	else // se ainda n�o � o ultimo byte do pacote
	{
		UDR = ModBus.txbuf[ModBus.txpt]; // transmite o byte
		ModBus.txpt++; // incrementa o ponteiro de transmiss�o
	}
}



ISR(TIMER2_COMP_vect) // interrup��o do temporizador
{
	if(modBusTimeoutCounter==modBusTimeout_ms) // estouro de timeout
	{
		 ModBusTimeout(); // chama a fun��o que trata a falta de resposta do escravo
		 modBusTimeoutCounter++; // incrementa para parar de contar 
	}
	if(modBusTimeoutCounter<modBusTimeout_ms) modBusTimeoutCounter++;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Inicializa a comunica��o serial
//////////////////////////////////////////////////////////////////////////////////////////////////
void usart_init() // inicia a comunica��o serial
{
	UCSRB |= (1 << RXEN) | (1 << TXEN); // Turn on the transmission and reception circuitry
	#if STOPBITS == 2
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1) | (1 << USBS);  // Use 8-bit character sizes 2 stop bits
	#else
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);  // Use 8-bit character sizes 1 stop bit
	#endif

	UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
	UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register

	UCSRB |= (1 << RXCIE); // Enable the USART Recieve Complete interrupt (USART_RXC)
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//                                    M A I N
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
int main()
{
	usart_init(); // inicia a comunica��o serial utilizada na ModBus
	ModBusReset(); // prepara para receber a transmiss�o
	inicia_timer_1ms();
	// inserir outras inicializa��es aqui
	
	sei(); // Habilita o Global Interrupt Enable flag permitindo interrup��es
	
	DDRC=0b11111111; // sa�das

	while(1)
	{
		// entradas
		if(E1) bitData[0]=1;
		else bitData[0]=0;
		
		if(E2) bitData[1]=1;
		else bitData[1]=0;

		if(E3) bitData[2]=1;
		else bitData[2]=0;
		
		if(E4) bitData[3]=1;
		else bitData[3]=0;
		
		if(E5) bitData[4]=1;
		else bitData[4]=0;
		
		if(E6) bitData[5]=1;
		else bitData[5]=0;
		
		if(E7) bitData[6]=1;
		else bitData[6]=0;
		
		if(E8) bitData[7]=1;
		else bitData[7]=0;
		
		if(E9) bitData[8]=1;
		else bitData[8]=0;
		
		if(E10) bitData[9]=1;
		else bitData[9]=0;
		
		if(E11) bitData[10]=1;
		else bitData[10]=0;
		
		if(E12) bitData[11]=1;
		else bitData[11]=0;
		
		if(E13) bitData[12]=1;
		else bitData[12]=0;
		
		////
		if(ModBus.status==inativo)
		{
			_delay_ms(1000);
			//modBusReadCoilStatusFC01(24, 1, &bitData[24]);
			//modBusReadHoldingRegistersFC03(10, 22, wordData);
			//modBusForceSingleCoilFC05(1, bitData[0]);
			//modBusPresetSingleRegisterFC06(12,11223);
			bitData[30]=1;
			bitData[31]=0;
			bitData[32]=0;
			bitData[33]=1;
			bitData[34]=1;
			bitData[35]=0;
			bitData[36]=0;
			bitData[37]=1;
			bitData[38]=0;
			bitData[39]=1;
			//modBusForceMultipleCoilsFC15(30, 10, &bitData[30]);
			wordData[0]=10;
			wordData[1]=11;
			wordData[2]=12;
			wordData[3]=13;
			wordData[4]=14;
			modBusPresetMultipleRegistersFC16(10, 5, wordData);
		}
		if(ModBus.status==respostaRecebida)
		{			
			if(ModBus.erro==semErro)
			{
				// processa os dados lidos
						if(bitData[24]==0) S5_L;
						else S5_H;
						
						if(bitData[25]==0) S6_L;
						else S6_H;
						
						if(bitData[26]==0) S7_L;
						else S7_H;
						
						if(bitData[27]==0) S8_L;
						else S8_H;
						
				S1_H;
				S2_L;
				S3_L;
			}
			else
			{
				// trata o erro de leitura
				S2_H;
				S1_L;
			}
			ModBusReset(); // prepara para a pr�xima transmiss�o
		}
		if(ModBus.status==semResposta)
		{
			S3_H;
			S1_L;
			// trata o erro sem Resposta ou resposta muito curta
			// pode ser um c�digo de erro...
			ModBusReset(); // prepara para a pr�xima transmiss�o
		}		
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////