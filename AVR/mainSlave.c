/*
 * Teste ModBus Rtu Slave
 *
 * Testado no ATmega32
 *
 * Created: 27/03/2022 09:29:55
 * Author : Kerschbaumer
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Liga o temporizador usado na modBus com intervalor igual ou superior de 3,5 caracteres
//	relativo a taxa de transmiss�o atual, deve ficar antes do #include "ModBusSlave.h"
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void  liga_timer_modbus()
{
	OCR2=173;			// Ajusta o valor de compara��o do timer 2 para 3,5 caracteres
	TCNT2=0;			// Zera a contagem do timer 2
	TCCR2=0b00000101;	// habilita o clock do timer 2 com prescaller para 3,5 caracteres
	TIMSK|=0b10000000;	// habilita a interrup��o do timer 2
}

#include "ModBusSlave.h"

// configura��o da serial
#define F_CPU 8000000 // frequencia de clock do microcontrolador
#define USART_BAUDRATE 19200 // taxa de transmiss�o desejada
#define STOPBITS 2 // n�mero de bits de parada 1 ou 2
#define BAUD_PRESCALE ((F_CPU/ (USART_BAUDRATE * (long)16))-1) //calcula o valor do prescaler da usart
// fim da configura��o da serial

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrup��o de recep��o de caractere
//////////////////////////////////////////////////////////////////////////////////////////////////
ISR(USART_RXC_vect) 
{
	ModBus.rxbuf[ModBus.rxpt] = UDR; // recebe o byte
	if(ModBus.status==aguardando && ModBus.rxpt==0) // primeiro byte do pacote
	{
		liga_timer_modbus(); // liga o timer para detectar pacotes truncados
	}
	else
	{
		TCNT2=0; // Zera a contagem do timer 2 
	}
	if(ModBus.status==aguardando && ModBus.rxpt==6) // recebe o come�o do pacote
	{
		ModBusDefineFunction(ModBus.rxbuf[1]); // seta a fun��o
		
		if(ModBus.rxbuf[0]==ModBus.end_modbus) //se o endere�o confere inicia a recep��o
		{
			ModBus.status=recebendo;
		}
		else // sen�o ignora o pacote
		{
			ModBus.status=ignorando;
		}

	}
	if(ModBus.status!=aguardando && ModBus.rxpt==ModBus.rxsize) // recebe o restante do pacote
	{
		if(ModBus.status==recebendo) // se recebeu o pacote com endere�o v�lido
		{
			ModBusProcess(); // inicia o processamento do pacote
		}
		if(ModBus.status==ignorando) // se estava ignorando o pacote
		{
			ModBusReset(); // reinicia a modbus para receber o pr�ximo pacote
		}
	}
	if(ModBus.rxpt<tam_buff_recep) ModBus.rxpt++; // incrementa o ponteiro de recep��o se o tamanho n�o chegou no limite
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrup��o de caractere transmitido
//////////////////////////////////////////////////////////////////////////////////////////////////
ISR(USART_UDRE_vect) // interrup��o de caractere transmitido
{
	if(ModBus.txpt>=ModBus.txsize) // se transmitiu o ultimo caractere do pacote
	{
		ModBusReset(); // prepara para receber nova transmiss�o
		UCSRB &= ~(1 << UDRIE); // desabilita a interrup��o e transmiss�o
		//PORTD&=~0b00000100; // Habilita a recep��o do driver 485 se necess�rio
	}
	else // se ainda n�o � o ultimo byte do pacote
	{
		UDR = ModBus.txbuf[ModBus.txpt]; // transmite o byte
		ModBus.txpt++; // incrementa o ponteiro de transmiss�o
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrup��o do temporizador
//////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER2_COMP_vect) // interrup��o do temporizador
{
	if(ModBus.status==iniciandoTransmisao)
	{
		//PORTD|=0b00000100; // Habilita a transmiss�o do driver 485 se necess�rio
		ModBus.status = transmitindo; // indica que est� transmitindo
		TCCR2=0b00000000; // desliga o timer
		UDR = ModBus.txbuf[ModBus.txpt]; // transmite o primeiro byte, os seguintes s�o transmitidos na interrup��o da serial
		UCSRB |= (1 << UDRIE); // habilita a interrup��o da serial
		ModBus.txpt++; // incrementa o ponteiro de transmiss��o
	}
	
	if(ModBus.status==aguardando || ModBus.status==recebendo) // se o timer disparou na recep��o houve erro  
	{
		//PORTD&=~0b00000100; // Habilita a recep��o do driver 485 se necess�rio
		ModBusReset(); // prepara para receber nova transmiss�o
		TCCR2=0b00000000; // desliga o timer
	}
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

int main(void)
{
    usart_init(); // inicia a comunica��o serial utilizada na ModBus
	ModBusReset(); // prepara para receber a transmiss�o
	// inserir outras inicializa��es aqui

	sei(); // Habilita o Global Interrupt Enable flag permitindo interrup��es
    while (1) 
    {
	
    }
}

