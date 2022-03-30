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
//	relativo a taxa de transmissão atual, deve ficar antes do #include "ModBusSlave.h"
//////////////////////////////////////////////////////////////////////////////////////////////////
extern void  liga_timer_modbus()
{
	OCR2=173;			// Ajusta o valor de comparação do timer 2 para 3,5 caracteres
	TCNT2=0;			// Zera a contagem do timer 2
	TCCR2=0b00000101;	// habilita o clock do timer 2 com prescaller para 3,5 caracteres
	TIMSK|=0b10000000;	// habilita a interrupção do timer 2
}

#include "ModBusSlave.h"

// configuração da serial
#define F_CPU 8000000 // frequencia de clock do microcontrolador
#define USART_BAUDRATE 19200 // taxa de transmissão desejada
#define STOPBITS 2 // número de bits de parada 1 ou 2
#define BAUD_PRESCALE ((F_CPU/ (USART_BAUDRATE * (long)16))-1) //calcula o valor do prescaler da usart
// fim da configuração da serial

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrupção de recepção de caractere
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
	if(ModBus.status==aguardando && ModBus.rxpt==6) // recebe o começo do pacote
	{
		ModBusDefineFunction(ModBus.rxbuf[1]); // seta a função
		
		if(ModBus.rxbuf[0]==ModBus.end_modbus) //se o endereço confere inicia a recepção
		{
			ModBus.status=recebendo;
		}
		else // senão ignora o pacote
		{
			ModBus.status=ignorando;
		}

	}
	if(ModBus.status!=aguardando && ModBus.rxpt==ModBus.rxsize) // recebe o restante do pacote
	{
		if(ModBus.status==recebendo) // se recebeu o pacote com endereço válido
		{
			ModBusProcess(); // inicia o processamento do pacote
		}
		if(ModBus.status==ignorando) // se estava ignorando o pacote
		{
			ModBusReset(); // reinicia a modbus para receber o próximo pacote
		}
	}
	if(ModBus.rxpt<tam_buff_recep) ModBus.rxpt++; // incrementa o ponteiro de recepção se o tamanho não chegou no limite
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrupção de caractere transmitido
//////////////////////////////////////////////////////////////////////////////////////////////////
ISR(USART_UDRE_vect) // interrupção de caractere transmitido
{
	if(ModBus.txpt>=ModBus.txsize) // se transmitiu o ultimo caractere do pacote
	{
		ModBusReset(); // prepara para receber nova transmissão
		UCSRB &= ~(1 << UDRIE); // desabilita a interrupção e transmissão
		//PORTD&=~0b00000100; // Habilita a recepção do driver 485 se necessário
	}
	else // se ainda não é o ultimo byte do pacote
	{
		UDR = ModBus.txbuf[ModBus.txpt]; // transmite o byte
		ModBus.txpt++; // incrementa o ponteiro de transmissão
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Interrupção do temporizador
//////////////////////////////////////////////////////////////////////////////////////////////////

ISR(TIMER2_COMP_vect) // interrupção do temporizador
{
	if(ModBus.status==iniciandoTransmisao)
	{
		//PORTD|=0b00000100; // Habilita a transmissão do driver 485 se necessário
		ModBus.status = transmitindo; // indica que está transmitindo
		TCCR2=0b00000000; // desliga o timer
		UDR = ModBus.txbuf[ModBus.txpt]; // transmite o primeiro byte, os seguintes são transmitidos na interrupção da serial
		UCSRB |= (1 << UDRIE); // habilita a interrupção da serial
		ModBus.txpt++; // incrementa o ponteiro de transmissção
	}
	
	if(ModBus.status==aguardando || ModBus.status==recebendo) // se o timer disparou na recepção houve erro  
	{
		//PORTD&=~0b00000100; // Habilita a recepção do driver 485 se necessário
		ModBusReset(); // prepara para receber nova transmissão
		TCCR2=0b00000000; // desliga o timer
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//	Inicializa a comunicação serial
//////////////////////////////////////////////////////////////////////////////////////////////////
void usart_init() // inicia a comunicação serial
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
    usart_init(); // inicia a comunicação serial utilizada na ModBus
	ModBusReset(); // prepara para receber a transmissão
	// inserir outras inicializações aqui

	sei(); // Habilita o Global Interrupt Enable flag permitindo interrupções
    while (1) 
    {
	
    }
}

