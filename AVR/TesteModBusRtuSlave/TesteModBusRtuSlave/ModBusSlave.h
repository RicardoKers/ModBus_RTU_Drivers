/*
 *		ModBusSlave.h
 *
 *		Versão: 03-2022
 *      Author: Kerschbaumer
 *
 *  Funções implementadas:
 *
 *  	Read Coil Status (FC=01)
 *  	Read Holding Registers (FC=03)
 *  	Force Single Coil (FC=05)
 *  	Preset Single Register (FC=06)
 *  	Force Multiple Coils (FC=15)
 *  	Preset Multiple Registers (FC=16)
 */

// Configuração MODBus
#define endereco_modbus 1 // endereço inicial da modbus, pode ser mudado depois
#define num_reg_words_modbus 50 // número de registradores (words) usados na modbus (variável data_word) funções 3 e 16
#define num_reg_bits_modbus 100 // número de registradores (bits) usados na modbus (variável data_bits) funções 1 e 15
#define tam_buff_recep 255
#define tam_buff_trans 255
// final Configuração MODBus

struct mb
{
	//enum e_status {aguardando, recebendo, ignorando, iniciandoTransmisao, delayTransmisao, transmitindo, reiniciando} status;
	enum e_status {aguardando, recebendo, ignorando, iniciandoTransmisao, transmitindo} status;
	uint8_t end_modbus; // armazena o endereço na modbus
	uint16_t rxsize; // tamanho do pacote na recepção
	uint16_t txsize; // tamanho do pacote na transmissão
	uint8_t rxbuf[tam_buff_recep]; // buffer de recepção
	uint8_t txbuf[tam_buff_trans]; // buffer de transmissão
	uint8_t funcao;
	uint16_t data_reg[num_reg_words_modbus]; // dados de words a serem transmitidos e recebidos pela modbus
	uint8_t data_bit[num_reg_bits_modbus]; // dados de bits a serem transmitidos e recebidos pela modbus
	uint16_t rxpt; // ponteiro para o buffer de recepçao, necessario em algumas arquiteturas
	uint16_t txpt; // ponteiro para o buffer de transmissão, necessario em algumas arquiteturas
} ModBus;

//////////////////////////////////////////////////////////////////////////////////////////////////
//                            P R O T O T I P O S
//////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t CRC16Table256(uint16_t i) // gera tabela de crc
{
	uint16_t crc,c,j;
	crc = 0;
	c   = (uint16_t) i;
	for (j=0; j<8; j++)
	{
		if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ 0xA001;
		else crc =   crc >> 1;
		c = c >> 1;
    }
	return crc;
}


uint16_t update_crc_16(uint16_t crc, uint8_t c) // atualiza o valor do crc
{
	uint16_t tmp, short_c;
	short_c = 0x00ff & (uint16_t) c;
	tmp =  crc       ^ short_c;
	crc = (crc >> 8) ^ CRC16Table256((uint16_t) tmp & 0xff);
	return crc;
}

uint16_t CRC16(uint8_t *ptr,uint16_t npts) // calcula o crc de um vetor
{
	uint16_t crc;
	uint16_t i;
	crc=0xffff;// valor inicial para CRC16modbus
	for(i=0;i<npts;i++)
	{
		crc=update_crc_16(crc,(uint8_t) *(ptr+i));
    }
	return crc;
}

void ModBusReset()
{
	ModBus.rxsize=6;
	ModBus.end_modbus=endereco_modbus;
	ModBus.status=aguardando;
	ModBus.rxpt=0;
	ModBus.txpt=0;
}

void ModBusDefineFunction(uint8_t rchar)
{
	uint16_t tmp;
	if(rchar==1) //função 1 (identifica a função 1 do modbus)
	{
		ModBus.rxsize = 7; // prepara para receber 7 bytes conforme função 1
		ModBus.funcao=1;
	}
	if(rchar==3) //função 3 (identifica a função 3 do modbus)
	{
		ModBus.rxsize = 7; // prepara para receber 7 bytes conforme função 3
		ModBus.funcao=3;
	}
	if(rchar==5) //função 5 (identifica a função 3 do modbus)
	{
		ModBus.rxsize = 7; // prepara para receber 7 bytes conforme função 5
		ModBus.funcao=5;
	}
	if(rchar==6) //função 6 (identifica a função 3 do modbus)
	{
		ModBus.rxsize = 7; // prepara para receber 7 bytes conforme função 6
		ModBus.funcao=6;
	}
	if(rchar==15) //função 15 (identifica a função 15 do modbus)
	{
		tmp=(uint16_t)((ModBus.rxbuf[4]<<8)|ModBus.rxbuf[5]);
		ModBus.rxsize = 8+(tmp/8); // depende do número de registradores a ser alterados
		if((tmp%8)!=0) ModBus.rxsize++; // soma 1 se o byte não foi completado
		ModBus.funcao=15;
	}
	if(rchar==16) //função 16 (identifica a função 16 do modbus)
	{
		tmp=(uint16_t)((ModBus.rxbuf[4]<<8)|ModBus.rxbuf[5]);
		ModBus.rxsize = 8+(tmp*2); // depende do número de registradores a ser alterados
		ModBus.funcao=16;
	}
}

void ModBusSendErrorMessage(uint8_t function, uint8_t code)
{
	uint16_t crc; // armazena o valor do crc do pacote
	ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote de resposta com o endereço
	ModBus.txbuf[1]=function|0x80; // indica a função 1 na resposta com erro
	ModBus.txbuf[2]=code; // indica o número de registradores transmitidos em bytes
	crc=CRC16(ModBus.txbuf,3); // calcula o crc da resposta
	ModBus.txbuf[3]=(uint8_t)(crc&0x00ff); // monta 8 bits do crc para transmitir
	ModBus.txbuf[4]=(uint8_t)(crc>>8); // monta mais 8 bits do crc para transmitir
	ModBus.txsize=5; // armazena o tamanho do pacote para transmissão
	ModBus.status = iniciandoTransmisao; // atualiza o status para o main ativar o timer
	liga_timer_modbus();
}

void ModBusProcess()
{
	uint16_t crc; // armazena o valor do crc do pacote
	uint16_t temp; // variável para valores temporários
	uint16_t num_reg; // número de registradores que estão sendo lidos
	uint16_t cont_tx; // armazena o tamanho do pacote de transmissão
	uint16_t cont_rx; // armazena a contagem de bytes da recepção
	uint16_t cont_bit; // conta o bita nas transnissões de bits
	uint16_t cont; // variável para contar os registradores transmitidos

	crc=CRC16(ModBus.rxbuf,ModBus.rxsize-1); // calcula o crc do pacote
	if((uint8_t)(crc&0x00ff)==ModBus.rxbuf[ModBus.rxsize-1]&&(uint8_t)(crc>>8)==ModBus.rxbuf[ModBus.rxsize]) // testa se o crc é válido
	{
		if(ModBus.funcao==1) // se for a função 1
		{
			temp=(uint16_t)((ModBus.rxbuf[2]<<8)|ModBus.rxbuf[3]); //recebe o endereço dos registradores a serem lidos
			num_reg=(uint16_t)((ModBus.rxbuf[4]<<8)|ModBus.rxbuf[5]); // recebe a quantidade de registradores a serem lidos
			if((temp+num_reg)<=num_reg_bits_modbus) // verifica se é válido
			{
				ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote de resposta com o endereço
				ModBus.txbuf[1]=1; // indica a função 1 na resposta
				cont_tx=3; // inicia o contador de tamanho do pacote de resposta
				ModBus.txbuf[cont_tx]=0; // zera o byte que vai receber os bits da resposta
				cont_bit=1;
				for(cont=0; cont<num_reg; cont++) // conta os registradores enviados
				{
					if(ModBus.data_bit[cont+temp]!=0) ModBus.txbuf[cont_tx]=ModBus.txbuf[cont_tx]|cont_bit; // se o bit for 1 grava um na resposta
					cont_bit=cont_bit<<1; // aponta para o próximo bit di byte de resposta
					if(cont_bit==0x100) // testa se ja montou um byte inteiro
					{
						cont_bit=1; // aponta para o primeiro bit
						cont_tx++; // incrementa o contador de bytes da resposta
						ModBus.txbuf[cont_tx]=0; // zera o byte que vai receber os bits da resposta
					}
				}
				cont_tx++; // incrementa o contador de bytes da resposta
				ModBus.txbuf[2]=(uint8_t)(cont_tx-3); // indica o número de registradores transmitidos em bytes
				crc=CRC16(ModBus.txbuf,(uint16_t)(cont_tx)); // calcula o crc da resposta
				ModBus.txbuf[cont_tx]=(uint8_t)(crc&0x00ff); // monta 8 bits do crc para transmitir
				cont_tx++; // incrementa o contador do tamanho da resposta
				ModBus.txbuf[cont_tx]=(uint8_t)(crc>>8); // monta mais 8 bits do crc para transmitir
				cont_tx++; // incrementa o contador do tamanho da resposta
				ModBus.txsize=(uint8_t)(cont_tx); // armazena o tamanho do pacote para transmissão
				ModBus.status = iniciandoTransmisao; // atualiza o status para o main ativar o timer
				liga_timer_modbus();
			}
			else
			{
				ModBusSendErrorMessage(1, 2); // retorna erro de endereço ilegal
			}
		}

		if(ModBus.funcao==3) // se for a função 3
		{
			temp=(uint16_t)((ModBus.rxbuf[2]<<8)|ModBus.rxbuf[3]); //recebe o endereço dos registradores a serem lidos
			num_reg=(uint16_t)((ModBus.rxbuf[4]<<8)|ModBus.rxbuf[5]); // recebe a quantidade de registradores a serem lidos
			if((temp+num_reg-1)<num_reg_words_modbus) // verifica se é válido
			{
				ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote de resposta com o endereço
				ModBus.txbuf[1]=3; // indica a função 3 na resposta
				ModBus.txbuf[2]=(uint8_t)(num_reg*2); // indica o número de registradores transmitidos em bytes
				cont_tx=3; // inicia o contador de tamanho do pacote de resposta
				for(cont=0; cont<num_reg; cont++) // conta os registradores enviados
				{
					ModBus.txbuf[cont_tx]=(uint8_t)((ModBus.data_reg[cont+temp])>>8); // envia os 8 bits mais altos do registrador
					cont_tx++; // incrementa o contador do tamanho da resposta
					ModBus.txbuf[cont_tx]=(uint8_t)((ModBus.data_reg[cont+temp])&0x00ff); // envia os 8 bits mais baixos do registrador
					cont_tx++; // incrementa o contador do tamanho da resposta
				}
				crc=CRC16(ModBus.txbuf,(uint16_t)((num_reg*2)+3)); // calcula o crc da resposta
				ModBus.txbuf[cont_tx]=(uint8_t)(crc&0x00ff); // monta 8 bits do crc para transmitir
				cont_tx++; // incrementa o contador do tamanho da resposta
				ModBus.txbuf[cont_tx]=(uint8_t)(crc>>8); // monta mais 8 bits do crc para transmitir
				cont_tx++; // incrementa o contador do tamanho da resposta
				ModBus.txsize=(uint8_t)(cont_tx); // armazena o tamanho do pacote para transmissão
				ModBus.status = iniciandoTransmisao; // atualiza o status para o main ativar o timer
				liga_timer_modbus();
			}
			else
			{
				ModBusSendErrorMessage(3, 2); // retorna erro de endereço ilegal
			}
		}

		if(ModBus.funcao==5) // se for a função 5
		{
			temp=(uint16_t)((ModBus.rxbuf[2]<<8)|ModBus.rxbuf[3]); //recebe o endereço do registrador a ser gravado
			if(temp<num_reg_bits_modbus&&temp>=0) // verifica se é válido
			{
				if((ModBus.rxbuf[4])==0xFF) ModBus.data_bit[temp]=1; // grava o valor do registrador
				else ModBus.data_bit[temp]=0; // grava o valor do registrador
				ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote de resposta com o endereço
				ModBus.txbuf[1]=5; // indica a função 15
				ModBus.txbuf[2]=ModBus.rxbuf[2]; // retorna 8 bits do dado gravado
				ModBus.txbuf[3]=ModBus.rxbuf[3]; // retorna 8 bits do dado gravado
				ModBus.txbuf[4]=ModBus.rxbuf[4];
				ModBus.txbuf[5]=ModBus.rxbuf[5];
				crc=CRC16(ModBus.txbuf,6); // calcula o crc da resposta
				ModBus.txbuf[6]=(uint8_t)(crc&0x00ff); // monta 8 bits do crc para transmitir
				ModBus.txbuf[7]=(uint8_t)(crc>>8); // monta mais 8 bits do crc para transmitir
				ModBus.txsize=8; // armazena o tamanho do pacote para transmissão
				ModBus.status = iniciandoTransmisao; // atualiza o status para o main ativar o timer
				liga_timer_modbus();
			}
			else
			{
				ModBusSendErrorMessage(5, 2); // retorna erro de endereço ilegal
			}
		}

		if(ModBus.funcao==6) // se for a função 6
		{
			temp=(uint16_t)((ModBus.rxbuf[2]<<8)|ModBus.rxbuf[3]); //recebe o endereço do registrador a ser gravado
			if(temp<num_reg_words_modbus&&temp>=0) // verifica se é válido
			{
				ModBus.data_reg[temp]=((ModBus.rxbuf[4]<<8)|ModBus.rxbuf[5]); // grava o valor do registrador
				ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote de resposta com o endereço
				ModBus.txbuf[1]=6; // indica a função 16
				ModBus.txbuf[2]=ModBus.rxbuf[2]; // retorna 8 bits do dado gravado
				ModBus.txbuf[3]=ModBus.rxbuf[3]; // retorna 8 bits do dado gravado
				ModBus.txbuf[4]=ModBus.rxbuf[4];
				ModBus.txbuf[5]=ModBus.rxbuf[5];
				crc=CRC16(ModBus.txbuf,6); // calcula o crc da resposta
				ModBus.txbuf[6]=(uint8_t)(crc&0x00ff); // monta 8 bits do crc para transmitir
				ModBus.txbuf[7]=(uint8_t)(crc>>8); // monta mais 8 bits do crc para transmitir
				ModBus.txsize=8; // armazena o tamanho do pacote para transmissão
				ModBus.status = iniciandoTransmisao; // atualiza o status para o main ativar o timer
				liga_timer_modbus();
			}
			else
			{
				ModBusSendErrorMessage(6, 2); // retorna erro de endereço ilegal
			}
		}

		if(ModBus.funcao==15) // se for a função 15
		{
			temp=(uint16_t)((ModBus.rxbuf[2]<<8)|ModBus.rxbuf[3]); //recebe o endereço do registrador a ser gravado
			num_reg=(uint16_t)((ModBus.rxbuf[4]<<8)|ModBus.rxbuf[5]); // recebe a
			if((temp+num_reg-1)<num_reg_bits_modbus&&temp>=0) // verifica se é válido
			{
				ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote de resposta com o endereço
				ModBus.txbuf[1]=15; // indica a função 15
				ModBus.txbuf[2]=ModBus.rxbuf[2]; // retorna 8 bits do dado gravado
				ModBus.txbuf[3]=ModBus.rxbuf[3]; // retorna 8 bits do dado gravado
				ModBus.txbuf[4]=ModBus.rxbuf[4]; // retorna 8 bits dnúmero de registradores gravados
				ModBus.txbuf[5]=ModBus.rxbuf[5]; // retorna 8 bits dnúmero de registradores gravados
				cont_bit=1; //inicia no primeiro bit do byte recebido
				cont_rx=7; // aponta para o primeiro byte de dados recebidos
				for(cont=0; cont<num_reg; cont++) // conta os registradores enviados
				{
					if((ModBus.rxbuf[cont_rx]&cont_bit)!=0)
					{
						ModBus.data_bit[temp]=1;
					}
					else
					{
						ModBus.data_bit[temp]=0;
					}
					cont_bit=cont_bit<<1; // aponta para o próximo bit di byte de resposta
					if(cont_bit==0x100) // testa se ja montou um byte inteiro
					{
						cont_bit=1; // aponta para o primeiro bit
						cont_rx++; // incrementa o contador de bytes
					}
					temp++;
				}
				crc=CRC16(ModBus.txbuf,6); // calcula o crc da resposta
				ModBus.txbuf[6]=(uint8_t)(crc&0x00ff); // monta 8 bits do crc para transmitir
				ModBus.txbuf[7]=(uint8_t)(crc>>8); // monta mais 8 bits do crc para transmitir
				ModBus.txsize=8; // armazena o tamanho do pacote para transmissão
				ModBus.status = iniciandoTransmisao; // atualiza o status para o main ativar o timer
				liga_timer_modbus();
			}
			else
			{
				ModBusSendErrorMessage(15, 2); // retorna erro de endereço ilegal
			}
		}

		if(ModBus.funcao==16) // se for a função 16
		{
			temp=(uint16_t)((ModBus.rxbuf[2]<<8)|ModBus.rxbuf[3]); //recebe o endereço do registrador a ser gravado
			num_reg=(uint16_t)((ModBus.rxbuf[4]<<8)|ModBus.rxbuf[5]); // recebe a quantidade de registradores a serem gravados
			if((temp+num_reg-1)<num_reg_words_modbus&&temp>=0) // verifica se é válido
			{
				ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote de resposta com o endereço
				ModBus.txbuf[1]=16; // indica a função 16
				ModBus.txbuf[2]=ModBus.rxbuf[2]; // retorna 8 bits do dado gravado
				ModBus.txbuf[3]=ModBus.rxbuf[3]; // retorna 8 bits do dado gravado
				ModBus.txbuf[4]=ModBus.rxbuf[4]; // retorna 8 bits número de registradores gravados
				ModBus.txbuf[5]=ModBus.rxbuf[5]; // retorna 8 bits número de registradores gravados
				for(cont=0; cont<num_reg; cont++) // conta os registradores enviados
				{
					ModBus.data_reg[temp]=((ModBus.rxbuf[(cont*2)+7]<<8)|ModBus.rxbuf[(cont*2)+8]);
					temp++;
				}
				crc=CRC16(ModBus.txbuf,6); // calcula o crc da resposta
				ModBus.txbuf[6]=(uint8_t)(crc&0x00ff); // monta 8 bits do crc para transmitir
				ModBus.txbuf[7]=(uint8_t)(crc>>8); // monta mais 8 bits do crc para transmitir
				ModBus.txsize=8; // armazena o tamanho do pacote para transmissão
				ModBus.status = iniciandoTransmisao; // atualiza o status para o main ativar o timer
				liga_timer_modbus();
			}
			else
			{
				ModBusSendErrorMessage(16, 2); // retorna erro de endereço ilegal
			}
		}
	}
	else // CRC inválido
	{
		ModBusReset();
	}
}
