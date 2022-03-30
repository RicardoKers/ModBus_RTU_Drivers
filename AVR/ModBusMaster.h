/*
 * ModBusMaster.h
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

// Configuração ModBus
#define endereco_modbus 1 // endereço inicial do escravo
#define tam_buff_recep 255
#define tam_buff_trans 255
#define modBusTimeout_ms 1000
// final Configuração ModBus

struct mb
{
	enum e_status {inativo, transmitindo, aguardandoResposta, respostaRecebida, semResposta} status;
	enum e_erro {semErro, timeout, respostaInvalida} erro;
	uint8_t end_modbus; // armazena o endereço na modbus
	uint16_t rxsize; // tamanho do pacote na recepção
	uint16_t txsize; // tamanho do pacote na transmissão
	uint8_t rxbuf[tam_buff_recep]; // buffer de recepção
	uint8_t txbuf[tam_buff_trans]; // buffer de transmissão
	uint8_t funcao;
	uint16_t *data_reg_addr; // ponteiro para os dados de word recebidos pela modbus
	uint8_t *data_bit_addr; // ponteiro para os dados de bit recebidos pela modbus
	uint16_t numRegs; // armazena o número de registradores da operação
	uint16_t rxpt; // ponteiro para o buffer de recepçao, necessario em algumas arquiteturas
	uint16_t txpt; // ponteiro para o buffer de transmissão, necessario em algumas arquiteturas
} ModBus;

// definições
#define modBusTimeoutCounterStart() modBusTimeoutCounter=0
#define modBusTimeoutCounterStop() modBusTimeoutCounter=modBusTimeout_ms+1

// variáveis globais
unsigned int modBusTimeoutCounter=modBusTimeout_ms+1;

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

void ModBusReset() // prepara para a próxima transmissão
{
	ModBus.end_modbus=endereco_modbus;
	ModBus.status=inativo;
	ModBus.erro=semErro;	
}

void modBusReadCoilStatusFC01(uint16_t endInicial, uint16_t numRegistradores, uint8_t *dataAddr)
{
	uint16_t crc;
	if(ModBus.status==inativo) // está pronto para transmitir
	{
		ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote com o endereço do slave
		ModBus.txbuf[1]=1; // indica a função 1
		ModBus.txbuf[2]=(uint8_t)(endInicial>>8);
		ModBus.txbuf[3]=(uint8_t)(endInicial);
		ModBus.txbuf[4]=(uint8_t)(numRegistradores>>8);
		ModBus.txbuf[5]=(uint8_t)(numRegistradores);
		crc=CRC16(ModBus.txbuf,(uint16_t)6); // calcula o crc
		ModBus.txbuf[6]=(uint8_t)(crc);
		ModBus.txbuf[7]=(uint8_t)(crc>>8);
		ModBus.txsize=8;
		ModBus.status=transmitindo;
		ModBus.funcao=1; // para ajustar a recepção
		ModBus.data_bit_addr=dataAddr; // armazena o ponteiro para a recepção
		ModBus.numRegs=numRegistradores; // armazena o número de registradores para a recepção
		ModBus.rxsize=5+(numRegistradores/8); // determina o número de bytes na resposta para preparar a recepção
		if((numRegistradores%8)!=0) ModBus.rxsize++; // soma um se não completou 8 bits de recepção
		ModBus.txpt=1; // atualiza o ponteiro de transmissção
		iniciaTransmissao(ModBus.txbuf[0]);
	}
}

void modBusReadHoldingRegistersFC03(uint16_t endInicial, uint16_t numRegistradores, uint16_t *dataAddr)
{
	uint16_t crc;
	if(ModBus.status==inativo) // está pronto para transmitir
	{
		ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote com o endereço do slave
		ModBus.txbuf[1]=3; // indica a função 3
		ModBus.txbuf[2]=(uint8_t)(endInicial>>8);
		ModBus.txbuf[3]=(uint8_t)(endInicial);
		ModBus.txbuf[4]=(uint8_t)(numRegistradores>>8);
		ModBus.txbuf[5]=(uint8_t)(numRegistradores);
		crc=CRC16(ModBus.txbuf,(uint16_t)6); // calcula o crc
		ModBus.txbuf[6]=(uint8_t)(crc);
		ModBus.txbuf[7]=(uint8_t)(crc>>8);
		ModBus.txsize=8;
		ModBus.status=transmitindo;
		ModBus.funcao=3; // para ajustar a recepção
		ModBus.data_reg_addr=dataAddr; // armazena o ponteiro para a recepção
		ModBus.numRegs=numRegistradores; // armazena o número de registradores para a recepção
		ModBus.rxsize=5+(numRegistradores*2); // determina o número de bytes na resposta para preparar a recepção
		ModBus.txpt=1; // atualiza o ponteiro de transmissção
		iniciaTransmissao(ModBus.txbuf[0]);
	}
}

void modBusForceSingleCoilFC05(uint16_t end, uint8_t valor)
{
	uint16_t crc;
	if(ModBus.status==inativo) // está pronto para transmitir
	{
		ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote com o endereço do slave
		ModBus.txbuf[1]=5; // indica a função 5
		ModBus.txbuf[2]=(uint8_t)(end>>8);
		ModBus.txbuf[3]=(uint8_t)(end);
		if(valor==1)
		{
			ModBus.txbuf[4]=0xFF;
			ModBus.txbuf[5]=0;
		}
		else
		{
			ModBus.txbuf[4]=0;
			ModBus.txbuf[5]=0;
		}
		crc=CRC16(ModBus.txbuf,(uint16_t)6); // calcula o crc
		ModBus.txbuf[6]=(uint8_t)(crc);
		ModBus.txbuf[7]=(uint8_t)(crc>>8);
		ModBus.txsize=8;
		ModBus.status=transmitindo;
		ModBus.funcao=5; // para ajustar a recepção
		ModBus.rxsize=8; // determina o número de bytes na resposta para preparar a recepção
		ModBus.txpt=1; // atualiza o ponteiro de transmissção
		iniciaTransmissao(ModBus.txbuf[0]);
	}
}

void modBusPresetSingleRegisterFC06(uint16_t end, uint16_t valor)
{
	uint16_t crc;
	if(ModBus.status==inativo) // está pronto para transmitir
	{
		ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote com o endereço do slave
		ModBus.txbuf[1]=6; // indica a função 6
		ModBus.txbuf[2]=(uint8_t)(end>>8);
		ModBus.txbuf[3]=(uint8_t)(end);
		ModBus.txbuf[4]=(uint8_t)(valor>>8);
		ModBus.txbuf[5]=(uint8_t)(valor);
		crc=CRC16(ModBus.txbuf,(uint16_t)6); // calcula o crc
		ModBus.txbuf[6]=(uint8_t)(crc);
		ModBus.txbuf[7]=(uint8_t)(crc>>8);
		ModBus.txsize=8;
		ModBus.status=transmitindo;
		ModBus.funcao=6; // para ajustar a recepção
		ModBus.rxsize=8; // determina o número de bytes na resposta para preparar a recepção
		ModBus.txpt=1; // atualiza o ponteiro de transmissção
		iniciaTransmissao(ModBus.txbuf[0]);
	}
}

void modBusForceMultipleCoilsFC15(uint16_t endInicial, uint16_t numRegistradores, uint8_t *dataAddr)
{
	uint16_t temp; // variável para valores temporários
	uint16_t cont_bit; // conta os bits
	uint16_t cont;
	uint16_t crc;
	if(ModBus.status==inativo) // está pronto para transmitir
	{
		ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote com o endereço do slave
		ModBus.txbuf[1]=15; // indica a função 15
		ModBus.txbuf[2]=(uint8_t)(endInicial>>8);
		ModBus.txbuf[3]=(uint8_t)(endInicial);
		ModBus.txbuf[4]=(uint8_t)(numRegistradores>>8);
		ModBus.txbuf[5]=(uint8_t)(numRegistradores);
		ModBus.txbuf[6]=(numRegistradores/8); // determina o número de bytes na resposta para preparar a recepção
		if((numRegistradores%8)!=0) ModBus.txbuf[6]++; // soma um se não completou 8 bits de recepção
		temp=7;// primeiro byte de dados do pacote
		cont_bit=1; //determina a posição do bit no byte
		ModBus.txbuf[7]=0; //zera o byte para depois inserir os uns
		for(cont=0; cont<numRegistradores; cont++) // conta os bits recebidos
		{
			if(dataAddr[cont]==1) // bit a ser enviado é 1
			{
				ModBus.txbuf[temp]=ModBus.txbuf[temp]|cont_bit; // liga o bit
			}
			cont_bit=cont_bit<<1;
			if(cont_bit==256) //fim do byte
			{
				temp++; // aponta o proximo byte de dados do pacote
				cont_bit=1; // recomeça no primeiro bit
				ModBus.txbuf[temp]=0;
			}
		}
		crc=CRC16(ModBus.txbuf,(uint16_t)temp+1); // calcula o crc
		temp++;
		ModBus.txbuf[temp]=(uint8_t)(crc);
		temp++;
		ModBus.txbuf[temp]=(uint8_t)(crc>>8);
		ModBus.txsize=temp+1;
		ModBus.status=transmitindo;
		ModBus.funcao=15; // para ajustar a recepção
		ModBus.rxsize=8; // determina o número de bytes na resposta para preparar a recepção
		ModBus.txpt=1; // atualiza o ponteiro de transmissção
		iniciaTransmissao(ModBus.txbuf[0]);
	}
}

void modBusPresetMultipleRegistersFC16(uint16_t endInicial, uint16_t numRegistradores, uint16_t *dataAddr)
{
	uint16_t crc;
	uint16_t temp; // variável para valores temporários
	uint16_t cont;
	if(ModBus.status==inativo) // está pronto para transmitir
	{
		ModBus.txbuf[0]=ModBus.end_modbus; // inicia o pacote com o endereço do slave
		ModBus.txbuf[1]=16; // indica a função 16
		ModBus.txbuf[2]=(uint8_t)(endInicial>>8);
		ModBus.txbuf[3]=(uint8_t)(endInicial);
		ModBus.txbuf[4]=(uint8_t)(numRegistradores>>8);
		ModBus.txbuf[5]=(uint8_t)(numRegistradores);
		ModBus.txbuf[6]=(uint8_t)(numRegistradores*2);
		temp=7;
		for(cont=0; cont<numRegistradores; cont++)
		{
			ModBus.txbuf[temp]=(uint8_t)(dataAddr[cont]>>8);
			temp++;
			ModBus.txbuf[temp]=(uint8_t)(dataAddr[cont]);
			temp++;
		}
		crc=CRC16(ModBus.txbuf,(uint16_t)temp); // calcula o crc
		ModBus.txbuf[temp]=(uint8_t)(crc);
		temp++;
		ModBus.txbuf[temp]=(uint8_t)(crc>>8);
		ModBus.txsize=temp+1;
		ModBus.status=transmitindo;
		ModBus.funcao=16; // para ajustar a recepção
		ModBus.rxsize=8; // determina o número de bytes na resposta para preparar a recepção
		ModBus.txpt=1; // atualiza o ponteiro de transmissção
		iniciaTransmissao(ModBus.txbuf[0]);
	}
}

void ModBusProcess()
{
	uint16_t crc; // armazena o valor do crc do pacote
	uint16_t temp; // variável para valores temporários
	uint16_t cont_bit; // conta o bit nas recepções de bits
	uint16_t cont; // variável para contar os registradores recebidos

	ModBus.erro=respostaInvalida;
	crc=CRC16(ModBus.rxbuf,ModBus.rxsize-1); // calcula o crc do pacote
	if((uint8_t)(crc&0x00ff)==ModBus.rxbuf[ModBus.rxsize-1]&&(uint8_t)(crc>>8)==ModBus.rxbuf[ModBus.rxsize]) // testa se o crc é válido
	{
		if(ModBus.funcao==1) // se for a função 1
		{
			if(ModBus.rxbuf[0]!=ModBus.end_modbus)// erro: endereço inválido
			{
				return;
			}
			if(ModBus.rxbuf[1]!=1)// erro: função inválida
			{
				return;
			}
			if(ModBus.rxbuf[2]!=(ModBus.rxsize-5))// erro: contador de bytes incorreto
			{
				return;
			}
			// tudo certo, armazenando dados lidos
			temp=3;// primeiro byte de dados do pacote
			cont_bit=1; //determina a posição do bit no byte
			for(cont=0; cont<ModBus.numRegs; cont++) // conta os bits recebidos
			{
				if((ModBus.rxbuf[temp]&cont_bit)!=0) // bit recebido é 1
				{
					ModBus.data_bit_addr[cont]=1;
				}
				else // bit recebido é 1
				{
					ModBus.data_bit_addr[cont]=0;
				}
				cont_bit=cont_bit<<1;
				if(cont_bit==256) //fim do byte
				{
					temp++; // aponta o proximo byte de dados do pacote
					cont_bit=1; // recomeça no primeiro bit
				}
			}
			ModBus.erro=semErro;
			return;
		}
		if(ModBus.funcao==3) // se for a função 3
		{
			if(ModBus.rxbuf[0]!=ModBus.end_modbus)// erro: endereço inválido
			{
				return;
			}
			if(ModBus.rxbuf[1]!=3)// erro: função inválida
			{
				return;
			}
			if(ModBus.rxbuf[2]!=(ModBus.numRegs*2))// erro: contador de bytes incorreto
			{
				return;
			}
			// tudo certo, armazenando dados lidos
			temp=3;// primeiro byte de dados do pacote
			for(cont=0; cont<ModBus.numRegs; cont++) // conta os bits recebidos
			{
				ModBus.data_reg_addr[cont]=(ModBus.rxbuf[temp+1]|(ModBus.rxbuf[temp]<<8));
				temp=temp+2; // aponta o proximo dado no pacote
			}
			ModBus.erro=semErro;
			return;
		}
		if(ModBus.funcao==5) // se for a função 5
		{
			if(ModBus.rxbuf[0]!=ModBus.end_modbus)// erro: endereço inválido
			{
				return;
			}
			if(ModBus.rxbuf[1]!=5)// erro: função inválida
			{
				return;
			}
			if(ModBus.rxbuf[2]!=ModBus.txbuf[2]||ModBus.rxbuf[3]!=ModBus.txbuf[3])// erro: endereço incorreto
			{
				return;
			}
			if(ModBus.rxbuf[4]!=ModBus.txbuf[4]||ModBus.rxbuf[5]!=ModBus.txbuf[5])// erro: dado incorreto
			{
				return;
			}
			// tudo certo
			ModBus.erro=semErro;
			return;
		}
		if(ModBus.funcao==6) // se for a função 6
		{
			if(ModBus.rxbuf[0]!=ModBus.end_modbus)// erro: endereço inválido
			{
				return;
			}
			if(ModBus.rxbuf[1]!=6)// erro: função inválida
			{
				return;
			}
			if(ModBus.rxbuf[2]!=ModBus.txbuf[2]||ModBus.rxbuf[3]!=ModBus.txbuf[3])// erro: endereço incorreto
			{
				return;
			}
			if(ModBus.rxbuf[4]!=ModBus.txbuf[4]||ModBus.rxbuf[5]!=ModBus.txbuf[5])// erro: dado incorreto
			{
				return;
			}
			// tudo certo
			ModBus.erro=semErro;
			return;
		}
		if(ModBus.funcao==15) // se for a função 15
		{
			if(ModBus.rxbuf[0]!=ModBus.end_modbus)// erro: endereço inválido
			{
				return;
			}
			if(ModBus.rxbuf[1]!=15)// erro: função inválida
			{
				return;
			}
			if(ModBus.rxbuf[2]!=ModBus.txbuf[2]||ModBus.rxbuf[3]!=ModBus.txbuf[3])// erro: endereço incorreto
			{
				return;
			}
			if(ModBus.rxbuf[4]!=ModBus.txbuf[4]||ModBus.rxbuf[5]!=ModBus.txbuf[5])// erro: dado incorreto
			{
				return;
			}
			// tudo certo
			ModBus.erro=semErro;
			return;
		}
		if(ModBus.funcao==16) // se for a função 16
		{
			if(ModBus.rxbuf[0]!=ModBus.end_modbus)// erro: endereço inválido
			{
				return;
			}
			if(ModBus.rxbuf[1]!=16)// erro: função inválida
			{
				return;
			}
			if(ModBus.rxbuf[2]!=ModBus.txbuf[2]||ModBus.rxbuf[3]!=ModBus.txbuf[3])// erro: endereço incorreto
			{
				return;
			}
			if(ModBus.rxbuf[4]!=ModBus.txbuf[4]||ModBus.rxbuf[5]!=ModBus.txbuf[5])// erro: dado incorreto
			{
				return;
			}
			// tudo certo
			ModBus.erro=semErro;
			return;
		}
	}
}

void ModBusTimeout() // atingiu o tempo máximo para responder
{
	ModBus.status=semResposta;
	ModBus.erro=timeout;
}