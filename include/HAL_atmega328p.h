/*Bibliotecas e frequ�ncia do uc*/
#define F_CPU 16000000        //define a frequencia do uC para 16MHz
#include <avr/io.h>           //Biblioteca geral dos AVR
#include <avr/interrupt.h>    //Biblioteca de interrup��o
#include <util/delay.h>

//vari�veis de comando para os registradores
#define set_bit(y,bit) (y|=(1<<bit)) //coloca em 1 o bit x da vari�vel Y
#define clr_bit(y,bit) (y&=~(1<<bit)) //coloca em 0 o bit x da vari�vel Y
#define cpl_bit(y,bit) (y^=(1<<bit)) //troca o estado l�gico do bit x da vari�vel Y
#define tst_bit(y,bit) (y&(1<<bit)) //retorna 0 ou 1 conforme leitura do bit
/*==============================================================*/

/*Mapeamento de Hardware*/
#define sensor_de_curva   PB0
#define sensor_de_parada  PD7
#define led               PB5
#define leitura_curva    PINB
#define leitura_parada   PIND

extern void setFreq(char option);
extern void ADC_init();
void INT_INIT();
extern void UART_config(unsigned int ubrr);

/*tempo =65536 * Prescaler/Fosc = 65536 * 1024/16000000 = 4, 19s
 tempo = X_bit_timer * Prescaler/Fosc
 Valor inicial de contagem = 256 - tempo_desejado*Fosc/Prescaler = 256 - 0,001*16000000/1024 = 255
 Valor inicial de contagem = X_bit_timer - tempo_desejado*Fosc/Prescaler */
/*===========================================================================*/


void setup_Hardware()
{
    MCUCR &= 0xef;      //habilita pull up quando configurado e desabilita algumas configura��es pr�vias do MCU

    DDRD = 0b01111010; //PD3 - PD6 definidos como sa�da, PD7 como entrada, PD0 como entrada(RX) e PD1 como sa�da(TX)
    PORTD = 0b10000000; //inicializados em n�vel baixo e PD7 com pull up
    DDRB = 0b00100110; //Habilita PB0 como entrada e PB5, PB1 e PB2 como sa�da
    PORTB = 0b00000001; //PORTB inicializa desligado e pull up no PB0
    DDRC = 0b00000000; //PORTC como entrada
    PORTC = 0b00001111; //PC3 - PC0 com pull up (colocar resistor de pull up nos pinos A6 e A7)
    TCCR1A = 0xA2; //Configura opera��o em fast PWM, utilizando registradores OCR1x para compara��o
    //TCCR1A = 0xA3 //Configura em phase correct PWM OC1A e OC1B limpos
    
    //esquerdo pino 4 - PD2
    UART_config(16); //Inicializa a comunica��o UART com 57.6kbps
    
    setFreq(4); //Seleciona op��o para frequ�ncia
    //62,5Hz de PWM
    
    ADC_init(); //Inicializa o AD
    INT_INIT(); //Inicializa o Timer0

}//end setup_hardware

void INT_INIT()
{    //f�rmula nas primeiras linhas do c�digo
    TCCR0B = 0b00000010; //TC0 com prescaler de 8
    TCNT0 = 56; //Inicia a contagem em 56 para, no final, gerar 100us
    TIMSK0 = 0b00000001; //habilita a interrup��o do TC0
}