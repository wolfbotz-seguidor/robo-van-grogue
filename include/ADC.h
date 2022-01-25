/*---------------------------------------------------------------
 * BIBLIOTECA PARA UTILIZACAO DO CONVERSOR AD DO AVR
 * Modificada por: PROF. Andr� Costa Canella
 * 08/2021
 * -----------------------------------------------------------------*/

void ADC_init (void) {
    //FADC = 1MHz
    //Tadc = 1/1MHz = 1us
    //Primeira Convers�o = 25*1 = 25us
    //Demais convers�es = 14*1 = 14us
    //Tempo total do prieiro ciclo = (25*1) + (14*1*5) = 95us
    //Tempo das demais convers�es = 14*1*6 = 84us
    //Utilizando teoria de amostragem -> 168->190us

    
    ADMUX = 0x60; //0110-0000   //Refer�ncia no AVCC, deslocado a direita
    ADCSRA = 0x8c; //1000-1100  //ADC habilitado, interrup��o do AD habilitado e prescaler de 16 - 1MHz
    ADCSRB = 0x00; // 0000-0000 //Modo livre
    DIDR0 = 0x3f;// 0011-1111   //Desabilita a entrada digital desses terminais
 
    //Aten��o. Segundo o datasheet quanto maior a velocidade,
    //menor a precis�o do AD, logo, utilizar 8bits em freq. elevadas
}

void ADC_conv_ch (unsigned char canal) {
    
    ADMUX &= 0xf0;
    ADMUX |= (canal & 0x0f);
    
    ADCSRA |= 0x40;
            
}

unsigned char ADC_ler ()
{
    
    unsigned char dado = ADCH;
    return dado;
}