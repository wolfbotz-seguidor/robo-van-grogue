/*Macros*/
#define PWM_RETURN      80
#define setpoint        0
#define NOP() __asm__ __volatile__ ("nop")

/*Vari�veis externas*/
extern char flag_curva;
extern char flag_parada;
extern char flag;
extern unsigned int PWMA, PWMB;

/*Fun��es externas*/
extern int PID(int error);
extern unsigned char ADC_ler ();
extern void frente();
extern void PWM_limit();
extern void setDuty_1(int duty);
extern void setDuty_2(int duty);
extern void giro_esquerda();
extern void giro_direita();

/*Vari�veis globais da biblioteca*/
//Vari�veis globais da calibra��o de sensores
/*unsigned char valor_max[5] = {0, 0, 0, 0, 0};
unsigned char valor_min[5] = {255, 255, 255, 255, 255};*/
unsigned char valor_max_abs = 255;
unsigned char valor_min_abs = 0;
unsigned char sensores_frontais[5];
unsigned char sensores_frontais_atual[5];
int erro = 0;      //vari�vel para c�culo do erro da dire��o do rob� em cima da linha

void ADC_maq () 
{
    //inicializo no setup na fun��o calibration e em seguida toda
    //vez que ocorre uma conver��o a interrup��o do AD ocorre
    //e ent�o esta fun��o � chamada pelo vetor de interrup��o
    //do AD, obtendo os dados da convers�o em "paralelo" � rotina
    
    //Leio primeiro o default que seria o primeiro canal
    //e em seguida fa�o uma l�gica circular de leitura dos canais
    
    static unsigned char estado = 10;
    
    switch (estado) {
        
        case 0:
            estado = 1;
            sensores_frontais[0] = ADC_ler();
            ADC_conv_ch(1);
            break;
            
        case 1:
            estado = 2;
            sensores_frontais[1] = ADC_ler();
            ADC_conv_ch(0);
            break;
            
        case 2:
            estado = 3;
            sensores_frontais[2] = ADC_ler();
            ADC_conv_ch(7);
            break;
            
        case 3:
            estado = 4;
            sensores_frontais[3] = ADC_ler();
            ADC_conv_ch(6);
            break;
            
        case 4:
            estado = 0;
            sensores_frontais[4] = ADC_ler();
            ADC_conv_ch(2);
            break;
            
            
        default:
            estado = 0;
            ADC_conv_ch(2);
            sensores_frontais[0] = ADC_ler();
            break; 
    }
    
}//end ADC_maq


void calibra_sensores() 
{
    //=====Fun��o que inicializa a calibra��o====//
    /*for (int i = 0; i < 120; i++) {
        for (int i = 0; i < 5; i++) {
            if (sensores_frontais[i] < valor_min [i]) {
                valor_min[i] = sensores_frontais[i];
            }
            if (sensores_frontais[i] > valor_max [i]) {
                valor_max[i] = sensores_frontais[i];
            }
        }

        //_delay_ms(20);  //tempo o suficiente para o pessoa calibrar os sensores mecanicamente
        
        
        //Ap�s isso determinar o limiar de todos os sensores para que eles tenham os mesmos valores do AD. 
        //Para que todos tenham um limite inferior e superior igual.
        
    }*/

}

void seta_calibracao() {
    //----> Calibra��o dos Sensores frontais <----//

    //fun��o que seta o limiar dos sensores
    //Este � o algoritmo a ser usado no rob�. Desmcomente antes de compilar e comente o outro.
    /*for (int i = 0; i < 5; i++) {
        if (valor_min [i] > valor_min_abs && valor_min[i] !=0 ) //esse !0 foi colocado pois estava havendo um bug ao simular
        {
            valor_min_abs = valor_min [i];
        } 
        
        if (valor_max [i] < valor_max_abs) {
            valor_max_abs = valor_max [i];
        }
        

    }*/
    valor_min_abs = 100; //valores vistos pelo monitor serial
    valor_max_abs = 200;
}

void sensores() 
{

    //======Estabelece o limiar da leitura dos sensores====//
    //fun��o de corre��o da calibra��o
    for (int i = 0; i < 5; i++)
    {
        if (sensores_frontais[i] < valor_min_abs) 
        {
            sensores_frontais_atual[i] = valor_min_abs;
        }
        if (sensores_frontais[i] > valor_max_abs)
        {
            sensores_frontais_atual[i] = valor_max_abs;
        }

    }
    
}

void le_marcadores(void)
{
    //cruzamento
    //branco = 0, preto = 1
    static unsigned char flag_count = 0;
    static unsigned char s_curva = 0, s_parada = 0;
    
    s_curva =  tst_bit(leitura_curva, sensor_de_curva);     //l� valor do sensor de curva
    
    for(int i = 0; i < 16; i++)    //62,5ns cada NOP, 62,5*16 = 1000ns = 1us
    {
        NOP();
    }
    
    s_parada = tst_bit(leitura_parada, sensor_de_parada);   //l� valor do sensor de parada
    //leitura de marcador de parada
    
    if ((s_curva) && (!s_parada) && !flag_count)
    {
        flag = 1;
        flag_count = 1;
        flag_parada = 1;
        flag_curva = 0;
        set_bit(PORTB, PB5);
    }

    else if ((!s_curva) && (!s_parada)) //verifica se � crizamento
    {
        flag = 0;
        flag_count = 1;
        flag_curva = 0;
        clr_bit(PORTB, PB5);
    }

    else if ((s_curva) && (s_parada))
    {
        flag = 0;
        flag_count = 0;
        flag_curva = 0;
        clr_bit(PORTB, PB5);
    }
    else if (!(s_curva) && (s_parada) && !flag_curva)
    {
        flag = 0;
        flag_count = 0;
        flag_curva = 1;
        clr_bit(PORTB, PB5);
    }
}

void sentido_de_giro()
{
    //-----> �rea do senstido de giro       
    static int u = 0;
    static unsigned int PWMR = 140; // valor da for�a do motor em linha reta
    static unsigned int PWM_Curva = 120; //PWM ao entrar na curva
    
    
    if ((sensores_frontais_atual[0] < 101 && sensores_frontais_atual[4] > 190) || (sensores_frontais_atual[0]  > 190 && sensores_frontais_atual[4] < 101))    
        //Valores vistos na serial
        //se o primeiro sensor ou o �ltimo sensor estiverem lendo branco...
        //necess�rio teste com monitor serial
        //estudar a melhor quantidade de sensores e seu espa�amento
    {
        u = PID(erro); //valor de retorno do PID (rotacional)
        PWMA = PWM_Curva - u;
        PWMB = PWM_Curva + u;
        frente();
        PWM_limit();
        setDuty_1(PWMA);
        setDuty_2(PWMB);
    } //em cima da linha
        
    else
    { 
        //pra frente - reta
        //--------------->AREA DO PID<---------------
        u = PID(erro); //valor de retorno do PID (rotacional)
        PWMA = PWMR - u;
        PWMB = PWMR + u;
        frente();
        PWM_limit();
        setDuty_1(PWMA);
        setDuty_2(PWMB);
    }
    
    
    //sprintf(buffer, "%d\n", u);
    //UART_enviaString(buffer);

    
    //sprintf(buffer, "%d\t%d\n", PWMA, PWMB);
    //UART_enviaString(buffer);
}


void volta_pra_pista(void)
{    
    /*if ((sensores_frontais[1] < 101) && (sensores_frontais[3] > 190))//curva � esquerda
    {
      if (sensores_frontais[2] > 107)
      {
          
        giro_esquerda();
        setDuty_1(PWMA);
        setDuty_2(PWMB);

      }
    }
    
    else if ((sensores_frontais[3] < 101) && (sensores_frontais[1] > 190))//curva � direta
    {
      if (sensores_frontais[2] > 107)
      {
        
        giro_direita();
        setDuty_1(PWMA);
        setDuty_2(PWMB);
      }  
    }*/
    
    if ((sensores_frontais_atual[4] > 190) && (sensores_frontais_atual[2] > 190))//saindo da pista, curva � esquerda
    {
        giro_esquerda();
        setDuty_1(PWM_RETURN);    //utilizar vari�vel fixa / define
        setDuty_2(PWM_RETURN); 
        
    }
    
    else if((sensores_frontais_atual[0] > 190) && sensores_frontais_atual[2] > 190)
    {
        giro_direita();
        setDuty_1(PWM_RETURN);
        setDuty_2(PWM_RETURN); 
    }
    
        
    /*Fim de �rea para voltar para a pista*/
    //Obs.: Os valores mudam de acordo com o N� de sens. e suas posi��es
    //bem como a calibra��o dos mesmos.
}


void volta_pra_pista_calibracao(void)/*A ser usado em uma calib. auto.*/
{    
    if ((sensores_frontais_atual[4] < 200) && (sensores_frontais_atual[0] < 200))
    {
            while (sensores_frontais_atual[1] < 101 && sensores_frontais_atual[2] < 120)
            {
        
                giro_esquerda();
                setDuty_1(PWMA);
                setDuty_2(PWMB);

            }  
    }
    
    else if ((sensores_frontais_atual[4] < 200) && (sensores_frontais_atual[0] < 200))
    {
            while (sensores_frontais_atual[3] < 100 && sensores_frontais_atual[2] < 99)
            {
        
                giro_direita();
                setDuty_1(PWMA);
                setDuty_2(PWMB);

            }  
    }
}


void calculo_do_erro()
{
    static int denominador = 6;
    int soma_total = 0;   //caso aumente o peso da m�dia_ponderada, tomar cuidado com a vari�vel char
    
    static int peso [] = {-2, -1, 0, 1, 2};
    //os pesos precisar�o ser corrigidos pois os sensores do Van Grogue est�o um pouco assim�tricos
    
    for (int j = 0; j < 4; j++) 
    {
        soma_total += (sensores_frontais_atual[j] * peso[j]);
    }

    soma_total /= denominador;
    
    erro = setpoint - soma_total;   //valor esperado(estar sempre em cima da linha) - valor medido

    
    /*if(erro > 33)   //corrigindo assimetria(tentando)
    {
        erro = 33;
    }
    
    if(erro < -33)
    {
        erro = -33;
    }*/
    
    /*for(int i = 0; i < 5; i++)
    {
        sprintf(buffer, "%d\t", sensores_frontais[i]);
        UART_enviaString(buffer);
    }
    UART_enviaCaractere('\n');*/
    
    //sprintf(buffer, "%d\n", erro);
    //UART_enviaString(buffer);
    
}



