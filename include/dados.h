#include <stdio.h>

/*Macros*/
#define tempo           0
#define distancia       1
#define raio            2
#define vel_linear      0       //vel_linear_m�dia
#define pwm_medio       1
#define acel_medio      2

/*Vari�veis externas*/
extern char flag_curva;
extern unsigned char f_millis;
extern unsigned int millisegundos; 
extern char f_parada;
extern char flag_parada;
extern unsigned char f_stop;
extern unsigned char pulse_numberR, 
                     pulse_numberL; //vari�veis para contagem dos pulsos dos encoders
extern unsigned int PWMA, PWMB;


/*Vari�veis globais desta bilioteca*/
int matriz_telemetria [100][3];   //matriz de colhimento de dados
int matriz_pista      [30][3];    //colhe somente distancia, tempo e raio
float dist_right = 0, dist_left = 0;
unsigned char f_dados_t = 0, f_dados_c = 0;         //t = telemetria, c = coleta
char buffer[5]; //String que armazena valores de entrada para serem printadas


/*Fun�oes externas*/
extern void UART_config(unsigned int ubrr); 
extern void UART_enviaCaractere(unsigned char ch);
extern void UART_enviaString(char *s);
extern void UART_enviaHex(unsigned char ch);



unsigned int valor_pwm(void)
{
    return ((PWMA + PWMB) / 2);
}

unsigned int distancia_calculo(void)
{
    static int dist = 0;

    dist_right = pulse_numberR * 0.812; //converte o n�mero de pulsos em mm
    dist_left = pulse_numberL * 0.812;
    
    pulse_numberR = 0x00;
    pulse_numberL = 0x00; //zera as vari�veis de c�lculo do raio da curva

    
    dist = (dist_right + dist_left) / 2;    //em uma reta
    
    /*Criar condi��es para reta e curva e criar fun��o de curva*/
    return dist;
}


unsigned int calculo_do_raio() //esta fun��o calcula o raio a partir da dis�ncia percorrida pelas duas rodas do rob�
{
    static int raio_f = 0;            //raio
    unsigned int diametro = 126;        //126mm, di�metro sas rodas
    
    if((dist_right - dist_left) < 1 || (dist_right - dist_left) > -1)
    {
    
        raio_f = (diametro / 2.0) * ((dist_right + dist_left) / (dist_right - dist_left)); //C�lculo do raio em m�dulo
        
        if(raio_f < 0)
        {
            raio_f *= -1;       //m�dulo
        }
    
    }
    
    else
    {
        raio_f = 0;
    }
        
        /*sprintf(buffer, "Raio %5d\n", raio_f); //Converte para string
        UART_enviaString(buffer); //Envia para o computador
        UART_enviaCaractere(0x0D); //pula linha*/
 

    return raio_f;
}


unsigned int velocid_linear()
{
    static int distancia_f = 0;
    static int velocidade  = 0;
    
    distancia_f = distancia_calculo();
    
    velocidade  = distancia_f / 0.5;     //c�lculo da velocidade em mm/s
    
    return velocidade;
}

unsigned int speed_avrg(void)
{
    return(velocid_linear() / 0.5);     //ca?culo da acelera��o em mm/s�
}


void envia_dados()
{
    if(f_dados_t)
    {
        for(int linha = 0; linha < 100; linha++)
        {
            for(int coluna = 0; coluna < 3; coluna++)
            {
                sprintf(buffer, "%d\t", matriz_telemetria[linha][coluna]);
                UART_enviaString(buffer);
            }
            UART_enviaCaractere('\n');
        }  
        f_dados_t = 0;
    }
    
    if(f_dados_c)
    {
      for(int linha = 0; linha < 30; linha++)
        {
            for(int coluna = 0; coluna < 3; coluna++)
            {
                sprintf(buffer, "%d\t", matriz_pista[linha][coluna]);
                UART_enviaString(buffer);
            }
            UART_enviaCaractere('\n');
        }  
        f_dados_c = 0;  
    }
}



void coleta_de_dados(void)
{
  
    static char f_record = 0;
    
    static unsigned int i = 0;  //vari�vel de posi��o da linha da matriz
    
    if(flag_parada)//criar uma fun��o para printar na tela
    { 
        if(flag_curva && !f_record)
        {
            matriz_pista[i][tempo]      = millisegundos;             //tempo em milissegundos
            matriz_pista[i][distancia]  = distancia_calculo();  //dist�ncia em mm
            matriz_pista[i][raio]       = calculo_do_raio();    //raio em mm 
            //sprintf(buffer, "%dms\t", millis());
            //UART_enviaString(buffer);
            f_millis = 1;                  //zerando o timer
            //sprintf(buffer, "%.2fmm\t", distancia_calculo());  //dist�ncia em mil�metros
            //UART_enviaString(buffer);
            f_dados_c = 1;
            envia_dados();
            f_record = 1;                     //flag para gravar uma vez
        }

        else if(!flag_curva && f_record)
        {
            f_record = 0;
            f_millis = 0;               //voltando o timer
        }       
        
       
        if(f_parada)
        {
            matriz_pista[i][tempo]      = millisegundos;
            matriz_pista[i][distancia]  = distancia_calculo();
            //sprintf(buffer, "%dms\t", millis());
            //UART_enviaString(buffer);
            f_millis = 1;                  //zerando o timer
            //sprintf(buffer, "%.2fmm\t", distancia_calculo());  //dist�ncia em mil�metros
            //UART_enviaString(buffer);
            flag_parada = 0;
            f_stop = 1;
        }
        
        i++;                //pulo a linha da matriz
        
    }
}


void telemetria(void)
{
    static int i = 0;
    
    matriz_telemetria[i][vel_linear]    = velocid_linear();
    matriz_telemetria[i][pwm_medio]     = valor_pwm();
    matriz_telemetria[i][acel_medio]    = speed_avrg();
    
    f_dados_t = 1;
    
    envia_dados();
}