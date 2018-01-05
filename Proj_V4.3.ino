#include "FreeRTOS_AVR.h"
#include "basic_io_avr.h"
#include <Edubot.h>
#include <LCD_Mega.h>
#include <SPI.h>
//---------------------------------------------------------
//#define TURN_DIR true// true --> turn always right
// false --> turn always left
//---------------------------------------------------------
#define ECHOPIN A6 // Pin recebe ECHO
#define TRIGPIN A7 // Pin envia TRIGGER
#define R_LED 22
#define L_LED 23
#define DIR_BUTTON 24
#define VEL_MAX 85  // Constante da velocidade base
#define BRAIN_DELAY 44 // Delay until lock time
#define LCD_SEM_LOCK_TIME 300000 // semaphore take lock time(5 min)
#define MOTORS_SEM_LOCK_TIME 100000 // semaphore take lock time(100 ms)
#define SENSORS_SEM_LOCK_TIME 100000 // semaphore take lock time(100ms)


#define R 1
#define L 2
#define B 3
#define S 4
//LEFT
#define LBR 231
#define LBS 234
#define RBL 132
#define SBL 432
#define SBS 434
#define LBL 232
//RIGHT
#define RBS 134
#define SBR 431
#define RBR 131
//-------------
//lcd constants
#define G 5
//volatile unsigned long ulIdleCycleCount = 0UL;


extern "C" { // FreeRTOS expects C linkage
  void vApplicationIdleHook( void )
  {
    /* This hook function does nothing but increment a counter. */
    //vPrint1String("[IDLE]\n\r");
    //ulIdleCycleCount++;
  }
}


Edubot robot; //declara uma variavel robot
LCD_Mega lcd; //declara uma variavel lcd

static void Brain_task( void *pvParameters );
static void Sensors_task( void *pvParameters );
static void Motors_task( void *pvParameters );
static void LCD_task( void *pvParameters );
static void Config_task( void *pvParameters );

static void InterruptSonar( void );


QueueHandle_t ConfQueue, SensorsQueue, MotorsQueue, LCDQueue; // Declara Queue
const uint8_t IntPinSonar = 21; // Define pinos interrupção

SemaphoreHandle_t sem_sensors;
SemaphoreHandle_t sem_motors;
SemaphoreHandle_t sem_lcd;


TaskHandle_t xConfigHandle;


/*------------------------------------------- => SETUP <= ---------------------------------------*/
void setup( void )
{
  Serial.begin(9600); //Abre porta de comunicação serie com baudrate de 9600 bit/s
  Serial1.begin(9600); //Abre porta de comunicação serie com baudrate de 9600 bit/s
  //todo: fazer crusamentos mais suaves para evitar desalinhamentos
  vPrint1String("DEBUG PORT\n\r");
  lcd.begin();
  lcd.LcdClear(); //Limpa o LCD
  delay(100); //Delay pedido no datasheet do LCD
  robot.Setup(650);
  pinMode(ECHOPIN, INPUT); //define-se o pino de receção dos ultra-sons
  pinMode(TRIGPIN, OUTPUT); //define-se o pino de emição dos ultra-sons
  pinMode(R_LED, OUTPUT);
  pinMode(L_LED, OUTPUT);
  pinMode(DIR_BUTTON, INPUT);
  SensorsQueue = xQueueCreate( 1, sizeof( int ) * 4); //Cria-se uma Queue de tamanho 1 com vetor de 4 inteiros
  MotorsQueue = xQueueCreate( 1, sizeof( int ) * 2); //Cria-se uma Queue de tamanho 1 com vetor de 2 inteiros
  ConfQueue = xQueueCreate( 1, sizeof( bool )); //Cria-se uma Queue de tamanho 1
  LCDQueue = xQueueCreate( 1, sizeof( char )); //Cria-se uma Queue de tamanho 1
  pinMode(IntPinSonar, OUTPUT); //Define o pino da interrupção como saída
  attachInterrupt(2, InterruptSonar, RISING); // Define as interrupções, escolhe a interrupção, e o flanco ascendente
  
  lcd.gotoXY(0, 0);
  lcd.LcdString(" [SCE]");
  lcd.gotoXY(0, 1);
  lcd.LcdString(" Trabalho");
  lcd.gotoXY(0, 2);
  lcd.LcdString(" Realizado");
  lcd.gotoXY(0, 3);
  lcd.LcdString(" Por :");
  lcd.gotoXY(0, 4);
  lcd.LcdString(" Miguel Santos");
  lcd.gotoXY(0, 5);
  lcd.LcdString(" Jose Rosa");


  vSemaphoreCreateBinary( sem_sensors );
  vSemaphoreCreateBinary( sem_motors );
  vSemaphoreCreateBinary( sem_lcd );

  // Cria as tasks
  if (sem_sensors != NULL && sem_motors != NULL && sem_lcd != NULL ) {

    xTaskCreate( Brain_task, "Brain_task", 500, NULL, 5, NULL );
    xTaskCreate( Sensors_task, "Sensors_task", 200, NULL, 4, NULL );
    xTaskCreate( Motors_task, "Motors_task", 200, NULL, 3, NULL);
    xTaskCreate( LCD_task, "LCD_task", 200, NULL, 2, NULL );
    xTaskCreate( Config_task, "Config_task", 100, NULL, 1, &xConfigHandle);

    //Serial.println(robot.sensor_thresh);
    while (digitalRead(robot.SWITCH1)) {}; // Espera botão a '1'
    while (!digitalRead(robot.SWITCH1)) {}; // Espera botão a '0'
    lcd.gotoXY(0, 0);
    lcd.LcdClear(); //Limpa o LCD
    vTaskStartScheduler();
  }

  for ( ;; );
}

/*----------------------------------------- => BRAIN <= -------------------------------------*/
static void Brain_task( void *pvParameters ) {
  int Velocidade = VEL_MAX;
  int RecebeInfra [4] ;

  int EnviaRodas [2] = {Velocidade, Velocidade};
  char optimal_path [60];
  char index_optimal = 0;
  char size_path = 0;
  char send_lcd = 0;

  int last3 = 0;
  char white_cycle = 0;
  char black_cycle = 0;
  char ignore_cycle = 0;
  char ignore_go = 0;
  //char * direction_flag = "STR";
  bool RECT_CURVE_R  = false;
  bool RECT_CURVE_L = false ;
  bool IGNORE = false ;
  bool CROSSING = false ;
  bool PATH_FOUND = false ;
  bool GOAL = false ;
  bool preventBTL = false;
  bool ignoreGO = false;
  bool GOING_BACK = false;
  bool UPDATE_INDEX = false;
  bool TURN_DIR = false;

  vTaskPrioritySet( xConfigHandle, 7 );

  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  //recebe o sentido para o qual vira quando ainda não sabe o caminho otimizado
  if (xQueueReceive(ConfQueue, &TURN_DIR, 0) != pdPASS) {
    vPrint1String( "[Brain] -> Could not receive from -> [Conf].\n\r" );
    TURN_DIR = true;
  }

  if (TURN_DIR) {
    digitalWrite(R_LED, HIGH);
    digitalWrite(L_LED, LOW);
  } else {
    digitalWrite(L_LED, HIGH);
    digitalWrite(R_LED, LOW);
  }

  portBASE_TYPE EstadoRecebeInfra;
  portBASE_TYPE Estado_Envio_Motors;
  portBASE_TYPE Estado_Envio_LCD;

  xSemaphoreTake( sem_sensors, 0);
  xSemaphoreTake( sem_motors, 0);
  xSemaphoreTake( sem_lcd, 0);
  for ( ;; )
  {
    //vPrint1StringAndNumber( "IDLE:", ulIdleCycleCount );
    if (GOAL) {
      GOAL = false;
      CROSSING = false;
      GOING_BACK = true;
      white_cycle = 0;
      //black_cycle = 0;
      vPrint1String("meta\n\r");
      RECT_CURVE_L = false;
      RECT_CURVE_R = false;
      if (!PATH_FOUND) {
        PATH_FOUND = true;
        index_optimal--;
        size_path = index_optimal;
        //vPrint1StringAndNumber("meta:", size_path);
        char i;
        for (i = 0; i <= index_optimal; i++) {
          vPrint1StringAndNumber("", optimal_path[i]);
        }
      }

      //xQueueReceive(SensorsQueue, &RecebeInfra, 0);//removes the last value ([1 1 1 1])
    }


    //vPrint1String( "[Brain]\n\r" );
    xSemaphoreGive( sem_sensors);// gives a semaphore to alow the sensors and motors task to run once when the brain tasck is in blocked state
    xSemaphoreGive( sem_motors);

    EstadoRecebeInfra = xQueueReceive(SensorsQueue, &RecebeInfra, (50 / portTICK_PERIOD_MS)); //reads the value of the sensors from the queue
    if (EstadoRecebeInfra != pdPASS) {
      vPrint1String( "[CEREBRO] -> Could not receive from -> [INFRA-VERMELHOS].\n\r" );
      xSemaphoreTake( sem_motors, 0);
    } else {//executes the main algorithm only if something was read from the queue
      // [DIR MDIR MESQ ESQ]  => '1' means black; '0' means white
      // [0 1 1 0]
      if ((!robot.chkLine(RecebeInfra[0]) && robot.chkLine(RecebeInfra[1]) && robot.chkLine(RecebeInfra[2]) && !robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0)) {
        white_cycle = 0;
        black_cycle = 0;
        ignore_go++;
        if (ignore_go >= 4) {
          ignore_go = 0;
          if (RECT_CURVE_R || RECT_CURVE_L) { // if after 4 cycles these variables are still true it is because the robot passed through an intersection and didnt turn
            vPrint1String( "\t\tS\n\r" );
            UPDATE_INDEX = true;
            send_lcd = S;
          }
          preventBTL = false;
          RECT_CURVE_R = false;
          RECT_CURVE_L = false;
        }

        //goes in front by default if it didn't passed by a crossing
        EnviaRodas[0] = Velocidade;
        EnviaRodas[1] = Velocidade;
        //a crossing was detected before so it will turn instead of going in front
        if (CROSSING) {
          IGNORE = true;
          CROSSING = false;
          // the robot turns right if it is the predefined way of turning while the optimal path is not found.
          // can turn right if: the path is found; the robot is going from the start to the goal; and the next intersection on the optimal_path vector is a right turn
          // can turn right if: the path is found; the robot is going back from the goal to the start; and the next intersection on the optimal_path vector is a left turn
          //this is because the optimal_path is writen when the robot is going from the start to goal, so a right turn when the robot is going from the start to goal is
          //  a left turn when the robot is comming back
          if ( (TURN_DIR && !PATH_FOUND) || (optimal_path[index_optimal] == R && PATH_FOUND && !GOING_BACK)
               || (optimal_path[index_optimal] == L && PATH_FOUND && GOING_BACK) ) {

            //turn right
            EnviaRodas[0] = Velocidade * 1.4 ; //curva 90 graus direita
            EnviaRodas[1] = (Velocidade * -0.4);
            UPDATE_INDEX = true;
            vPrint1String( "\t\tR\n\r" );
            send_lcd = R;

            // the robot turns left if it is the predefined way of turning while the optimal path is not found.
            // can turn left if: the path is found; the robot is going from the start to the goal; and the next intersection on the optimal_path vector is a left turn
            // can turn left if: the path is found; the robot is going back from the goal to the start; and the next intersection on the optimal_path vector is a right turn
            //this is because the optimal_path is writen when the robot is going from the start to goal, so a left turn when the robot is going from the start to goal is
            //  a right turn when the robot is comming back
          } else if ( (!TURN_DIR && !PATH_FOUND) || (optimal_path[index_optimal] == L && PATH_FOUND && !GOING_BACK)
                      || (optimal_path[index_optimal] == R && PATH_FOUND && GOING_BACK)) {
            //turn left
            //robot.turnRobotInCross(false); //Cruzamento --> Roda 90º á esquerda
            EnviaRodas[0] = (Velocidade * -0.4); //curva 90 graus esquerda
            EnviaRodas[1] = Velocidade * 1.4;

            UPDATE_INDEX = true;

            vPrint1String( "\t\tL\n\r" );
            send_lcd = L;

          } else {
            UPDATE_INDEX = true;
            send_lcd = S;
            //vPrint1String( "\t\tS\n\r" );
          }
        }


        // [0 0 1 0]
      } else if ((!robot.chkLine(RecebeInfra[0]) && !robot.chkLine(RecebeInfra[1]) && robot.chkLine(RecebeInfra[2]) && !robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0)) {
        white_cycle = 0;
        black_cycle = 0;
        //RECT_CURVE_R = false;
        //RECT_CURVE_L = false;
        //curva ligeira a esquerda
        EnviaRodas[0] = Velocidade * 0.5;
        EnviaRodas[1] = Velocidade ;
        //direction_flag = "CLE\n\r";


        // [0 1 0 0]
      } else if ((!robot.chkLine(RecebeInfra[0]) && robot.chkLine(RecebeInfra[1]) && !robot.chkLine(RecebeInfra[2]) && !robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0)) {
        white_cycle = 0;
        black_cycle = 0;
        //RECT_CURVE_R = false;
        //RECT_CURVE_L = false;
        //curva ligeira a direita
        EnviaRodas[0] = Velocidade ;
        EnviaRodas[1] = Velocidade * 0.5;
        //direction_flag = "CLD\n\r";


        //        // [0 0 1 1]
        //      }  else if ((!robot.chkLine(RecebeInfra[0]) && !robot.chkLine(RecebeInfra[1]) && robot.chkLine(RecebeInfra[2]) && robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0)) {
        //        white_cycle = 0;
        //        black_cycle = 0;
        //RECT_CURVE_R = false;
        //RECT_CURVE_L = false;
        //curva brusca a esquerda
        //EnviaRodas[0] =  Velocidade * -0.3;//-0.4;
        //EnviaRodas[1] = Velocidade * 1;
        //direction_flag = "[0011]\n\r";

        //        // [1 1 0 0]
        //      }  else if ((robot.chkLine(RecebeInfra[0]) && robot.chkLine(RecebeInfra[1]) && !robot.chkLine(RecebeInfra[2]) && !robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0)) {
        //        white_cycle = 0;
        //        black_cycle = 0;
        //RECT_CURVE_R = false;
        //RECT_CURVE_L = false;
        //curva brusca a direita
        //EnviaRodas[0] = Velocidade * 1;
        //EnviaRodas[1] = Velocidade * -0.3;
        //direction_flag = "[1100]\n\r";

        // [0 0 0 1]
      }  else if ((!robot.chkLine(RecebeInfra[0]) && !robot.chkLine(RecebeInfra[1]) && !robot.chkLine(RecebeInfra[2]) && robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0)) {
        white_cycle = 0;
        black_cycle = 0;
        preventBTL = false;
        RECT_CURVE_R = false;
        RECT_CURVE_L = false;
        //curva brusca a esquerda
        //EnviaRodas[0] = Velocidade * 0.3;
        //EnviaRodas[1] = Velocidade;
        EnviaRodas[0] = (Velocidade * -0.4); //sensor da direita no preto-> faz cuva  "brusca" à esquerda
        EnviaRodas[1] = (Velocidade * 1.3);
        // direction_flag = "CBE\n\r";


        // [1 0 0 0]
      }  else if ((robot.chkLine(RecebeInfra[0]) && !robot.chkLine(RecebeInfra[1]) && !robot.chkLine(RecebeInfra[2]) && !robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0)) {
        white_cycle = 0;
        black_cycle = 0;
        preventBTL = false;
        RECT_CURVE_R = false;
        RECT_CURVE_L = false;
        //curva brusca a direita

        EnviaRodas[0] = (Velocidade * 1.3); //sensor da direita no preto-> faz cuva  "brusca" à direita
        EnviaRodas[1] = (Velocidade * -0.4);
        //direction_flag = "CBD\n\r";


        // [0 0 0 0]
      }  else if (!robot.chkLine(RecebeInfra[0]) && !robot.chkLine(RecebeInfra[1]) && !robot.chkLine(RecebeInfra[2]) && !robot.chkLine(RecebeInfra[3]) && (ignore_cycle == 0)) {
        //volta para a linha
        black_cycle = 0;


        //direction_flag = "branco\n\r" ;
        if ( (RECT_CURVE_L && !PATH_FOUND) || (optimal_path[index_optimal] == L && PATH_FOUND && RECT_CURVE_L && !GOING_BACK)
             || (optimal_path[index_optimal] == R && PATH_FOUND && RECT_CURVE_L && GOING_BACK)) {
          //turn left
          EnviaRodas[0] = (Velocidade * -0.5); //curva 90 graus esquerda
          EnviaRodas[1] = Velocidade * 1.4;
          vPrint1String( "\t\tL\n\r" );
          UPDATE_INDEX = true;
          send_lcd = L;
          preventBTL = true;
          //direction_flag = "\t\tC9E\n\r";
          RECT_CURVE_L = false;
          IGNORE = true;
        } else if ( (RECT_CURVE_R && !PATH_FOUND) || (optimal_path[index_optimal] == R && PATH_FOUND && RECT_CURVE_R && !GOING_BACK )
                    || (optimal_path[index_optimal] == L && PATH_FOUND && RECT_CURVE_R && GOING_BACK)) {
          //turn right
          EnviaRodas[0] = Velocidade * 1.4; //curva 90 graus direita
          EnviaRodas[1] = (Velocidade * -0.5);
          UPDATE_INDEX = true;
          vPrint1String( "\t\tR\n\r" );
          send_lcd = R;
          preventBTL = true;
          RECT_CURVE_R = false;
          IGNORE = true;
        } else if (CROSSING ) {
          CROSSING = false;
          IGNORE = true;
          if ( (TURN_DIR && !PATH_FOUND) || (optimal_path[index_optimal] == R && PATH_FOUND && !GOING_BACK)
               || (optimal_path[index_optimal] == L && PATH_FOUND && GOING_BACK) ) {
            //turn right
            //robot.turnRobotInCross(true); //Cruzamento --> Roda 90º á Direita
            EnviaRodas[0] = Velocidade * 1.4 ; //curva 90 graus direita
            EnviaRodas[1] = (Velocidade * -0.5);

            UPDATE_INDEX = true;
            vPrint1String( "\t\tR\n\r" );
            send_lcd = R;
            preventBTL = true;
          } else if ( (!TURN_DIR && !PATH_FOUND) || (optimal_path[index_optimal] == L && PATH_FOUND && !GOING_BACK)
                      || (optimal_path[index_optimal] == R && PATH_FOUND && GOING_BACK)) {
            //turn left
            //robot.turnRobotInCross(false); //Cruzamento --> Roda 90º á esquerda
            EnviaRodas[0] = (Velocidade * -0.5); //curva 90 graus esquerda
            EnviaRodas[1] = Velocidade * 1.4;

            UPDATE_INDEX = true;
            vPrint1String( "\t\tL\n\r" );
            send_lcd = L;
            preventBTL = true;
          }
        } else if (white_cycle >= 11) {
          //robot.turnRobotBackToLine();
          EnviaRodas[0] = Velocidade * 1;
          EnviaRodas[1] = (Velocidade * -1);
          //direction_flag = "\t\tBTL\n\r";
          vPrint1String( "\t\tB\n\r" );
          if (PATH_FOUND ) {
            index_optimal = 0;
            GOING_BACK = false;
          }
          send_lcd = B;
          preventBTL = true;
          white_cycle = 0;
          //white_cycle++;
        } else if ( !preventBTL) {
          //vPrint1String( "2\n\r" );
          white_cycle++;
        }

        // [1 1 1 1]
      } else if ((robot.chkLine(RecebeInfra[0]) && robot.chkLine(RecebeInfra[1]) && robot.chkLine(RecebeInfra[2]) && robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0) ) {
        white_cycle = 0;
        black_cycle++;
        RECT_CURVE_L = false;
        RECT_CURVE_R = false;
        CROSSING = true;
        //meta ou cruzamento
        if (black_cycle >= 8) {
          taskENTER_CRITICAL();
          {
            robot.setMotorSpeed(robot.ESQ, 0); //Set Motor Speed
            robot.setMotorSpeed(robot.DIR, 0);
            while (digitalRead(robot.SWITCH1)) {}; // Espera botão a '1'
            while (!digitalRead(robot.SWITCH1)) {}; // Espera botão a '0'

          }
          taskEXIT_CRITICAL();

        } else if (black_cycle >= 4) {
          //CROSSING = false;
          IGNORE = true;
          GOAL = true;
          send_lcd = G;
          //black_cycle = 0;
          EnviaRodas[0] = Velocidade * 1.4; //meta
          EnviaRodas[1] = -Velocidade * 1.4;
          if (PATH_FOUND) {
            index_optimal = size_path;
            GOING_BACK = true;
          }
        }


        // [0 1 1 1]
      } else if ((!robot.chkLine(RecebeInfra[0]) && robot.chkLine(RecebeInfra[1]) && robot.chkLine(RecebeInfra[2]) && robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0))  {
        black_cycle = 0;
        white_cycle = 0;
        CROSSING = false;

        if ( (!TURN_DIR && !PATH_FOUND) || (optimal_path[index_optimal] == L && PATH_FOUND && !GOING_BACK)
             || (optimal_path[index_optimal] == R && PATH_FOUND && GOING_BACK)) {
          //turn left
          EnviaRodas[0] = (Velocidade * -0.4); //curva 90 graus esquerda
          EnviaRodas[1] = Velocidade * 1.2 ;

          UPDATE_INDEX = true;
          vPrint1String( "\t\tL\n\r" );
          send_lcd = L;
          RECT_CURVE_L = false;
          IGNORE = true;
        } else  {
          RECT_CURVE_L = true;
          preventBTL = true;
          ignore_go = 0;
          //vPrint1String( "L True\n\r" );
        }

        // [1 1 1 0]
      }  else if ((robot.chkLine(RecebeInfra[0]) && robot.chkLine(RecebeInfra[1]) && robot.chkLine(RecebeInfra[2]) && !robot.chkLine(RecebeInfra[3])) && (ignore_cycle == 0) )  {
        black_cycle = 0;
        white_cycle = 0;
        CROSSING = false;

        if ( (TURN_DIR && !PATH_FOUND) || (optimal_path[index_optimal] == R && PATH_FOUND && !GOING_BACK)
             || (optimal_path[index_optimal] == L && PATH_FOUND && GOING_BACK)) {

          EnviaRodas[0] = Velocidade * 1.2; //curva 90 graus direita
          EnviaRodas[1] = (Velocidade * -0.4);
          UPDATE_INDEX = true;
          vPrint1String( "\t\tR\n\r" );
          send_lcd = R;
          RECT_CURVE_R = false;
          IGNORE = true;
        } else  {
          RECT_CURVE_R = true;
          preventBTL = true;
          ignore_go = 0;
          //vPrint1String( "R True\n\r" );
        }
      }

      if (UPDATE_INDEX && PATH_FOUND ) {
        if (!GOING_BACK) { // robo sabe o caminho e vai da partida para a meta logo incrementa pois le o vetor do inicio para o fim
          index_optimal++;
        } else { //robo segue da meta para a partida logo lê o vetor do caminho do fim para o inicio
          index_optimal--;
        }
      }
      UPDATE_INDEX = false;


      if (ignore_cycle <= 11 && IGNORE) {
        ignore_cycle++;
        white_cycle = 0;
        RECT_CURVE_R = false;
        RECT_CURVE_L = false;
        // vPrint1String( "IGN1\n\r" );
      } else {
        ignore_cycle = 0;
        //vPrint1String( "IGN0\n\r" );
        IGNORE = false;
        //RECT_CURVE_R = false;
        //RECT_CURVE_L = false;
      }

      Estado_Envio_Motors = xQueueSendToBack( SensorsQueue, &EnviaRodas, 0); // envia dados para a queue
      // Caso não envie para a queue dentro do tempo definido imprime na porta série
      if ( Estado_Envio_Motors != pdPASS ) {
        vPrint1String( "[BRAIN] -> Could not send to -> [MOTORS].\n\r" );
      }
      //vPrint1StringAndNumber("Msended:", EnviaRodas[0]);
      // vPrint1StringAndNumber("Msended:", EnviaRodas[1]);

      if (send_lcd != 0) {
        Estado_Envio_LCD = xQueueSendToBack( LCDQueue, &send_lcd, 0); // envia dados para a queue
        // Caso não envie para a queue dentro do tempo definido imprime na porta série
        if ( Estado_Envio_LCD != pdPASS ) {
          vPrint1String( "[BRAIN] -> Could not send to -> [LCD].\n\r" );
        }
        xSemaphoreGive( sem_lcd);

        if (!PATH_FOUND && !GOAL) {
          optimal_path[index_optimal] = send_lcd ;
          index_optimal++;

          //xQueuePeek(LCDQueue, &send_lcd, 0);
          //vPrint1StringAndNumber("sended:", send_lcd);
          //send_lcd = 0;

          //vPrint1String( "giv" );

          if (index_optimal >= 3) {
            last3 = ((optimal_path[index_optimal - 3] * 100) + (optimal_path[index_optimal - 2] * 10) + (optimal_path[index_optimal - 1]));

            if (last3 == LBR || last3 == RBL || last3 == SBS) {
              optimal_path[index_optimal - 3] = B;
              index_optimal = index_optimal - 2;
            } else if (last3 == LBL || last3 == RBR) {
              optimal_path[index_optimal - 3] = S;
              index_optimal = index_optimal - 2;
              //RIGHT
            } else if (TURN_DIR && (last3 == RBS || last3 == SBR) ) {
              optimal_path[index_optimal - 3] = L;
              index_optimal = index_optimal - 2;
              //Left
            } else if (!TURN_DIR && (last3 == LBS || last3 == SBL)) {
              optimal_path[index_optimal - 3] = R;
              index_optimal = index_optimal - 2;
            }
          }
        }



      }
      send_lcd = 0;

      //robot.setMotorSpeed(robot.ESQ, EnviaRodas[0]); //Set Motor Speed
      //robot.setMotorSpeed(robot.DIR, EnviaRodas[1]);
      //vPrint1StringAndNumber("IGN---:", ignore_cycle);
      //vPrint1StringAndNumber("WHI---:", white_cycle);
      //vPrint1String( direction_flag );
      //direction_flag = "STR EMPTY\n\r";
    }




    vTaskDelayUntil( &xLastWakeTime, ( BRAIN_DELAY / portTICK_PERIOD_MS )); // delay until
  }
}
/*------------------------------------ => Sensors <= -------------------------------------------*/
static void Sensors_task( void *pvParameters ) {


  //Declaração Sensores
  int S_ESQ[4] =      {999, 999, 999, 999};
  int S_MEIO_ESQ[4] = {999, 999, 999, 999};
  int S_MEIO_DIR[4] = {999, 999, 999, 999};
  int S_DIR[4] =      {999, 999, 999, 999};
  int EnviaInfra[4];
  char distance = 0;
  char count = 0;
  //Obtenção Tempo Inicio Tarefa
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  portBASE_TYPE Estado_Envio;
  portBASE_TYPE sem_result;
  for ( ;; ) {
    //vPrint1String( "[Sen]\n\r" );

    sem_result = xSemaphoreTake( sem_sensors, ( SENSORS_SEM_LOCK_TIME / portTICK_PERIOD_MS ));
    if (sem_result != pdFAIL) {
      char i;
      for (i = 0; i < 2; i++) {
        robot.readSensors();  // Lê os sensores ir

        S_DIR[i] = robot.sensor_values[0];
        S_MEIO_DIR[i] = robot.sensor_values[1];
        S_MEIO_ESQ[i] = robot.sensor_values[2];
        S_ESQ[i] = robot.sensor_values[3];
      }
      //Declaração Array para Guardar Média Sensores. Cada Indice => Uma Média Sensor

      //Calcula Média Sensores
      EnviaInfra[0] = (S_DIR[0] + S_DIR[1]  ) / 2;                      //Sensor Dir.
      EnviaInfra[1] = (S_MEIO_DIR[0] + S_MEIO_DIR[1]  ) / 2;  //Sensor Meio Dir.
      EnviaInfra[2] = (S_MEIO_ESQ[0] + S_MEIO_ESQ[1] ) / 2;  //Sensor Meio Esq.
      EnviaInfra[3] = (S_ESQ[0] + S_ESQ[1]  ) / 2;                      //Sensor Esq.



      //Envia a Média dos 4 Sensores (Array) para a Queue
      Estado_Envio = xQueueSendToBack( SensorsQueue, &EnviaInfra, 0); // envia dados para a queue


      digitalWrite(TRIGPIN, LOW); // Set the trigger pin to low for 2uS
      delayMicroseconds(2);
      digitalWrite(TRIGPIN, HIGH); // Send a 10uS high to trigger ranging
      delayMicroseconds(10);
      digitalWrite(TRIGPIN, LOW); // Send pin low again
      unsigned int duration = pulseIn(ECHOPIN, HIGH, 1200 ); // Lê tempo
      if (duration != 0) {
        distance += duration / 58; // Calcula distância
        //vPrint1StringAndNumber("[Sensors]:", distance);
        count++;
        if (count == 4 ) {
          count = 0;
          distance = distance / 4;
          if (distance < 9 ) {
            vPrint1String( "obs\n\r" );
            digitalWrite(IntPinSonar, LOW);
            digitalWrite(IntPinSonar, HIGH); //gera interrupção para contornar objeto
          }
        }
      }



      // Caso não envie para a queue dentro do tempo definido imprime na porta série
      if ( Estado_Envio != pdPASS ) {
        vPrint1String( "[INFRA-VERMELHOS] -> Could not send to -> [CEREBRO].\n\r" );
      }
    }
  }
}
/*------------------------------------ => LCD <= -------------------------------------------*/
static void LCD_task( void *pvParameters ) {
  char receive_lcd = 0;
  char row = 0;
  char column = 0;
  
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  portBASE_TYPE EstadoRecebeBrain;
  portBASE_TYPE sem_result;
  for ( ;; )
  {
    //vPrint1String( "[LCD]\n\r" );
    sem_result = xSemaphoreTake( sem_lcd, ( LCD_SEM_LOCK_TIME / portTICK_PERIOD_MS ));
    //vPrint1String( "take\n\r" );
    if (sem_result != pdFAIL) {


      EstadoRecebeBrain = xQueueReceive(LCDQueue, &receive_lcd, 0);// (100 / portTICK_PERIOD_MS));
      //vPrint1StringAndNumber("ai", receive_lcd);
      if (EstadoRecebeBrain != pdPASS) {
        vPrint1String( "[LCD] -> Could not receive from -> [BRAIN].\n\r" );
      } else {
        //vPrint1StringAndNumber("col", column);
        //vPrint1StringAndNumber("row", row);
        switch (receive_lcd) {

          case S :
            lcd.LcdCharacter('S');
            //lcd.gotoXY(column, row);
            break;

          case B :
            lcd.LcdCharacter('B');
            //lcd.gotoXY(column, row);
            break;

          case R :
            lcd.LcdCharacter('R');
            //lcd.gotoXY(column, row);
            break;

          case L :
            lcd.LcdCharacter('L');
            //lcd.gotoXY(column, row);
            break;
          case G :
            lcd.gotoXY(0, row + 1);
            lcd.LcdString("Meta:       ");
            lcd.gotoXY(42, row + 1);
            //lcd.gotoXY(column, row);
            break;
            //default:
            //lcd.LcdCharacter("ERRO");
        }
        column = column + 7;
        if (column > 84) {
          column = 0;
          row++;
        }
        if (row > 5)row = 0;

      }





      //vTaskDelayUntil( &xLastWakeTime, ( 250 / portTICK_PERIOD_MS ) ); //delay until;
      //taskYIELD();
    }
  }
}


/*---------------------------------------- => Motors <= -----------------------------------------*/
static void Motors_task ( void *pvParameters ) {
  int RecebeRodas [2] = {0, 0};
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  portBASE_TYPE EstadoRecebeBrain;
  portBASE_TYPE sem_result;
  for ( ;; )
  {
    // vPrint1String( "[Mot]\n\r" );

    sem_result = xSemaphoreTake( sem_motors, ( MOTORS_SEM_LOCK_TIME / portTICK_PERIOD_MS ));
    if (sem_result != pdFAIL) {
      EstadoRecebeBrain = xQueueReceive(SensorsQueue, &RecebeRodas, (50 / portTICK_PERIOD_MS));
      if (EstadoRecebeBrain != pdPASS) {
        vPrint1String( "[Motors] -> Could not receive from -> [BRAIN].\n\r" );
      } else {
        //vPrint1StringAndNumber("Mrec:", RecebeRodas[0]);
        //vPrint1StringAndNumber("Mrec:", RecebeRodas[1]);
        robot.setMotorSpeed(robot.ESQ, RecebeRodas[0]); //Set Motor Speed
        robot.setMotorSpeed(robot.DIR, RecebeRodas[1]);
      }
      //vTaskDelayUntil( &xLastWakeTime, (10 / portTICK_PERIOD_MS ) ); // delay until
      //taskYIELD();

    }
  }
}


/*---------------------------------------- => Config <= -----------------------------------------*/
static void Config_task ( void *pvParameters ) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  bool state;
  for ( ;; )
  {
    vPrint1String( "[Config]\n\r" );
    state = digitalRead(DIR_BUTTON);

    xQueueSendToBack( ConfQueue, &state, 0); // envia dados para a queue


    vTaskDelete(NULL);
  }
}

/*-------------------------- => INTERRUPÇÃO ULTRA-SONS <= -------------------------*/
static void InterruptSonar( void ) {

  vPrint1String( "[ULTRA]\n\r");
  char vel_DIR = 85 + 4;
  char vel_ESQ = 85;

  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  robot.setMotorSpeed(robot.ESQ, -vel_ESQ); //anda para a esquerda
  robot.setMotorSpeed(robot.DIR, vel_DIR);
  delay(115000 / portTICK_PERIOD_MS);
  // delayMicroseconds(615);

  robot.setMotorSpeed(robot.ESQ, vel_ESQ); //anda para em frente
  robot.setMotorSpeed(robot.DIR, vel_DIR);
  delay(180000 / portTICK_PERIOD_MS);
  //delayMicroseconds(1000);

  robot.setMotorSpeed(robot.ESQ, vel_ESQ); //anda para a direita
  robot.setMotorSpeed(robot.DIR, -vel_DIR);
  delay(120000 / portTICK_PERIOD_MS);
  //delayMicroseconds(615);

  robot.setMotorSpeed(robot.ESQ, vel_ESQ); //anda para a frente
  robot.setMotorSpeed(robot.DIR, vel_DIR);
  delay(360000 / portTICK_PERIOD_MS);
  //delayMicroseconds(2000);

  robot.setMotorSpeed(robot.ESQ, vel_ESQ); //anda para a direita
  robot.setMotorSpeed(robot.DIR, -vel_DIR);
  delay(120000 / portTICK_PERIOD_MS);
  //delayMicroseconds(615);

  robot.setMotorSpeed(robot.ESQ, vel_ESQ); //anda para a frente
  robot.setMotorSpeed(robot.DIR, vel_DIR);
  delay(200000 / portTICK_PERIOD_MS);
  //delayMicroseconds(1000);

  robot.setMotorSpeed(robot.ESQ, -vel_ESQ); //anda para a esquerda
  robot.setMotorSpeed(robot.DIR, vel_DIR);
  delay(120000 / portTICK_PERIOD_MS);
  //delayMicroseconds(615);


  if ( xHigherPriorityTaskWoken == pdTRUE )
  {
    vPortYield(); // Liberta a interrupção
  }
}

void loop() {}





