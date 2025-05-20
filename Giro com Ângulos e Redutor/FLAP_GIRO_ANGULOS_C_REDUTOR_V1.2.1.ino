/* --------------------------------------------------------------------------------------------------------------------------------------------------------

Este software é protegido por direitos autorais e leis internacionais. Qualquer cópia não autorizada, distribuição ou uso deste software, total ou parcial,
será considerada uma violação dos direitos autorais e sujeita a medidas legais. 
Conforme estipulado pela Lei de Direitos Autorais, Lei nº 9.610/98, a pirataria de software é estritamente proibida e sujeita a penalidades legais. A cópia
não autorizada deste software constitui uma violação dos direitos de propriedade intelectual, passível de processo civil e criminal.
Ressaltamos que qualquer tentativa de reprodução, distribuição ou uso não autorizado deste software será monitorada e tratada com rigor dentro dos limites 
da lei.

-------------------------------------------------------------------------------------------------------------------------------------------------------- */

#include <IRremote.h>        // IR
#include <RCSwitch.h>        // RF
#include <AccelStepper.h>    // Motor de passo
#include <EEPROM.h>          // Memoria
  
// Os #define utilizados abaixo serve para as conexões do motor de passo e o tipo de interface do motor.
// O tipo de interface do motor deve ser definido como '1' ao usar um driver.

#define motorInterfaceType 1
#define dirPin 6                       // Pino de direcao do Motor de passo
#define stepPin 7                      // Pino de passos do Motor de passo
#define FC_F 11                        // Fim de curso de fechamento 
#define FC_A 10                        // Fim de curso de abertura

// Já esses #define abaixo são para o Atuador Linear 

#define MtiltR 4         // Atuador Linear - Right (Direita)
#define MtiltL 5         // Atuador Linear - Left (Esquerda)

// --------------------------------------------------------------------------------------------------------------------------------------------------------

unsigned long int tempoanteriorGiro = 0;      // Variavel que guarda o momento que comeca a girar o motor
unsigned long int tempobotao = 0;             // Variavel que guarda o momento que comeca a girar o motor
unsigned long int TempoAbreFecha = 0; 

bool dir;                               // EEPROM 11 - Velocidade do giro
bool esq;                               // EEPROM 12 - Velocidade do giro
bool dirStatus;                         // EEPROM 13 - Velocidade do giro
bool esqStatus;                         // EEPROM 14 - Velocidade do giro
bool volta;                             // EEPROM 15 - Velocidade do giro

int vel = 0;                            // EEPROM 10 - Velocidade do giro 
int acceleration = 0;
long ToGo = 0;                          // EEPROM 10 - Velocidade do giro
int contador;                           // EEPROM 16 - Velocidade do giro
int controleoff;                        // EEPROM 17 - Velocidade do giro
int controleoff2;                       // EEPROM 18 - Velocidade do giro
int controleON;                        // EEPROM 17 - Velocidade do giro
int control;                            // EEPROM 19 - Velocidade do giro
int encStatus;                          // EEPROM 20 - Velocidade do giro
int ultimoCmdPan;                       // EEPROM 21 - Velocidade do giro
int controlByPass;                       // EEPROM 22 - Velocidade do giro
int estadoCmdTil;                       // EEPROM 23 - Velocidade do giro
int ultimoCmdTil;                       // EEPROM 24 - Velocidade do giro
int momentoDir;                         // EEPROM 25 - Velocidade do giro
int momentoEsc;                         // EEPROM 26 - Velocidade do giro
int CodCmd;                             // EEPROM 27 - Velocidade do giro 
int CodIFs;                             // EEPROM 28 - Velocidade do giro
int CodExecutation;                     // EEPROM 29 - Velocidade do giro
int MtiltState1 = HIGH;                 // EEPROM 30 - ESTADO DE SAIDA DO PINO
int MtiltState2 = HIGH;                 // EEPROM 31 - ESTADO DE SAIDA DO PINO
int estadotil;                          // EEPROM 32 - Estado controle abertura
int estadopan;                          // EEPROM 33 - Estado controle giro
int RECV_PIN = 16;                      // Pino IR 12
int pinoSensor = 9;                     // PINO DIGITAL UTILIZADO PELO SENSOR ENCODER

// ---- POSITION --------------------------------------------------------- POSITION --------------------------------------------------------- POSITION ----
long Position;                           // Posicao atual
long SetPoint;                           // Ponde de interesse de parada
int CTRLposition = 0;                   // Variavel de controle da logica de Posicao


// ---- IR ------------------------------------------------------------------ IR ------------------------------------------------------------------ IR ----
float armazenavalor;                    // Variavel valor do IR

IRrecv irrecv(RECV_PIN);
decode_results results;

// ---- RF ------------------------------------------------------------------ RF ------------------------------------------------------------------ RF ----

RCSwitch mySwitch = RCSwitch();

// ---- Motor de passo ------------------------------------------------ Motor de passo ------------------------------------------------ Motor de passo ----

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup()
{
  pinMode(MtiltR, OUTPUT);             // Define os pinos dos motores como saida
  pinMode(MtiltL, OUTPUT);             // Define os pinos dos motores como saida
  pinMode(13, OUTPUT);                 // Define os pinos dos motores como saida

  pinMode(pinoSensor, INPUT);            //DEFINE O PINO DO SENSOR ENCODER COMO ENTRADA
  pinMode(FC_A, INPUT);                  //DEFINE O PINO DE ENTRADA DE SINAL DO FIM DE CURSO DE ABERTURA
  pinMode(FC_F, INPUT);                  //DEFINE O PINO DE ENTRADA DE SINAL DO FIM DE CURSO DE FECHAMENTO

  digitalWrite(MtiltR, MtiltState1);   // Define o estado inicial
  digitalWrite(MtiltL, MtiltState2);   // Define o estado inicial
  
  irrecv.enableIRIn();                 // Inicializa o receptor IR

  mySwitch.enableReceive(0);           // Receiver on interrupt 0 => that is pin #2

  stepper.setSpeed(0);
  
  pinMode(dirPin, OUTPUT);             // Direcao do Motor aux
  pinMode(stepPin, OUTPUT);            // Define os pinos dos motores como saida
  
  Serial.begin(115200);

  //vel = EEPROM.read(10);
  dir = EEPROM.read(11);
  esq = EEPROM.read(12);
  dirStatus = EEPROM.read(13);
  esqStatus = EEPROM.read(14);
  volta = EEPROM.read(15);
  contador = EEPROM.read(16);
  controleoff = EEPROM.read(17);
  controleoff2 = EEPROM.read(18);
  control = EEPROM.read(19);
  encStatus = EEPROM.read(20);
  ultimoCmdPan = EEPROM.read(21);
  controlByPass = EEPROM.read(22);
  estadoCmdTil = EEPROM.read(23);
  ultimoCmdTil = EEPROM.read(24);
  momentoDir = EEPROM.read(25);
  momentoEsc = EEPROM.read(26);
  CodCmd = EEPROM.read(27);
  CodIFs = EEPROM.read(28);
  CodExecutation = EEPROM.read(29);

  //MtiltState1 = EEPROM.read(30);
  //MtiltState2 = EEPROM.read(31);

  estadotil = EEPROM.read(32);
  estadopan = EEPROM.read(33);

}

void loop() {

        Position = stepper.currentPosition();                     // Position recebe o valor em pulos da biblioteca AccellStepper

// ---- Loop do RF ------------------------------------------------------ Loop do RF ------------------------------------------------------ Loop do RF ----

    if (mySwitch.available()) {

    int value = mySwitch.getReceivedValue();
     if (value == 0){
     Serial.print("Codigo desconhecido");
     CodCmd = 101; // ##################################################### Codigo RF Desconhecido
     } else {

      if (mySwitch.getReceivedValue() == 15130376) {             //Abre/Para
        
        
        if (millis() - tempobotao >= 500){
           if (MtiltState1 == HIGH && MtiltState2 == HIGH){
            
              Descer();
            
           } else {
            
              PararControl();
              controleON = 0;
            
           }
       }  
    }   
       else if (mySwitch.getReceivedValue() == 15130380) {       //Fecha/Para
             
        if (millis() - tempobotao >= 500){  
            if (MtiltState1 == HIGH && MtiltState2 == HIGH){

              Subir();

           } else {

              PararControl();
              controleON = 0;
           }
       }
    }
      else if (mySwitch.getReceivedValue() == 15130372) {       //Fecha/Para
             
        if (millis() - tempobotao >= 800){  
            if (stepper.isRunning() == 0){

              ToGo = 200000;
              Direito();

           } else {

              PararControl();
              controleON = 0;
           }
       }
    }
      else if (mySwitch.getReceivedValue() == 15130377) {       //Fecha/Para
             
        if (millis() - tempobotao >= 800){  
            if (stepper.isRunning() == 0){
              
              ToGo = -200000;
              Esquerdo();

           } else {

              PararControl();
              controleON = 0;
           }
       }
    }        
     
    Serial.print("Received ");
    Serial.println( mySwitch.getReceivedValue() );
    
    mySwitch.resetAvailable();
    }
    }
  
// ---- Loop do IR ----------------------------------------------------- Loop do IR ------------------------------------------------------- Loop do IR ----

 
  if (irrecv.decode(&results)) {
    
    Serial.print("Valor lido : ");
    Serial.println(results.value, HEX);
    armazenavalor = (results.value);

    if (armazenavalor == 0x4DB253AC || armazenavalor == 0x76B366E3)       //Verifica se a tecla DESCER foi acionada
    {
      //tempobotao = millis();
      if (millis() - tempobotao >= 800){  
      Subir();
      }
      CodCmd = 108; // ##################################################### Subir via IR
    }
    else if (armazenavalor == 0x4DB24BB4 || armazenavalor == 0x6EDFE961 ) //Verifica se a tecla SUBIR foi acionada
    {
      //tempobotao = millis();
      if (millis() - tempobotao >= 800){  
      Descer();
      }
      CodCmd = 109; // ##################################################### Descer via IR
    }
    else if (armazenavalor == 0x4DB2837C || armazenavalor == 0x406F92E7)  //Verifica se a tecla DIREITO foi acionada
    {
      //tempobotao = millis();
      if (millis() - tempobotao >= 800){
      ToGo = 200000;  
      Direito();
      }
      CTRLposition = 0;
      CodCmd = 110; // ##################################################### Direito via IR
    }
    else if (armazenavalor == 0x4DB29966 || armazenavalor == 0x20208F7)  //Verifica se a tecla ESQUERDO foi acionada
    { 
      //tempobotao = millis();
      if (millis() - tempobotao >= 800){
      ToGo = -200000;
      Esquerdo();
      }
      CTRLposition = 0;
      CodCmd = 111; // ##################################################### Esquerdo via IR
    }
    else if (armazenavalor == 0x4DB2738C || armazenavalor == 0x202B04F)  //Verifica se a tecla PARAR foi acionada
    { 
      //tempobotao = millis();
      if (millis() - tempobotao >= 800){
      PararControl();
      controleON = 0;
      }
      CodCmd = 112; // ##################################################### Parar via IR
    }
    else if (armazenavalor == 0x4DB23BC4 || armazenavalor == 0x203E1DBF)  //Verifica se a tecla POWER OFF foi acionada
    {
      if (millis() - tempobotao >= 800){  
      if (dirStatus == true && esqStatus == false){
          if (control == 0) {
              controleoff = 10; EEPROM.write(17, controleoff);
              ToGo = -200000;
              Esquerdo();
              CTRLposition = 0;
              CodCmd = 113; // ##################################################### Esquerdo via Power IR
          } else {
              controleoff = 10; EEPROM.write(17, controleoff);
              ToGo = 200000;
              Direito();
              CTRLposition = 0;
              CodCmd = 114; // ##################################################### Direito via Power IR
          }
          
      }else if (dirStatus == false && esqStatus == true){
              if (control == 0) {
                  controleoff = 10; EEPROM.write(17, controleoff);
                  ToGo = 200000;
                  Direito();
                  CTRLposition = 0;
                  CodCmd = 115; // ##################################################### Direito via Power IR
          } else {
                  controleoff = 10; EEPROM.write(17, controleoff);
                  ToGo = -200000;
                  Esquerdo();
                  CTRLposition = 0;
                  CodCmd = 116; // ##################################################### Esquerdo via Power IR
         }
    }
      Subir();  
  }    
}

else if (armazenavalor == 0x4DB2F10E || armazenavalor == 0x6EDFE961)                                //0º Positivo
    {
      if (millis() - tempobotao >= 800){

         if (digitalRead(FC_A) == LOW) {
      CTRLposition = 0;
      if (Position > 0) {
        ToGo = -200;
        Esquerdo();
      }else {
        ToGo = 200;
        Direito();
      }
      if (Position < 0) {
        ToGo = 200;
        Direito();
      }else {
        ToGo = -200;
        Esquerdo();
      }
    } else {
          Descer();
          ToGo = 0;
        }
    }
}
     
else if (armazenavalor == 0x4DB2A35C || armazenavalor == 0x6EDFE961)                                //45º Positivo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = 7787;
      if (ToGo < Position){
        Esquerdo();
      }else {
        Direito();
      }
      CodCmd = 109; // ##################################################### 
        } else {
          Descer();
          ToGo = 7787;
          controleON = 10;
        }
    }
    }
else if (armazenavalor == 0x4DB211EE || armazenavalor == 0x6EDFE961)                                //90º Positivo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = 15575;
      if (ToGo < Position){
        Esquerdo();
      }else {
        Direito();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = 15575;
          controleON = 10;
        }
      }
    }
else if (armazenavalor == 0x4DB241BE || armazenavalor == 0x6EDFE961)                                //135º Positivo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = 23362;
      if (ToGo < Position){
        Esquerdo();
      }else {
        Direito();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = 23362;
          controleON = 10;
        }
    }
    }
else if (armazenavalor == 0x4DB249B6 || armazenavalor == 0x6EDFE961)                                //180º Positivo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) { 
      CTRLposition = 1;
      ToGo = 31150;
      if (ToGo < Position){
        Esquerdo();
      }else {
        Direito();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = 31150;
          controleON = 10;
        }
    }
    }
else if (armazenavalor == 0x4DB2C936 || armazenavalor == 0x6EDFE961)                                //225º Positivo
    {
      if (millis() - tempobotao >= 800){ 

        if (digitalRead(FC_A) == LOW) { 
      CTRLposition = 1;
      ToGo = 38937; 
      if (ToGo < Position){
        Esquerdo();
      }else {
        Direito();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = 38937;
          controleON = 10;
        }
    }
    }
else if (armazenavalor == 0x4DB233CC || armazenavalor == 0x6EDFE961)                                //270º Positivo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) { 
      CTRLposition = 1;
      ToGo = 46724;
      if (ToGo < Position){
        Esquerdo();
      }else {
        Direito();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = 46724;
          controleON = 10;
        }
    } 
    }
else if (armazenavalor == 0x4DB2718E || armazenavalor == 0x6EDFE961)                                //315º Positivo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = 54511;
      if (ToGo < Position){
        Esquerdo();
      }else {
        Direito();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = 54511;
          controleON = 10;
        } 
    }
    }
else if (armazenavalor == 0x4DB213EC || armazenavalor == 0x6EDFE961)                                //45º Negativo
    {
      if (millis() - tempobotao >= 800){  

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = -7787;
      if (ToGo > Position){
        Direito();
      }else {
        Esquerdo();
      }
      CodCmd = 109; // ##################################################### 
      } else {
          Descer();
          ToGo = -7787;
          controleON = 10;
        }
    }
    }
else if (armazenavalor == 0x4DB251AE || armazenavalor == 0x6EDFE961)                                //90º Negativo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = -15575;
      if (ToGo > Position){
        Direito();
      }else {
        Esquerdo();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = -15575;
          controleON = 10;
        } 
    }
    }
else if (armazenavalor == 0x4DB2D12E || armazenavalor == 0x6EDFE961)                                //135º Negativo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) { 
      CTRLposition = 1;
      ToGo = -23362;
      if (ToGo > Position){
        Direito();
      }else {
        Esquerdo();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = -23362;
          controleON = 10;
        } 
    }
    }
else if (armazenavalor == 0x4DB223DC || armazenavalor == 0x6EDFE961)                                //180º Negativo
    {
      if (millis() - tempobotao >= 800){

         if (digitalRead(FC_A) == LOW) {  
      CTRLposition = 1;
      ToGo = -31150;
      if (ToGo > Position){
        Direito();
      }else {
        Esquerdo();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = -31150;
          controleON = 10;
        } 
    }
    }
else if (armazenavalor == 0x4DB27B84 || armazenavalor == 0x6EDFE961)                                //225º Negativo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = -38937;
      if (ToGo > Position){
        Direito();
      }else {
        Esquerdo();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = -38937;
          controleON = 10;
        } 
    }
    }
else if (armazenavalor == 0x4DB2E11E || armazenavalor == 0x6EDFE961)                                //270º Negativo
    {
      if (millis() - tempobotao >= 800){

        if (digitalRead(FC_A) == LOW) {
      CTRLposition = 1;
      ToGo = -46724;
      if (ToGo > Position){
        Direito();
      }else {
        Esquerdo();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = -46724;
          controleON = 10;
        }  
    }
    }
else if (armazenavalor == 0x4DB2FB04 || armazenavalor == 0x6EDFE961)                                //315º Negativo
    {
      if (millis() - tempobotao >= 800){
        
        if (digitalRead(FC_A) == LOW) {  
      CTRLposition = 1;
      ToGo = -54511;
      if (ToGo > Position){
        Direito();
      }else {
        Esquerdo();
      }
      CodCmd = 109; // #####################################################
      } else {
          Descer();
          ToGo = -54511;
          controleON = 10;
        } 
    }           
  }
    irrecv.resume(); //Le o próximo valor
}

// ---- Loop Serial ---------------------------------------------------- Loop Serial ---------------------------------------------------- Loop Serial ----
  
      if (Serial.available() > 0) {
        char letra = Serial.read();
            if (letra == 'w'){
                    Subir();
                    CodCmd = 117; // ##################################################### Subir via Serial
                    
            } else if (letra == 's'){
                    Descer();
                    CodCmd = 118; // ##################################################### Descer via Serial
                    
            } else if (letra == 'q'){
                    PararControl();
                    CodCmd = 119; // ##################################################### Parar via Serial
                    
            } else if (letra == 'd'){
                    ToGo = 200000;
                    Direito();
                    CTRLposition = 0;
                    CodCmd = 120; // ##################################################### Direito via Serial
                                         
            } else if (letra == 'a'){
                    ToGo = -200000;
                    Esquerdo();
                    CTRLposition = 0;
                    CodCmd = 121; // ##################################################### Esquerdo via Serial
            } 
}

// ---- Loop Lógica ------------------------------------------------- Loop Lógica ------------------------------------------------- Loop Lógica ----

if (CTRLposition == 0){
    if (contador == 0){
        if (millis() - tempoanteriorGiro >= 1300){         //750
         
            if (digitalRead(pinoSensor) == LOW){           // No centro -> Posicao correta
                               
                PararEnc();
                CodIFs = 201; // ##################################################### IF do Encoder
         }
      }
   } 
} else if (CTRLposition == 1) {
  if (digitalRead(pinoSensor) == LOW){           // No centro -> Posicao correta
      if (controlByPass == 1) {
             
             dirStatus = dir;  EEPROM.write(13, dirStatus);
             esqStatus = esq;  EEPROM.write(14, esqStatus);

             controlByPass = 0;
         }
     }
 }

setPressMs(5000);

/*

if (botao for pressionado){
  led começa a piscar;
  controle = 1;
}


if (controle == 1){
  if (mySwitch.available()) {

    butt1 = mySwitch.getReceivedValue();
     if (butt1 == 0){
     Serial.print("Codigo desconhecido");
     } else {

  controle = 0;
  led para de piscar;
}

butt2 = butt1 -4;

*/









/*
if (CTRLposition == 0){
  if (millis() - tempoanteriorGiro <= 1500){
    if (Position <= 200 || Position >= -200){
        acceleration = 200;
    }
  } 
}
*/

   if (controleoff == 10 && controleoff2 == 10 && volta == true){
            if (digitalRead(pinoSensor) == LOW){           // No centro -> Posicao correta
              
                Subir();
                CodIFs = 202; // ##################################################### IF da subida automatica
         }        
      }

if (millis() - tempoanteriorGiro <= 1500){
      encStatus = 0; //EEPROM.update(20, encStatus);
}

if (CTRLposition == 0){
if (digitalRead(pinoSensor) == LOW) {
      encStatus = 1; EEPROM.update(20, encStatus);
  }
}

if (CTRLposition == 1 ) {                                          // Verifica de algum comando de posicao foi chamado
  if (vel == 800 || vel == -800){                                  // Confere se os motores estao em movimento

    if (Position == SetPoint) {                                    // Chegando na posicao executa a parada
      PararControl(); 
      CTRLposition = 0;                                            // Volta a varaiavel de controle para a posicao inicial
    }
  }
}

  if (MtiltState1 == HIGH && MtiltState2 == LOW) {                // Verifica se o fim de curso de fechamento esta atuado  MtiltState1 == HIGH && MtiltState2 == LOW
        if (digitalRead(FC_F) == LOW) {                           // Confere se o flap esta executando o movimento de fechamento
                PararControl();                       
        }         
  } else if (MtiltState1 == LOW && MtiltState2 == HIGH) {         // Verifica se o fim de curso de abertura esta atuado  MtiltState1 == LOW && MtiltState2 == HIGH
        if (digitalRead(FC_A) == LOW) {                           // Confere se o flap esta executando o movimento de abertura
                PararControl();
                stepper.setCurrentPosition(0); 

                if (controleON == 10){
                  if (ToGo > 0) {
                     Direito();
                     controleON = 0;
                  } else if (ToGo < 0) {
                     Esquerdo();
                     controleON = 0;
                  } else {
                    PararControl();
                    controleON = 0;
                  }

                }
        }  
}      

        //Serial.println(digitalRead(pinoSensor));
        
        stepper.enableOutputs();
        stepper.run(); 

        //stepper.enableOutputs(); //enable pins
        //stepper.setSpeed(vel);
        //stepper.runSpeed(); 

        digitalWrite(MtiltR, MtiltState1);       // ESCREVE NOS PINOS
        digitalWrite(MtiltL, MtiltState2);       // ESCREVE NOS PINOS  stepper.isRunning()   
/*
        Serial.print("Runing: ");
        Serial.println(stepper.isRunning());
        Serial.print("encStatus: ");
        Serial.println(encStatus); 
        delay(100);
*/
/*
        Serial.print("Fim de curso Abertura: ");
        Serial.println(digitalRead(FC_A));
        Serial.print("Fim de curso Fechamento: ");
        Serial.println(digitalRead(FC_F)); 
        delay(100); */
}
// --------------------------------------------------------------------------------------------------------------------------------------------------------
/*
         //Serial.println(digitalRead(FC_A));
         //Serial.println(digitalRead(pinoSensor));
         //Serial.println(digitalRead(FC_F));
         //digitalWrite(13, volta);
        //Position = stepper.currentPosition();
         
         //Serial.print(CodCmd);
         //Serial.print(CodIFs);
        // Serial.println(CodExecutation );
         //Serial.println(volta);

        Serial.print("dir = ");
        Serial.println(dir);
        Serial.print("esq = ");
        Serial.println(esq);
        Serial.print("dirStatus = ");
        Serial.println(dirStatus);
        Serial.print("esqStatus = ");
        Serial.println(esqStatus);
        */
void Subir() {

         if (volta == true && encStatus == 1){        // No centro -> Posicao correta
             TempoAbreFecha = millis();
             
             MtiltState1 = HIGH;  //EEPROM.write(30, MtiltState1);                            
             MtiltState2 = LOW;   //EEPROM.write(31, MtiltState2);                           
    
             digitalWrite(MtiltR, MtiltState1);       // ESCREVE NOS PINOS
             digitalWrite(MtiltL, MtiltState2);       // ESCREVE NOS PINOS
            
             controleoff = 0;  EEPROM.write(17, controleoff);
             controleoff2 = 0;  EEPROM.write(18, controleoff2);

             ultimoCmdTil = 1;  EEPROM.write(24, ultimoCmdTil);
             estadoCmdTil = 10;  EEPROM.write(23, estadoCmdTil);

             tempobotao = millis();

             CTRLposition = 0;
             
             Serial.println("SUBINDO");
             CodExecutation = 301; // ##################################################### Função subir - Executada
         } else {
            tempobotao = millis();

            Serial.println("Subida nao permitida");
            CodExecutation = 302; // ##################################################### Função subir - Executada mas não permitida
         }
}

void Descer() {

         TempoAbreFecha = millis();
        
         MtiltState1 = LOW;   //EEPROM.write(30, MtiltState1);                            
         MtiltState2 = HIGH;  //EEPROM.write(31, MtiltState2);                           

         digitalWrite(MtiltR, MtiltState1);           // ESCREVE NOS PINOS
         digitalWrite(MtiltL, MtiltState2);           // ESCREVE NOS PINOS
         
         Serial.println("DESCENDO");
         CodExecutation = 303; // ##################################################### Função descer - Executada

         controleoff = 0;  EEPROM.write(17, controleoff);
         controleoff2 = 0;  EEPROM.write(18, controleoff2);

         ultimoCmdTil = 2;  EEPROM.write(24, ultimoCmdTil);
         estadoCmdTil = 10;  EEPROM.write(23, estadoCmdTil);

         tempobotao = millis();

         CTRLposition = 0;
}

void Direito() {
       if (digitalRead(FC_A) == LOW) {
            if (volta == true) {

                  vel = 2000;  
                  acceleration = 990;

                  stepper.setMaxSpeed(vel);
                  stepper.setAcceleration(acceleration);
                  stepper.moveTo(ToGo);

                  tempoanteriorGiro = millis();
                  tempobotao = millis();
                  dir = true;  EEPROM.write(11, dir);
                  esq = false;  EEPROM.write(12, esq);
                    if (contador == 1){
                          dirStatus = dir;  EEPROM.write(13, dirStatus);
                          esqStatus = esq;  EEPROM.write(14, esqStatus);
                          control = 0;  EEPROM.write(19, control);
                    }
                  contador = 0;  EEPROM.write(16, contador);
                  ultimoCmdPan = 1;  EEPROM.write(21, ultimoCmdPan);
                  encStatus = 0;  EEPROM.write(20, encStatus);
                  momentoDir = 1;  EEPROM.write(25, momentoDir);
                  controlByPass = 1;
                  Serial.println("DEREITA1");
                  MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                  MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
                  CodExecutation = 304; // ##################################################### Funcao direito 1/2 Executada 
  
            } else {
                      if (dir == false || momentoDir == 2) {
                      
                      vel = 2000;  
                      acceleration = 990;
                  
                      stepper.setMaxSpeed(vel);
                      stepper.setAcceleration(acceleration);
                      stepper.moveTo(ToGo);
                      
                      tempoanteriorGiro = millis();
                      tempobotao = millis();
                      dir = true;  EEPROM.write(11, dir);
                      esq = false;  EEPROM.write(12, esq);
                        if (contador == 1){
                          dirStatus = dir;  EEPROM.write(13, dirStatus);
                          esqStatus = esq;  EEPROM.write(14, esqStatus);
                          control = 1;  EEPROM.write(19, control);
                        }
                      contador = 0;  EEPROM.write(16, contador);
                      ultimoCmdPan = 1;  EEPROM.write(21, ultimoCmdPan);
                      encStatus = 0;  EEPROM.write(20, encStatus);
                      momentoDir = 2;  EEPROM.write(25, momentoDir);
                      controlByPass = 1;
                      Serial.println("DEREITA2");
                      CodExecutation = 305; // ##################################################### Funcao direito 2/2 Executada
                      MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                      MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
               }
         }
    }
}

void Esquerdo() {
           if (digitalRead(FC_A) == LOW) {
                if (volta == true){

                    vel = 2000;  
                    acceleration = 990;
                  
                    stepper.setMaxSpeed(vel);
                    stepper.setAcceleration(acceleration);
                    stepper.moveTo(ToGo);

                    tempoanteriorGiro = millis();
                    tempobotao = millis();
                    esq = true;  EEPROM.write(12, esq);
                    dir = false;  EEPROM.write(11, dir);
                      if (contador == 1){
                          dirStatus = dir;  EEPROM.write(13, dirStatus);
                          esqStatus = esq;  EEPROM.write(14, esqStatus);
                          control = 0;  EEPROM.write(19, control);
                      }
                    contador = 0;  EEPROM.write(16, contador);
                    ultimoCmdPan = 2;  EEPROM.write(21, ultimoCmdPan);
                    encStatus = 0;  EEPROM.write(20, encStatus);
                    momentoEsc = 1;  EEPROM.write(26, momentoEsc);
                    controlByPass = 1;
                    Serial.println("ESQUERDA1");
                    MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                    MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
                    CodExecutation = 306; // ##################################################### Funcao Esquerdo 1/2 Executada 
         
                } else {
                     if (esq == false || momentoEsc == 2) {

                        vel = 2000;
                        acceleration = 990;

                        stepper.setMaxSpeed(vel);
                        stepper.setAcceleration(acceleration);
                        stepper.moveTo(ToGo);

                        tempoanteriorGiro = millis();
                        tempobotao = millis();
                        esq = true;  EEPROM.write(12, esq);
                        dir = false;  EEPROM.write(11, dir);
                          if (contador == 1){
                              dirStatus = dir;  EEPROM.write(13, dirStatus);
                              esqStatus = esq;  EEPROM.write(14, esqStatus);
                              control = 1;  EEPROM.write(19, control);
                          }
                        contador = 0;  EEPROM.write(16, contador);
                        ultimoCmdPan = 2;  EEPROM.write(21, ultimoCmdPan);
                        encStatus = 0;  EEPROM.write(20, encStatus);
                        momentoEsc = 2;  EEPROM.write(26, momentoEsc);
                        controlByPass = 1;
                        Serial.println("ESQUERDA2");
                        MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                        MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
                        CodExecutation = 307; // ##################################################### Funcao Esquerdo 2/2 Executada             
            } 
        }
    }
}     

void PararControl() {

         stepper.stop();

         MtiltState1 = HIGH;  //EEPROM.write(30, MtiltState1);                             
         MtiltState2 = HIGH;  //EEPROM.write(31, MtiltState2);                             
         
         digitalWrite(MtiltR, MtiltState1);           // ESCREVE NOS PINOS
         digitalWrite(MtiltL, MtiltState2);           // ESCREVE NOS PINOS

         CTRLposition = 0;

         controlByPass = 0;

         //esq = false;
         //dir = false;
         //dirStatus = dir;
         //esqStatus = esq;

         Serial.print("Position depois do PararControl: ");
         Serial.println(Position);

         estadoCmdTil = 0;  EEPROM.write(23, estadoCmdTil);

         //Serial.println(Position);

         tempobotao = millis();
         
         Serial.println("---- Parando ----");
         CodExecutation = 308; // ##################################################### Funcao parada via Ctr Executada
}

void PararEnc() {
        
        vel = 0;  //EEPROM.write(10, vel);
        stepper.setSpeed(vel);
        stepper.stop();

         controleoff2 = 10;  EEPROM.write(18, controleoff2);

         momentoEsc = 10;  EEPROM.write(26, momentoEsc);
         momentoDir = 10;  EEPROM.write(25, momentoDir);
 
         CTRLposition = 0;

         controlByPass = 0;
         //dirStatus = dir;
         //esqStatus = esq;
         
         Serial.print("Position depois do PararEnc: ");  
         Serial.println(Position);

         if (dir == dirStatus || esq == esqStatus) {
         volta = !volta;  EEPROM.write(15, volta);
         //EEPROM.write(5,volta);
            }
            
      if (volta == true) {
                esq = false;  EEPROM.write(12, esq);
                dir = false;  EEPROM.write(11, dir);
                esqStatus = false;  EEPROM.write(14, esqStatus);
                dirStatus = false;  EEPROM.write(13, dirStatus);

                encStatus = 1;  EEPROM.write(20, encStatus);
                stepper.setCurrentPosition(0);                                         //Position recebe o valor zero sempre que o flap estiver no centro verdadeiro
            }
         contador = 1;  EEPROM.write(16, contador);
         CodExecutation = 309; // ##################################################### Funcao Parada via Encoder executada
}
// --------------------------------------------------------------------------------------------------------------------------------------------------------