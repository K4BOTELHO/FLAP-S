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
#define dirPin2 8                      // 
#define enablePan 10                   // 
#define SF 11                          //    

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
int contador;                           // EEPROM 16 - Velocidade do giro
int controleoff;                        // EEPROM 17 - Velocidade do giro
int controleoff2;                       // EEPROM 18 - Velocidade do giro
int control;                            // EEPROM 19 - Velocidade do giro
int encStatus;                          // EEPROM 20 - Velocidade do giro
int ultimoCmdPan;                       // EEPROM 21 - Velocidade do giro
int estadoCmdPan;                       // EEPROM 22 - Velocidade do giro
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
int RECV_PIN = 16;                      // Pino IR
int pinoSensor = 9;                     // PINO DIGITAL UTILIZADO PELO SENSOR ENCODER

float armazenavalor;                    // Variavel valor do IR

// ---- IR ------------------------------------------------------------------ IR ------------------------------------------------------------------ IR ----

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

  pinMode(pinoSensor, INPUT);          //DEFINE O PINO DO SENSOR ENCODER COMO ENTRADA
  pinMode(enablePan, INPUT);           //DEFINE O PINO DO SENSOR ENCODER COMO ENTRADA
  pinMode(SF, INPUT);                  //DEFINE O PINO DO SENSOR ENCODER COMO ENTRADA

  digitalWrite(MtiltR, MtiltState1);   // Define o estado inicial
  digitalWrite(MtiltL, MtiltState2);   // Define o estado inicial
  
  irrecv.enableIRIn();                 // Inicializa o receptor IR

  mySwitch.enableReceive(0);           // Receiver on interrupt 0 => that is pin #2

  stepper.setMaxSpeed(10000);          //Stepper Motor
  stepper.setSpeed(0);
  stepper.setAcceleration(500);
  stepper.setCurrentPosition(0); 
  pinMode(dirPin, OUTPUT);             // Direcao do Motor aux
  pinMode(stepPin, OUTPUT);            // Define os pinos dos motores como saida
  pinMode(dirPin2, OUTPUT);            // Define os pinos dos motores como saida
  
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
  estadoCmdPan = EEPROM.read(22);
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

// ---- Loop do RF ------------------------------------------------------ Loop do RF ------------------------------------------------------ Loop do RF ----

    if (mySwitch.available()) {

    int value = mySwitch.getReceivedValue();
     if (value == 0){
     Serial.print("Codigo desconhecido");
     CodCmd = 101; // ##################################################### Codigo RF Desconhecido
     } else {
      
      if (mySwitch.getReceivedValue() == 1200021) {             //Primeiro Botao
           tempobotao = millis();
           delay(200);

           if (digitalRead(enablePan) == LOW) {
                Subir();
                Serial.print("Subir via RF");
           } else {

                 if (estadoCmdTil == 0) {
                       if (ultimoCmdTil == 1) {
                             Descer();
                             Serial.print("Descer via RF");
                             CodCmd = 102; // ##################################################### Descer via RF
                       }else if (ultimoCmdTil == 2) {
                             Subir();
                             Serial.print("Subir via RF");
                             CodCmd = 103; // ##################################################### Subir via RF 
                       }
                  }
                  else {
                     PararControl();
                     Serial.print("Parar via RF");
                     CodCmd = 104; // ##################################################### Parar via RF
                 }
          }
  }   
       else if (mySwitch.getReceivedValue() == 1200022) {       //Segundo Botao
             tempobotao = millis();
             delay(200);

    if (vel == 0) {
               if (ultimoCmdPan == 1) {
                      if (digitalRead(pinoSensor) == LOW) {
                            if (volta == 0) {
                              Esquerdo();
                              Serial.print("Esquerdo via RF");
                            } else {
                              Direito();
                              Serial.print("Direito via RF");
                            }
                      }else {
                        Esquerdo();
                        Serial.print("Esquerdo via RF");
                      }
                 CodCmd = 105; // ##################################################### Esquerdo via RF
                 
               }else if (ultimoCmdPan == 2) {
                       if (digitalRead(pinoSensor) == LOW) {
                            if (volta == 0){
                              Direito();
                              Serial.print("Direito via RF");
                            } else {
                              Esquerdo();
                              Serial.print("Esquerdo via RF");
                            }
                       }else {
                        Direito();
                        Serial.print("Direito via RF");
                       }
                 CodCmd = 106; // ##################################################### Direito via RF 
               } 
    }
             else {
               PararControl();
               Serial.print("Parar via RF");
               CodCmd = 107; // ##################################################### Parar via RF
           }             
       }
   }
         
    Serial.print("Received ");
    Serial.println( mySwitch.getReceivedValue() );
    
    mySwitch.resetAvailable();
  }
  
  
// ---- Loop do IR ----------------------------------------------------- Loop do IR ------------------------------------------------------- Loop do IR ----

 if (millis() - tempobotao >= 1500){

  if (irrecv.decode(&results)) {
    
    Serial.print("Valor lido : ");
    Serial.println(results.value, HEX);
    armazenavalor = (results.value);
    if (armazenavalor == 0x202D02F || armazenavalor == 0x76B366E3)       //Verifica se a tecla DESCER foi acionada
    {
      tempobotao = millis();
      Subir();
      CodCmd = 108; // ##################################################### Subir via IR
    }
    else if (armazenavalor == 0x202708F || armazenavalor == 0x6EDFE961 ) //Verifica se a tecla SUBIR foi acionada
    {
      tempobotao = millis();
      Descer();
      CodCmd = 109; // ##################################################### Descer via IR
    }
    else if (armazenavalor == 0x2028877 || armazenavalor == 0x406F92E7)  //Verifica se a tecla DIREITO foi acionada
    {
      tempobotao = millis();
      Direito();
      CodCmd = 110; // ##################################################### Direito via IR
    }
    else if (armazenavalor == 0xE1C78121 || armazenavalor == 0x20208F7)  //Verifica se a tecla ESQUERDO foi acionada
    { 
      tempobotao = millis();
      Esquerdo();
      CodCmd = 111; // ##################################################### Esquerdo via IR
    }
    else if (armazenavalor == 0x923F150B || armazenavalor == 0x202B04F)  //Verifica se a tecla PARAR foi acionada
    { 
      tempobotao = millis();
      PararControl();
      CodCmd = 112; // ##################################################### Parar via IR
    }
    else if (armazenavalor == 0x202B24D || armazenavalor == 0x203E1DBF)  //Verifica se a tecla POWER OFF foi acionada
    {
      tempobotao = millis();
      if (dirStatus == true && esqStatus == false){
          if (control == 0) {
              controleoff = 10; EEPROM.write(17, controleoff);
              Esquerdo();
              CodCmd = 113; // ##################################################### Esquerdo via Power IR
          } else {
              controleoff = 10; EEPROM.write(17, controleoff);
              Direito();
              CodCmd = 114; // ##################################################### Direito via Power IR
          }
          
      }else if (dirStatus == false && esqStatus == true){
              if (control == 0) {
                  controleoff = 10; EEPROM.write(17, controleoff);
                  Direito();
                  CodCmd = 115; // ##################################################### Direito via Power IR
          } else {
                  controleoff = 10; EEPROM.write(17, controleoff);
                  Esquerdo();
                  CodCmd = 116; // ##################################################### Esquerdo via Power IR
         }
    }
      Subir();      
}




    irrecv.resume(); //Le o próximo valor
}
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
              
                    Direito();
                    CodCmd = 120; // ##################################################### Direito via Serial
                                         
            } else if (letra == 'a'){
                    Esquerdo();
                    CodCmd = 121; // ##################################################### Esquerdo via Serial
            } 
}

    if (contador == 0){
        if (millis() - tempoanteriorGiro >= 1300){         //750
         
            if (digitalRead(pinoSensor) == LOW){           // No centro -> Posicao correta
                               
                PararEnc();
                CodIFs = 201; // ##################################################### IF do Encoder
         }
      }
   } 

   if (controleoff == 10 && controleoff2 == 10 && volta == true){
            if (digitalRead(pinoSensor) == LOW){           // No centro -> Posicao correta
              
                Subir();
                CodIFs = 202; // ##################################################### IF da subida automatica
         }        
      }
if (vel == 0) {
   if (millis() - TempoAbreFecha >= 55000){                //750
            PararControl();
            CodIFs = 204;     // ##################################################### IF do Encoder
   }
}

if (millis() - tempoanteriorGiro <= 1500){
      encStatus = 0; //EEPROM.update(20, encStatus);
}


if (digitalRead(pinoSensor) == LOW) {
      encStatus = 1; EEPROM.update(20, encStatus);
}
            
   /*if (controleoff == 0) {
       if (Position == 62500 || Position == -62500){
          PararControl();
          CodIFs = 203;       // ##################################################### IF da posicao de 90 graus
       }
   }*/

        stepper.enableOutputs(); //enable pins
        stepper.setSpeed(vel);
        stepper.runSpeed();     

        digitalWrite(MtiltR, MtiltState1);       // ESCREVE NOS PINOS
        digitalWrite(MtiltL, MtiltState2);       // ESCREVE NOS PINOS
      
// --------------------------------------------------------------------------------------------------------------------------------------------------------

         //Serial.println(digitalRead(enablePan));
         //Serial.println(digitalRead(pinoSensor));
         //Serial.println(digitalRead(SF));
         //digitalWrite(13, volta);
        //Position = stepper.currentPosition();
         
         //Serial.print(CodCmd);
         //Serial.print(CodIFs);
        // Serial.println(CodExecutation );
         //Serial.println(volta);
}
/*
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
             
             Serial.println("SUBINDO");
             CodExecutation = 301; // ##################################################### Função subir - Executada
         } else {
             Serial.println("Subida nao permitida");
             CodExecutation = 302; // ##################################################### Função subir - Executada mas não permitida
         }
}

void Descer() {
       if (digitalRead(enablePan) == HIGH){
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
    }
}

void Direito() {
       if (digitalRead(enablePan) == LOW) {
            if (volta == true) {
                  tempoanteriorGiro = millis();
                  vel = -2500;  //EEPROM.write(10, vel);
                  digitalWrite(dirPin2,HIGH); 
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
                  Serial.println("DEREITA");
                  MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                  MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
                  CodExecutation = 304; // ##################################################### Funcao direito 1/2 Executada 
  
            } else {
                      if (dir == false || momentoDir == 2) {
                      tempoanteriorGiro = millis();
                      vel = -2500;  //EEPROM.write(10, vel);
                      digitalWrite(dirPin2,HIGH); 
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
                      Serial.println("DEREITA");
                      CodExecutation = 305; // ##################################################### Funcao direito 2/2 Executada
                      MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                      MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
               }
         }
    }
}
void Esquerdo() {
           if (digitalRead(enablePan) == LOW) {
                if (volta == true){
                    tempoanteriorGiro = millis();
                    vel = 2500;  //EEPROM.write(10, vel);
                    digitalWrite(dirPin2,LOW); 
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
                    Serial.println("ESQUERDA");
                    MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                    MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
                    CodExecutation = 306; // ##################################################### Funcao Esquerdo 1/2 Executada 
         
                } else {
                     if (esq == false || momentoEsc == 2) {
                        tempoanteriorGiro = millis();
                        vel = 2500;  //EEPROM.write(10, vel);
                        digitalWrite(dirPin2,LOW); 
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
                        Serial.println("ESQUERDA");
                        MtiltState1 = HIGH;   //EEPROM.write(30, MtiltState1);
                        MtiltState2 = HIGH;   //EEPROM.write(31, MtiltState2);
                        CodExecutation = 307; // ##################################################### Funcao Esquerdo 2/2 Executada             
            } 
        }
    }
}     

void PararControl() {

         MtiltState1 = HIGH;  //EEPROM.write(30, MtiltState1);                             
         MtiltState2 = HIGH;  //EEPROM.write(31, MtiltState2);                             
         
         digitalWrite(MtiltR, MtiltState1);           // ESCREVE NOS PINOS
         digitalWrite(MtiltL, MtiltState2);           // ESCREVE NOS PINOS

         //esq = false;
         //dir = false;
          
         //dirStatus = dir;
         //esqStatus = esq;
         
         vel = 0;  //EEPROM.write(10, vel);

         estadoCmdTil = 0;  EEPROM.write(23, estadoCmdTil);

         //Position = stepper.currentPosition();
         //Serial.println(Position);
         //stepper.setCurrentPosition(0);
         stepper.setCurrentPosition(0);
         
         Serial.println("---- Parando ----");
         CodExecutation = 308; // ##################################################### Funcao parada via Ctr Executada
}

void PararEnc() {
    
         controleoff2 = 10;  EEPROM.write(18, controleoff2);

         momentoEsc = 10;  EEPROM.write(26, momentoEsc);
         momentoDir = 10;  EEPROM.write(25, momentoDir);
 
         //dirStatus = dir;
         //esqStatus = esq;
         
         vel = 0;  //EEPROM.write(10, vel);     
         //Position = stepper.currentPosition();
         //Serial.println(Position);
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
            }
         contador = 1;  EEPROM.write(16, contador);
         CodExecutation = 309; // ##################################################### Funcao Parada via Encoder executada
}
// --------------------------------------------------------------------------------------------------------------------------------------------------------