/*
 An example analogue clock using a TFT LCD screen to show the time
 use of some of the drawing commands with the library.

 For a more accurate clock, it would be better to use the RTClib library.
 But this is just a demo.

 This sketch uses font 4 only.

 Make sure all the display driver and pin comnenctions are correct by
 editting the User_Setup.h file in the TFT_eSPI library folder.

 #########################################################################
 ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
 #########################################################################

 Based on a sketch by Gilchrist 6/2/2014 1.0
 */

#include "config.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>




#define VERSAO "0.2"
//#define BOTAO_POWER 36
#define LIMITCLICK 60


#define INTERVALO_10_SEGUNDOS 10000000LL // 10 segundos em microssegundos

TTGOClass *watch = nullptr;
TFT_eSPI *tft;
PCF8563_Class *rtc;

bool lenergy = false;
bool flgpower = false;
bool flgbutton = false;


bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t interval = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define TFT_GREY            0x5AEB

//Maquina de estado do relogio
typedef enum 
{
  EN_INICIO,  //Inicialização de setup
  EN_SETCLOCK, //Configura o relogio  
  EN_WATCH01, //Mostrando o display
  EN_REPOUSO, //Em repouso   
  EN_FIM      //Fim de opcoes
} Estado;

//Maquina de estado do relogio
typedef enum 
{
  TC_NONE,
  TC_MOVELEFT,  //Arrasta para a esquerda
  TC_MOVERIGHT, //Arrasta para Direita  
  TC_MOVEUP, //Arrasta para Cima
  TC_MOVEDOWN, //Arrasta para Baixo  
  TC_CLICK      //CLick
} MOVETOUCH;

typedef struct
{
  Estado estado_atual;
} MaquinaEstado;

/*Estrutura de estado do touch*/
typedef struct
{
   int16_t xIn;
   int16_t yIn;
   int16_t xOut;
   int16_t yOut; 
   bool flgtouch;
   MOVETOUCH move;
} TouchEstado;

//Variavies globais
TTGOClass *ttgo;
MaquinaEstado maquina;

TouchEstado touch;

#define TFT_GREY 0x5AEB

bool irq;

float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg = 0, mdeg = 0, hdeg = 0;
uint16_t osx = 120, osy = 120, omx = 120, omy = 120, ohx = 120, ohy = 120; // Saved H, M, S x & y coords
uint16_t x0 = 0, x1 = 0, yy0 = 0, yy1 = 0;
uint32_t targetTime = 0;                    // for next 1 second timeout

static uint8_t conv2d(const char *p); // Forward declaration needed for IDE 1.6.x
uint8_t hh = conv2d(__TIME__), mm = conv2d(__TIME__ + 3), ss = conv2d(__TIME__ + 6); // Get H, M, S from compile time

boolean initial = 1;

int64_t tempo_inicio = esp_timer_get_time();
int64_t tempo_atual;

/*Declarações de funcoes*/
void Start_Relogio();
void normal_energy();
void low_energy();
void MudaEstado(MaquinaEstado *maquina1, Estado valor);
void proximoEstado(MaquinaEstado *maquina1);
void printxy(int x,int y, char *info);
void drawSTATUS(bool status);
void AnalisaTouch();

bool setDateTimeFormBLE(const char *str)
{
    uint16_t year;
    uint8_t month,  day, hour,  min,  sec;
    String temp, data;
    int r1, r2;
    if (str == NULL)return false;

    data = str;

    r1 = data.indexOf(',');
    if (r1 < 0)return false;
    temp = data.substring(0, r1);
    year = (uint16_t)temp.toInt();

    r1 += 1;
    r2 = data.indexOf(',', r1);
    if (r2 < 0)return false;
    temp = data.substring(r1, r2);
    month = (uint16_t)temp.toInt();

    r1 = r2 + 1;
    r2 = data.indexOf(',', r1);
    if (r2 < 0)return false;
    temp = data.substring(r1, r2);
    day = (uint16_t)temp.toInt();

    r1 = r2 + 1;
    r2 = data.indexOf(',', r1);
    if (r2 < 0)return false;
    temp = data.substring(r1, r2);
    hour = (uint16_t)temp.toInt();

    r1 = r2 + 1;
    r2 = data.indexOf(',', r1);
    if (r2 < 0)return false;
    temp = data.substring(r1, r2);
    min = (uint16_t)temp.toInt();

    r1 = r2 + 1;
    temp = data.substring(r1);
    sec = (uint16_t)temp.toInt();

    // No parameter check, please set the correct time
    Serial.printf("SET:%u/%u/%u %u:%u:%u\n", year, month, day, hour, min, sec);
    rtc->setDateTime(year, month, day, hour, min, sec);

    return true;
}

class MyCallbacks: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string value = pCharacteristic->getValue();

        if (value.length() > 0) {
            Serial.println("*********");
            Serial.print("New value: ");
            value.c_str();
            for (int i = 0; i < value.length(); i++)
                Serial.print(value[i]);
            Serial.println();
            Serial.println("*********");
        }

        if (value.length() <= 0) {
            return;
        }
        if (!setDateTimeFormBLE(value.c_str())) {
            Serial.println("DateTime format error ...");
        }
    }
};

class MyServerCallback : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("onConnect");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("onDisconnect");
    }
};

void setupBLE(void)
{

    BLEDevice::init("LilyGo-Watch");
    BLEServer *pServer = BLEDevice::createServer();

    pServer->setCallbacks(new MyServerCallback);

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic->setCallbacks(new MyCallbacks());

    pCharacteristic->setValue("Format: YY,MM,DD,h,m,s");
    pService->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
}


/*Imprime na tela*/
void printxy(int x,int y, char *info)
{
    //ttgo->tft->fillScreen(TFT_BLACK);
    watch->tft->fillScreen(TFT_BLACK);
    //ttgo->tft->drawString(info,  x, y, 4);
    watch->tft->drawString(info,  x, y, 4);
    //ttgo->tft->setTextFont(4);
    watch->tft->setTextFont(4);
    //ttgo->tft->setTextColor(TFT_WHITE, TFT_BLACK);
    watch->tft->setTextColor(TFT_WHITE, TFT_BLACK);
}

void normal_energy()
{
  //ttgo->openBL();
  watch->openBL();

}
void low_energy()
{
  //ttgo->closeBL();         
  watch->closeBL();         
}

static uint8_t conv2d(const char *p)
{
    uint8_t v = 0;
    if ('0' <= *p && *p <= '9')
        v = *p - '0';
    return 10 * v + *++p - '0';
}


//Acende display
void AcendeDisplay()
{
  normal_energy();
  //ttgo->openBL();
}

void ApagaDisplay()
{
  low_energy();
  //ttgo->closeBL();
}


void Start_Serial()
{
  Serial.begin(115200);
}

void Start_definicoes()
{
  irq = false;
  maquina.estado_atual = EN_INICIO; //Maquina de estado
  Serial.println("Iniciou definicoes");

  //Touch Screen
  touch.xIn = 0;
  touch.xOut = 0;
  touch.yIn = 0; 
  touch.yOut = 0; 
  touch.flgtouch = false;
  touch.move = TC_NONE;
}

void Start_tft()
{
    Serial.println("Iniciou TTF");
    watch = TTGOClass::getWatch();
    watch->begin();
    watch->openBL();
    //ttgo = TTGOClass::getWatch();
    //ttgo->begin();
    //ttgo->openBL();
    tft = watch->tft;

    //ttgo->tft->fillScreen(TFT_BLACK);
    watch->tft->fillScreen(TFT_BLACK);
    //ttgo->tft->setTextColor(TFT_WHITE, TFT_BLACK);  // Adding a background colour erases previous text automatically  
    watch->tft->setTextColor(TFT_WHITE, TFT_BLACK);  // Adding a background colour erases previous text automatically  
   
}

void Start_Power()
{
    Serial.println("Start Power");
    pinMode(AXP202_INT, INPUT_PULLUP);
    attachInterrupt(AXP202_INT, [] {
        irq = true;
        Serial.print("Atribui valor irq");
    }, FALLING);

    //!Clear IRQ unprocessed  first
    //ttgo->power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ | AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_CHARGING_IRQ, true);
    watch->power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ | AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_CHARGING_IRQ, true);
    //ttgo->power->clearIRQ();
    watch->power->clearIRQ();


}

void proximoEstado(MaquinaEstado *maquina1) 
{
  /*
    Estado proximo;
    proximo = (Estado)(maquina1->estado_atual + 1) % EN_FIM;
    if((proximo && EN_FIM)==0)
    {
      //maquina->estado_atual = proximo; /*Muda o estado da maquina
      MudaEstado(maquina1, proximo); 
    }
    */
}

void MudaEstado(MaquinaEstado *maquina1, Estado valor)
{
  Serial.print("Recebeu:");
  Serial.println(valor);
  maquina1->estado_atual = (Estado)valor;
  /*Inicializa estado atual*/
  if(maquina1->estado_atual == EN_WATCH01)
  {
    Serial.println("Mudou estado:EN_WATCH01");
    watch->tft->fillScreen(TFT_BLACK); 
    tempo_inicio = esp_timer_get_time(); /*Ajusta a hora para agora*/
    //Start_Relogio();
    AcendeDisplay();
  } else
  if(maquina1->estado_atual == EN_REPOUSO)
  {
    Serial.println("Mudou estado:EN_REPOUSO");
    
    //tempo_inicio = esp_timer_get_time(); /*Ajusta a hora para agora*/
    ApagaDisplay();
    
  }  else
  if(maquina1->estado_atual == EN_INICIO)
  {
    Serial.println("Mudou estado:EN_INICIO");
    
    tempo_inicio = esp_timer_get_time(); /*Ajusta a hora para agora*/
    Start_Relogio();
  } else 
  if(maquina1->estado_atual == EN_SETCLOCK)
  {
    watch->tft->fillScreen(TFT_BLACK);
    drawSTATUS(false);
  }
}


void Start_Relogio()
{
  // Draw clock face
    //ttgo->tft->fillCircle(120, 120, 118, TFT_WHITE);
    watch->tft->fillCircle(120, 120, 118, TFT_WHITE);
    //ttgo->tft->fillCircle(120, 120, 110, TFT_BLACK);
    watch->tft->fillCircle(120, 120, 110, TFT_BLACK);

    // Draw 12 lines
    for (int i = 0; i < 360; i += 30) {
        sx = cos((i - 90) * 0.0174532925);
        sy = sin((i - 90) * 0.0174532925);
        x0 = sx * 114 + 120;
        yy0 = sy * 114 + 120;
        x1 = sx * 100 + 120;
        yy1 = sy * 100 + 120;

        //ttgo->tft->drawLine(x0, yy0, x1, yy1, TFT_RED);
        watch->tft->drawPixel(x0, yy0, TFT_WHITE);
    }

    // Draw 60 dots
    for (int i = 0; i < 360; i += 6) {
        sx = cos((i - 90) * 0.0174532925);
        sy = sin((i - 90) * 0.0174532925);
        x0 = sx * 102 + 120;
        yy0 = sy * 102 + 120;
        // Draw minute markers
        //ttgo->tft->drawPixel(x0, yy0, TFT_WHITE);
        watch->tft->drawPixel(x0, yy0, TFT_WHITE);
        // Draw main quadrant dots
        if (i == 0 || i == 180) //ttgo->tft->fillCircle(x0, yy0, 2, TFT_WHITE);
          watch->tft->fillCircle(x0, yy0, 2, TFT_WHITE);
        if (i == 90 || i == 270) //ttgo->tft->fillCircle(x0, yy0, 2, TFT_WHITE);
          watch->tft->fillCircle(x0, yy0, 2, TFT_WHITE);
    }

    //ttgo->tft->fillCircle(120, 121, 3, TFT_WHITE);
    watch->tft->fillCircle(120, 121, 3, TFT_WHITE);

    // Draw text at position 120,260 using fonts 4
    // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . - a p m
    // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
    // ttgo->tft->drawCentreString("Time flies", 120, 260, 4);
    targetTime = millis() + 1000;
}


void Wellcome()
{
  
  Serial.println("Maurinsoft - Firmware ");
  Serial.print("Versão:");
  Serial.println(VERSAO); 
  
}

void Start_Clock()
{
    //  Receive as a local variable for easy writing
    //rtc = ttgo->rtc;
    rtc = watch->rtc;
    //tft = ttgo->tft;
    tft = watch->tft;
    // Time check will be done, if the time is incorrect, it will be set to compile time
    rtc->check();

    // Some settings of BLE
    setupBLE();

}

void Start_Touch()
{
    // Use touch panel interrupt as digital input
    pinMode(TOUCH_INT, INPUT);
}

void setup(void)
{
    Start_Serial();    
    Start_definicoes(); //Iniciando definicoes de ambiente    
    Wellcome();
    
    Start_tft();
    Serial.println("Entra aqui");
    Start_Power();
    Start_Clock();
    Serial.println("Sai aqui");
    MudaEstado(&maquina, EN_WATCH01);   
    Serial.println("Finalizou Setup");   
}


void LePower()
{
    //Serial.println(irq); 
    if(irq) 
    {
        irq = false;
        //ttgo->power->readIRQ();
        watch->power->readIRQ();
        //if (ttgo->power->isVbusPlugInIRQ()) {
        if (watch->power->isVbusPlugInIRQ()) {
            //ttgo->tft->fillRect(20, 100, 200, 85, TFT_BLACK);
            //ttgo->tft->drawString("Power Plug In", 25, 100);
            Serial.println("Power Plug In");
            flgpower = true;
        }
        //if (ttgo->power->isVbusRemoveIRQ()) {
        if (watch->power->isVbusRemoveIRQ()) {          
            //ttgo->tft->fillRect(20, 100, 200, 85, TFT_BLACK);
            //ttgo->tft->drawString("Power Remove", 25, 100);
            Serial.println("Power Remove");
            flgpower = false;
        }
        //if (ttgo->power->isPEKShortPressIRQ()) 
        if (watch->power->isPEKShortPressIRQ())         
        {
            //ttgo->tft->fillRect(20, 100, 200, 85, TFT_BLACK);
            //ttgo->tft->drawString("PowerKey Press", 25, 100);
            Serial.println("PowerKey Press");
            flgbutton = true;
        }
        //ttgo->power->clearIRQ();
        watch->power->clearIRQ();
    }
    
}

void drawSTATUS(bool status)
{
    String str = status ? "Connection" : "Disconnect";
    int16_t cW = tft->textWidth("Connection", 2);
    int16_t dW = tft->textWidth("Disconnect", 2);
    int16_t w = cW > dW ? cW : dW;
    w += 6;
    int16_t x = 160;
    int16_t y = 20;
    int16_t h = tft->fontHeight(2) + 4;
    uint16_t col = status ? TFT_GREEN : TFT_GREY;
    tft->fillRoundRect(x, y, w, h, 3, col);
    tft->setTextColor(TFT_BLACK, col);
    tft->setTextFont(2);
    tft->drawString(str, x + 2, y);
}

void Display_Relogio()
{
    if (targetTime < millis()) 
    {
        targetTime += 1000;
        ss++;              // Advance second
        if (ss == 60) 
        {
            ss = 0;
            mm++;            // Advance minute
            if (mm > 59) 
            {
                mm = 0;
                hh++;          // Advance hour
                if (hh > 23) 
                {
                    hh = 0;
                }
            }
        }

        // Pre-compute hand degrees, x & y coords for a fast screen update
        sdeg = ss * 6;                // 0-59 -> 0-354
        mdeg = mm * 6 + sdeg * 0.01666667; // 0-59 -> 0-360 - includes seconds
        hdeg = hh * 30 + mdeg * 0.0833333; // 0-11 -> 0-360 - includes minutes and seconds
        hx = cos((hdeg - 90) * 0.0174532925);
        hy = sin((hdeg - 90) * 0.0174532925);
        mx = cos((mdeg - 90) * 0.0174532925);
        my = sin((mdeg - 90) * 0.0174532925);
        sx = cos((sdeg - 90) * 0.0174532925);
        sy = sin((sdeg - 90) * 0.0174532925);

        if (ss == 0 || initial) 
        {
            initial = 0;
            // Erase hour and minute hand positions every minute
            //ttgo->tft->drawLine(ohx, ohy, 120, 121, TFT_BLACK);
            watch->tft->drawLine(ohx, ohy, 120, 121, TFT_BLACK);
            ohx = hx * 62 + 121;
            ohy = hy * 62 + 121;
            //ttgo->tft->drawLine(omx, omy, 120, 121, TFT_BLACK);
            watch->tft->drawLine(omx, omy, 120, 121, TFT_BLACK);
            omx = mx * 84 + 120;
            omy = my * 84 + 121;
        }

        // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
        //ttgo->tft->drawLine(osx, osy, 120, 121, TFT_BLACK);
        watch->tft->drawLine(osx, osy, 120, 121, TFT_BLACK);
        osx = sx * 90 + 121;
        osy = sy * 90 + 121;
        //ttgo->tft->drawLine(osx, osy, 120, 121, TFT_RED);
        watch->tft->drawLine(osx, osy, 120, 121, TFT_RED);
        //ttgo->tft->drawLine(ohx, ohy, 120, 121, TFT_WHITE);
        watch->tft->drawLine(ohx, ohy, 120, 121, TFT_WHITE);
        //ttgo->tft->drawLine(omx, omy, 120, 121, TFT_WHITE);
        watch->tft->drawLine(omx, omy, 120, 121, TFT_WHITE);
        //ttgo->tft->drawLine(osx, osy, 120, 121, TFT_RED);
        watch->tft->drawLine(osx, osy, 120, 121, TFT_RED);
        //ttgo->tft->fillCircle(120, 121, 3, TFT_RED);
        watch->tft->fillCircle(120, 121, 3, TFT_RED);
    }
}

void Le_Touch()
{
  int16_t x,y;

  // Print touch coordinates to the screen
  if (watch->getTouch(x, y)) 
  {
      if(touch.flgtouch == TRUE)
      {
        //snprintf(buf, 64, "X:%03d Y:%03d", x, y);
        //tft->drawCentreString(buf, 120, 120, 2);
        touch.xOut = x;
        touch.yOut = y;
      }
      else
      {
        touch.xIn = x;
        touch.yIn = y;  
        touch.move = TC_NONE;      
        touch.flgtouch = TRUE; /*Seta valor*/
        //Serial.print("XIN:");
        //Serial.println(touch.xIn);
        //Serial.print("YIN:");
        //Serial.println(touch.yIn);        
      }
   } else 
   {
      if (touch.flgtouch)
      {
        //Chama evento de retirar Analisar touch
        touch.flgtouch = FALSE;
        
        //Serial.print("X_Out:");
        //Serial.println(touch.xOut);
        //Serial.print("Y_Out:");
        //Serial.println(touch.yOut);
        AnalisaTouch();
        
      }
   }

}

void Leituras()
{
   LePower(); /*Le funcionalidades de interrupcao AXP202 */
   Le_Touch();
       
 
}

void MedeTempo()
{
  /*Somente muda se diferente de repouso*/
  if(maquina.estado_atual != EN_REPOUSO)
  {
        tempo_atual = esp_timer_get_time();
        int64_t tempo_decorrido = tempo_atual - tempo_inicio;

        if (tempo_decorrido >= INTERVALO_10_SEGUNDOS) 
        {
            // 10 segundos se passaram
            //printf("10 segundos se passaram.\n");
            //tempo_inicio = tempo_atual; // Reiniciar a contagem
            Serial.println("MedeTempo em Repouso");
            
            
            MudaEstado(&maquina, EN_REPOUSO); //Muda o estado da maquina de estado
        }

        //vTaskDelay(pdMS_TO_TICKS(100)); // Adicionar um pequeno atraso para reduzir a carga da CPU
  }
}

void AnalisaTouch()
{
  int64_t descx;
  int64_t descy;
  //Nao implementado
  descx = touch.xOut - touch.xIn;
  descy = touch.yOut - touch.yIn;

  if(descx<LIMITCLICK&&descx>-LIMITCLICK&&descy<LIMITCLICK&&descy>-LIMITCLICK)
  {
    touch.move = TC_CLICK;
    //Serial.println("CLICK");
  } else
  {
    if(descx<LIMITCLICK&&descx>-LIMITCLICK&&descy<-LIMITCLICK)
    {
      touch.move = TC_MOVEUP;
      //Serial.println("MOVE UP");
    } else {
      if(descx<LIMITCLICK&&descx>-LIMITCLICK&&descy>LIMITCLICK)
      {
        touch.move = TC_MOVEDOWN;
        //Serial.println("MOVE DOWN");
      } else
      {
        if(descy<LIMITCLICK&&descy>-LIMITCLICK&&descx<-LIMITCLICK)
        {
          touch.move = TC_MOVELEFT;
          //Serial.println("MOVE LEFT");
        } else
        {
          if(descy<LIMITCLICK&&descy>-LIMITCLICK&&descx>LIMITCLICK)
          {
            touch.move = TC_MOVERIGHT;
            //Serial.println("MOVE RIGHT");
          }
        }
      }
    }  
  }
  
}

//Analisa sinais obtidos de leituras
void Analisa()
{
  // EN_SETCLOCK, //Configura o relogio  
  //EN_WATCH01, //Mostrando o display
  if(flgbutton) /*Controle de pressionar botao*/
  {
         /*Desliga ou liga a tela*/  
         if(maquina.estado_atual == EN_WATCH01)
         {
                  MudaEstado(&maquina, EN_REPOUSO);
                  flgbutton = false;
         } else 
         if(maquina.estado_atual == EN_SETCLOCK)
         {
                  MudaEstado(&maquina, EN_REPOUSO);
                  flgbutton = false;
         } else 
         if(maquina.estado_atual == EN_REPOUSO)
         {
                  MudaEstado(&maquina, EN_WATCH01);
                  flgbutton = false;
         }
   } 
   //Analisa Touch
   if(touch.move != TC_NONE)
   {
     if(touch.move == TC_CLICK)
     {
       Serial.println("Click");
     } else
     if(touch.move == TC_MOVEUP)
     {
       Serial.println("MOVE UP");
       if(maquina.estado_atual == EN_WATCH01)
       {
         MudaEstado(&maquina,EN_SETCLOCK);
       }
     } else 
     if(touch.move == TC_MOVEDOWN)
     {
       Serial.println("MOVE DOWN");
       if(maquina.estado_atual == EN_SETCLOCK)
       {
         MudaEstado(&maquina,EN_WATCH01);
       }
     } else 
     if(touch.move == TC_MOVELEFT)
     {
       Serial.println("MOVE LEFT");
     } else 
     if(touch.move == TC_MOVERIGHT)
     {
       Serial.println("MOVE RIGHT");
     }  

     touch.move = TC_NONE;
   }
}

//Maquina de estado
void EstadoAtual()
{
  //Serial.print("Estado Atual: ");
  //Serial.println(maquina.estado_atual);
  if(maquina.estado_atual == EN_WATCH01)
  {      
      //Serial.print('.');
      Display_Relogio();
      MedeTempo();
  } else
  if(maquina.estado_atual == EN_SETCLOCK)
  {
      // disconnected
    if (!deviceConnected && oldDeviceConnected) 
    {
        oldDeviceConnected = deviceConnected;
        Serial.println("Draw deviceDisconnected");
        drawSTATUS(false);
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) 
    {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        Serial.println("Draw deviceConnected");
        drawSTATUS(true);
    }

    if(maquina.estado_atual != EN_REPOUSO)
    {

        if (millis() - interval > 1000) 
        {
            interval = millis();

            tft->setTextColor(TFT_YELLOW, TFT_BLACK);

            tft->drawString(rtc->formatDateTime(PCF_TIMEFORMAT_DD_MM_YYYY), 50, 200, 4);

            tft->drawString(rtc->formatDateTime(PCF_TIMEFORMAT_HMS), 5, 118, 7);
        }
    }
  }

}

void loop()
{  
  Leituras();
  Analisa();
  //Maquinas de Estado
  EstadoAtual();
}
