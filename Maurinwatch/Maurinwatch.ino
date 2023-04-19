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
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/i2s.h>
#include "esp_sleep.h"
#include <ArduinoJson.h>
#include "crypto/base64.h"
#include <arduino.h>
#include <math.h>



#define BUFFER_SIZE (2*1024)
uint8_t buffer[BUFFER_SIZE] = {0};
uint8_t buffer_len = 0;

// TWATCH 2020 V3 PDM microphone pin
#define MIC_DATA            2
#define MIC_CLOCK           0

#define VERSAO "0.5"
//#define BOTAO_POWER 36
#define LIMITCLICK 60

#define MINIMALCUR 60 /*Corrente minima*/
#define MAXIMALCUR 380 /*Corrente maxima*/

// Tempo de suspensão em microssegundos (1 segundo = 1000000 microssegundos)
uint64_t tempoDeSuspensao = 1000000;
#define INTERVALO_10_SEGUNDOS 10000000LL // 10 segundos em microssegundos

TTGOClass *watch = nullptr;
AXP20X_Class *power  = nullptr;
//TFT_eSPI *tft;
PCF8563_Class *rtc;

lv_obj_t            *chart = nullptr;
lv_chart_series_t   *ser1 = nullptr;
const int           define_max = 200;
const int           define_avg = 15;
const int           define_zero = 3000;

//bool lenergy = false;
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
  EN_SPLASH,  //Splash de inicializaçao
  EN_WIFI,    //Informações sobre wifi
  EN_SETCLOCK, //Configura o relogio  
  EN_WATCH01, //Mostrando o display
  EN_MIC,     //Envia dados do microfone para o chatgpt
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
   bool press;
} TouchEstado;

typedef struct
{
    uint16_t cur; /*Percentual da Bateria*/
    bool flgpower;
} PowerDC;

typedef struct
{
  float               val_avg = 0;
  float               val_avg_1 = 0;
  float               all_val_avg = 0;

  uint8_t             val1, val2;
  int16_t             val16 = 0;
  int16_t             val_max = 0;
  int16_t             val_max_1 = 0;
  int32_t             all_val_zero1 = 0;
  int32_t             all_val_zero2 = 0;
  int32_t             all_val_zero3 = 0;
  uint32_t            j = 0;
} MicVals;


MicVals micval; /*Armazena valores de microfone*/


//Estrutura de Setup do Wifi
typedef struct
{
  char ssid[30];
  char pass[30];
  bool connected;
} SetupWifi;

typedef struct
{
    char googleApiKey;  //googleApiKey = "your_API_KEY";
    char *googleHost; //= "speech.googleapis.com";
    int googlePort;// = 443;
} GOOGLEAPI;

//Estrutura de Setup
typedef struct 
{
  SetupWifi setupwifi;
  PowerDC powerdc; 
} SetupCFG;

//Variavies globais

MaquinaEstado maquina;
GOOGLEAPI googleapi; //Cria variaveis da googleapi

TouchEstado touch;
SetupCFG setupcfg; //Setup geral do Relogio

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
void cls();
void printxy(int x,int y, char *info);
void drawSTATUS(bool status);
void AnalisaTouch();
void MarcaTempoInicio();
void ConnectWifi();
//Verifica se wifi esta conectado
bool WifiConnected();
void ConnectWifi();
void Le_MIC();
void Motor_Toque();



void scanAndDisplayNetworks() 
{
  watch->tft->drawString("Escaneando redes...", 10, 10);
  Serial.println("Escaneando redes...");

  int n = WiFi.scanNetworks(); // Realizar o escaneamento e obter o número de redes encontradas

  if (n == 0) {
    
    cls();
    watch->tft->drawString("Nenhuma rede encontrada", 10, 10);
    Serial.println("Nenhuma rede encontrada");
  } else {
    
    cls();
    watch->tft->drawString((String(n) + " redes encontradas").c_str(), 10, 10);
    Serial.print(n);
    Serial.println(" redes encontradas");

    int y_offset = 30;
    for (int i = 0; i < n; ++i) {
      // Imprimir as informações da rede (SSID, RSSI e criptografia) na tela e no monitor serial
      String networkInfo = String(i + 1) + ": " + WiFi.SSID(i) + " (" + WiFi.RSSI(i) + "dBm)";
      watch->tft->drawString(networkInfo.c_str(), 10, y_offset);
      Serial.println(networkInfo);
      
      y_offset += 20;
      if (y_offset >= watch->tft->height() - 20) {
        // Parar de exibir redes na tela se não houver mais espaço
        break;
      }
    }
  }
}



//Verifica se wifi esta conectado
bool WifiConnected()
{
  if (WiFi.status() == WL_CONNECTED)
  {
     setupcfg.setupwifi.connected = TRUE;
     
     return TRUE;
  }  else
  {
    setupcfg.setupwifi.connected = FALSE;
    return FALSE;
  }  
}

void ConnectWifi()
{
  // Conectar-se à rede Wi-Fi
  WiFi.begin(setupcfg.setupwifi.ssid, setupcfg.setupwifi.pass);

}


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

void Display_Relogio()
{
    if (targetTime < millis()) 
    {
        targetTime += 1000;
        ss++;              // Advance second
        
        RTC_Date dt;
        dt = rtc->getDateTime();
        hh = dt.hour;
        mm = dt.minute;
        ss = dt.second;
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
        
            watch->tft->drawLine(ohx, ohy, 120, 121, TFT_BLACK);
            ohx = hx * 62 + 121;
            ohy = hy * 62 + 121;
        
            watch->tft->drawLine(omx, omy, 120, 121, TFT_BLACK);
            omx = mx * 84 + 120;
            omy = my * 84 + 121;
        }

        // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
        
        watch->tft->drawLine(osx, osy, 120, 121, TFT_BLACK);
        osx = sx * 90 + 121;
        osy = sy * 90 + 121;
        
        watch->tft->drawLine(osx, osy, 120, 121, TFT_RED);
        
        watch->tft->drawLine(ohx, ohy, 120, 121, TFT_WHITE);
        
        watch->tft->drawLine(omx, omy, 120, 121, TFT_WHITE);
        
        watch->tft->drawLine(osx, osy, 120, 121, TFT_RED);
        
        watch->tft->fillCircle(120, 121, 3, TFT_RED);
    }
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

void cls()
{
  watch->tft->fillScreen(TFT_BLACK);
}

/*Imprime na tela*/
void printxy(int x,int y, char *info)
{
    watch->tft->drawString(info,  x, y, 4);
    
    watch->tft->setTextFont(4);
    
    watch->tft->setTextColor(TFT_WHITE, TFT_BLACK);
}

void normal_energy()
{
  watch->openBL();
  //watch->displayWakeup();
  //watch->displayOff();
  power->clearIRQ();

}
void low_energy()
{
  power->clearIRQ();
  //watch->closeBL(); 
  //watch->displaySleep();
  //watch->displayOff();
  //esp_deep_sleep_start();

   // Set screen and touch to sleep mode
    watch->displaySleep();

    /*
    When using T - Watch2020V1, you can directly call power->powerOff(),
    if you use the 2019 version of TWatch, choose to turn off
    according to the power you need to turn off
    */
#ifdef LILYGO_WATCH_2020_V1
    watch->powerOff();
    // LDO2 is used to power the display, and LDO2 can be turned off if needed
    // power->setPowerOutPut(AXP202_LDO2, false);
#else
    power->setPowerOutPut(AXP202_LDO3, false);
    power->setPowerOutPut(AXP202_LDO4, false);
    power->setPowerOutPut(AXP202_LDO2, false);
    // The following power channels are not used
    power->setPowerOutPut(AXP202_EXTEN, false);
    power->setPowerOutPut(AXP202_DCDC2, false);
#endif

    // Use ext0 for external wakeup
    esp_sleep_enable_ext0_wakeup((gpio_num_t)AXP202_INT, LOW);

    // Use ext1 for external wakeup
    //esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);

    esp_deep_sleep_start();

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
  
}

void ApagaDisplay()
{
  low_energy();
  
}


void Start_Serial()
{
  Serial.begin(115200);
}

//Inicia o motor
void Start_Motor()
{
    //! begin motor attach to 33 pin , In TWatch-2020 it is IO4
    watch->motor_begin();
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
  touch.press = FALSE;
   
  //Start Setup 
  memset(setupcfg.setupwifi.ssid,'\0',sizeof(setupcfg.setupwifi.ssid));
  sprintf(setupcfg.setupwifi.ssid,"maurin");
  memset(setupcfg.setupwifi.pass,'\0',sizeof(setupcfg.setupwifi.pass));
  sprintf(setupcfg.setupwifi.pass,"1425361425"); 

  /*Informações google api*/
  memset(&googleapi,'\0',sizeof(GOOGLEAPI));
  sprintf(&googleapi.googleApiKey,"your_API_KEY");
  sprintf(googleapi.googleHost,"speech.googleapis.com");
  googleapi.googleHost = "443";

}

void Start_mic()
{
    cls();
    Serial.println("Iniciou o Start mic");
    watch->lvgl_begin();
    lv_obj_t *text = lv_label_create(lv_scr_act(), NULL);
    Serial.println("Iniciou o Start mic1");
    lv_label_set_text(text, "PDM Microphone Test");
    Serial.println("Iniciou o Start mic1b");
    lv_obj_align(text, NULL, LV_ALIGN_IN_TOP_MID, 0, 20);
    Serial.println("Iniciou o Start mic2");
    chart = lv_chart_create(lv_scr_act(), NULL);
    lv_obj_set_size(chart, 200, 150);
    lv_obj_align(chart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(chart,  LV_CHART_TYPE_LINE);   /*Show lines and points too*/
    lv_chart_set_range(chart, 0, 800);
    Serial.println("Iniciou o Start mic3");
    ser1 = lv_chart_add_series(chart, LV_COLOR_RED);
    Serial.println("Iniciou o Start mic4");
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate =  44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = 128,
    };
    Serial.println("Iniciou o Start mic5");

    i2s_pin_config_t i2s_cfg;
    i2s_cfg.bck_io_num   = I2S_PIN_NO_CHANGE;
    i2s_cfg.ws_io_num    = MIC_CLOCK;
    i2s_cfg.data_out_num = I2S_PIN_NO_CHANGE;
    i2s_cfg.data_in_num  = MIC_DATA;


    Serial.println("Iniciou o Start mic6");
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &i2s_cfg);
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    Serial.println("Fechou o Start mic");
}

void Start_tft()
{
    Serial.println("Iniciou TTF");
    watch = TTGOClass::getWatch();
    watch->begin();
    watch->openBL();
  
    cls();
  
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

    // Habilitar o pino GPIO para acordar o ESP32 quando for HIGH
    esp_sleep_enable_ext0_wakeup((gpio_num_t)AXP202_INT, HIGH);
    power = watch->power;

  #ifdef LILYGO_WATCH_2020_V1
    watch->powerOff();
    // LDO2 is used to power the display, and LDO2 can be turned off if needed
    // power->setPowerOutPut(AXP202_LDO2, false);
#else
    power->setPowerOutPut(AXP202_LDO3, false);
    power->setPowerOutPut(AXP202_LDO4, false);
    power->setPowerOutPut(AXP202_LDO2, false);
    // The following power channels are not used
    power->setPowerOutPut(AXP202_EXTEN, false);
    power->setPowerOutPut(AXP202_DCDC2, false);
    watch->power->setChargeControlCur(MINIMALCUR);
#endif
     // Use ext1 for external wakeup
    //esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);

    //!Clear IRQ unprocessed  first  
    watch->power->enableIRQ(AXP202_PEK_SHORTPRESS_IRQ | AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_CHARGING_IRQ, true);
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

void MarcaTempoInicio()
{
  tempo_inicio = esp_timer_get_time(); /*Ajusta a hora para agora*/
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
  
    cls();
    tempo_inicio = esp_timer_get_time(); /*Ajusta a hora para agora*/
  
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
    
    //tempo_inicio = esp_timer_get_time(); /*Ajusta a hora para agora*/
    MarcaTempoInicio();
    Start_Relogio();
  } else 
  if(maquina1->estado_atual == EN_SETCLOCK)
  {
  
    cls();
    drawSTATUS(false);
  } else
  if(maquina1->estado_atual == EN_WIFI)
  {
  
        cls();
        watch->tft->drawString("Conectado!", 10, 10);
        Serial.println("Conectado!");
         // Imprimir o endereço IP local
        watch->tft->drawString("IP address:", 10, 30);
        watch->tft->drawString(WiFi.localIP().toString().c_str(), 10, 50);
        Serial.print("Endereço IP: ");
        Serial.println(WiFi.localIP());
  } else
  if(maquina1->estado_atual == EN_SPLASH)
  {
    Wellcome();
  } else
  if(maquina1->estado_atual == EN_MIC)
  {
    Start_mic();
  }
}


void Start_Relogio()
{
  // Draw clock face
    watch->tft->fillCircle(120, 120, 118, TFT_WHITE);
  
  
    cls();

    // Draw 12 lines
    for (int i = 0; i < 360; i += 30) {
        sx = cos((i - 90) * 0.0174532925);
        sy = sin((i - 90) * 0.0174532925);
        x0 = sx * 114 + 120;
        yy0 = sy * 114 + 120;
        x1 = sx * 100 + 120;
        yy1 = sy * 100 + 120;

  
        watch->tft->drawPixel(x0, yy0, TFT_WHITE);
    }

    // Draw 60 dots
    for (int i = 0; i < 360; i += 6) {
        sx = cos((i - 90) * 0.0174532925);
        sy = sin((i - 90) * 0.0174532925);
        x0 = sx * 102 + 120;
        yy0 = sy * 102 + 120;
        // Draw minute markers
  
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
  
    targetTime = millis() + 1000;
}


void Wellcome()
{ 
  watch->tft->setTextFont(4);
  watch->tft->drawString("Maurinsoft", 60, 80);
  watch->tft->setTextFont(4);
  watch->tft->drawString("Versao:", 60, 150);
  watch->tft->drawString(VERSAO, 180, 150);
  Serial.println("Maurinsoft - Firmware ");
  Serial.print("Versão:");
  Serial.println(VERSAO); 
  
  //sleep(4);
  
}

void Start_Clock()
{
    //  Receive as a local variable for easy writing
  
    rtc = watch->rtc;
    rtc->check();

    // Some settings of BLE
    setupBLE();

}

void Start_Touch()
{
    // Use touch panel interrupt as digital input
    pinMode(TOUCH_INT, INPUT);
    // Habilitar o pino GPIO para acordar o ESP32 quando for HIGH
    esp_sleep_enable_ext0_wakeup((gpio_num_t)TOUCH_INT, HIGH);
  
}


void Start_Wifi()
{
 ConnectWifi(); 
}

void Motor_Toque()
{
   watch->motor->onec();
}

void setup(void)
{
    Start_Serial();    
    Start_definicoes(); //Iniciando definicoes de ambiente    
  
    
    
    Start_tft();
    Serial.println("Entra aqui");
    Start_Power();
    Start_Clock();
    Start_Wifi();
    Start_Motor();
    
    Serial.println("Sai aqui");
    
    Serial.println("Finalizou Setup");   
    MudaEstado(&maquina, EN_SPLASH); 
    MudaEstado(&maquina, EN_WATCH01); 
    Motor_Toque();  

}


void LePower()
{
  
    if(irq) 
    {
        irq = false;
  
        watch->power->readIRQ();
  
        if (watch->power->isVbusPlugInIRQ()) {
            Serial.println("Power Plug In");
            setupcfg.powerdc.flgpower = true;
            //setupcfg.powerdc.cur = ((watch->power->getChargeControlCur()*100) / MAXIMALCUR);
        }
  
        if (watch->power->isVbusRemoveIRQ()) {          
            Serial.println("Power Remove");
            setupcfg.powerdc.flgpower = false;
            //setupcfg.powerdc.cur =  ((watch->power->getChargeControlCur()*100) / MAXIMALCUR);
        }
        //if (ttgo->power->isPEKShortPressIRQ()) 
        if (watch->power->isPEKShortPressIRQ())         
        {
            Serial.println("PowerKey Press");
            flgbutton = true;
        }
  
        watch->power->clearIRQ();
    }
    
}

void drawSTATUS(bool status)
{
    String str = status ? "Connection" : "Disconnect";
    int16_t cW =  watch->tft->textWidth("Connection", 2);
    int16_t dW =  watch->tft->textWidth("Disconnect", 2);
    int16_t w = cW > dW ? cW : dW;
    w += 6;
    int16_t x = 160;
    int16_t y = 20;
    int16_t h =  watch->tft->fontHeight(2) + 4;
    uint16_t col = status ? TFT_GREEN : TFT_GREY;
    watch->tft->fillRoundRect(x, y, w, h, 3, col);
    watch->tft->setTextColor(TFT_BLACK, col);

    watch->tft->setTextFont(2);
    watch->tft->drawString(str, x + 2, y);
    watch->tft->setTextFont(2);
    str = String(setupcfg.powerdc.cur);
    watch->tft->drawString(str, 0, y);
}



void Le_Touch()
{
  int16_t x,y;

  // Print touch coordinates to the screen
  if (watch->getTouch(x, y)) 
  {
      touch.press = TRUE;
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
        MarcaTempoInicio();
      }
   } else 
   {
      touch.press = FALSE;
      if (touch.flgtouch)
      {
        //Chama evento de retirar Analisar touch
        touch.flgtouch = FALSE;
        
        AnalisaTouch();
        
      }
   }

}

int EnviaParaGoogle(const char *jsonRequest, char *outputText) {
  // Conectar ao Wi-Fi se ainda não estiver conectado
  if (WiFi.status() != WL_CONNECTED) 
  {
            return 0; // Falha ao conectar ao Wi-Fi
  
  }

  // Conectar ao servidor do Google Speech-to-Text API
  WiFiClientSecure client;
  if (!client.connect(googleapi.googleHost, googleapi.googlePort)) {
    return 0; // Falha ao conectar ao servidor
  }

  // Preparar cabeçalhos da requisição HTTP POST
  String requestHeaders = "POST /v1/speech:recognize?key=" + String( googleapi.googleApiKey) + " HTTP/1.1\r\n";
  requestHeaders += "Host: " + String(googleapi.googleHost) + "\r\n";
  requestHeaders += "Content-Type: application/json\r\n";
  requestHeaders += "Content-Length: " + String(strlen(jsonRequest)) + "\r\n";
  requestHeaders += "Connection: close\r\n\r\n";

  // Enviar requisição para o Google Speech-to-Text API
  client.print(requestHeaders);
  client.print(jsonRequest);

  // Aguardar a resposta e ler o cabeçalho da resposta HTTP
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      break;
    }
  }

  // Ler o corpo da resposta (JSON)
  String jsonResponse = client.readString();

  // Analisar a resposta JSON
  DynamicJsonDocument jsonBuffer(2048);
  deserializeJson(jsonBuffer, jsonResponse);
  JsonObject json = jsonBuffer.as<JsonObject>();

  // Verificar se a resposta contém o resultado da transcrição
  if (json.containsKey("results") && json["results"].as<JsonArray>().size() > 0) {
    const char *transcript = json["results"][0]["alternatives"][0]["transcript"];
    strcpy(outputText, transcript);
    return 1; // Sucesso
  } else {
    return 0; // Falha
  }
}
/*
void EmpacoteVOZ(const uint8_t *audioData, size_t audioDataSize, char *jsonOutput) 
{
  
  // Configurações do Google Speech-to-Text API
  const char *languageCode = "pt-BR";
  const char *encoding = "LINEAR16";
  int32_t sampleRateHertz = 16000;

  // Calcular o tamanho necessário para o buffer JSON
  const size_t base64AudioSize = (audioDataSize * 4 + 2) / 3 + 1; // Tamanho do áudio em Base64
  const size_t jsonBufferSize = 300 + base64AudioSize; // Tamanho necessário para o buffer JSON

  // Criar buffer JSON e objeto JSON
  DynamicJsonDocument jsonBuffer(jsonBufferSize);
  JsonObject json = jsonBuffer.to<JsonObject>();

  // Adicionar informações do áudio ao objeto JSON
  JsonObject config = json.createNestedObject("config");
  config["encoding"] = encoding;
  config["sampleRateHertz"] = sampleRateHertz;
  config["languageCode"] = languageCode;

  // Converter áudio para Base64
  String base64Audio;
  base64Audio.reserve(base64AudioSize);
  base64::encode(audioData, audioDataSize, base64Audio);

  // Adicionar áudio codificado em Base64 ao objeto JSON
  JsonObject audio = json.createNestedObject("audio");
  audio["content"] = base64Audio;

  // Serializar o objeto JSON no buffer de saída fornecido
  serializeJson(json, jsonOutput);
  
}
*/

void EmpacoteVOZ(const uint8_t *audioData, size_t audioDataSize, char *jsonOutput) 
{
  /*
  // Configurações do Google Speech-to-Text API
  const char *languageCode = "pt-BR";
  const char *encoding = "LINEAR16";
  int32_t sampleRateHertz = 16000;

  // Calcular o tamanho necessário para o buffer JSON
  const size_t base64AudioSize = calcBase64BufferSize(audioDataSize); // Tamanho do áudio em Base64
  const size_t jsonBufferSize = 300 + base64AudioSize; // Tamanho necessário para o buffer JSON

  // Criar buffer JSON e objeto JSON
  DynamicJsonDocument jsonBuffer(jsonBufferSize);
  JsonObject json = jsonBuffer.to<JsonObject>();

  // Adicionar informações do áudio ao objeto JSON
  JsonObject config = json.createNestedObject("config");
  config["encoding"] = encoding;
  config["sampleRateHertz"] = sampleRateHertz;
  config["languageCode"] = languageCode;

  // Converter áudio para Base64
  char base64Audio[base64AudioSize];
  base64_encode(base64Audio, (char*)audioData, audioDataSize);

  // Adicionar áudio codificado em Base64 ao objeto JSON
  JsonObject audio = json.createNestedObject("audio");
  audio["content"] = base64Audio;

  // Serializar o objeto JSON no buffer de saída fornecido
  serializeJson(json, jsonOutput, jsonBufferSize);
  */
}

void EnviaVOZ()
{
  char json[5000];
  memset(buffer,'\0',sizeof(buffer));
  buffer_len = 0;
  memset(json,'\0',sizeof(json));
  EmpacoteVOZ(buffer, buffer_len, json);
  char texto[1000];
  memset(texto,'\0',sizeof(texto));
  if (EnviaParaGoogle(json,texto) !=0)
  {
    printxy(10,100,texto);
  }



}

void Le_MIC()
{
    Serial.println("Start Le_MIC");
    size_t read_len = 0;
    micval.j = micval.j + 1;
    i2s_read(I2S_NUM_0, (char *) buffer, BUFFER_SIZE, &read_len, portMAX_DELAY);
    buffer_len = buffer_len+BUFFER_SIZE;
    for (int i = 0; i < BUFFER_SIZE / 2 ; i++) {
        micval.val1 = buffer[i * 2];
        micval.val2 = buffer[i * 2 + 1] ;
        micval.val16 = micval.val1 + micval.val2 *  256;
        if (micval.val16 > 0) {
            micval.val_avg = micval.val_avg + micval.val16;
            micval.val_max = max( micval.val_max, micval.val16);
        }
        if (micval.val16 < 0) {
            micval.val_avg_1 = micval.val_avg_1 + micval.val16;
            micval.val_max_1 = min( micval.val_max_1, micval.val16);
        }
        micval.all_val_avg = micval.all_val_avg + micval.val16;
        if (abs(micval.val16) >= 20)
            micval.all_val_zero1 = micval.all_val_zero1 + 1;
        if (abs(micval.val16) >= 15)
            micval.all_val_zero2 = micval.all_val_zero2 + 1;
        if (abs(micval.val16) > 5)
            micval.all_val_zero3 = micval.all_val_zero3 + 1;
    }

    if (micval.j % 2 == 0 && micval.j > 0) {
        micval.val_avg = micval.val_avg / BUFFER_SIZE ;
        micval.val_avg_1 = micval.val_avg_1 / BUFFER_SIZE;
        micval.all_val_avg = micval.all_val_avg / BUFFER_SIZE ;

        lv_chart_set_next(chart, ser1, micval.val_avg);

        micval.val_avg = 0;
        micval.val_max = 0;

        micval.val_avg_1 = 0;
        micval.val_max_1 = 0;

        micval.all_val_avg = 0;
        micval.all_val_zero1 = 0;
        micval.all_val_zero2 = 0;
        micval.all_val_zero3 = 0;
    }
    lv_task_handler();
    delay(5);
    Serial.println("Finalizou Le_MIC");
}

void Le_Energia()
{
  if(setupcfg.powerdc.flgpower==false)
  {
    //setupcfg.powerdc.cur =   (uint16_t)((watch->power->getChargeControlCur() / MAXIMALCUR)*100);
    setupcfg.powerdc.cur = watch->power->getBattPercentage() ;
  } else
  {
    setupcfg.powerdc.cur = 100;
  }
  Serial.print("Corrente:");
  Serial.println(watch->power->getChargeControlCur());
  Serial.print("Percentual:");
  Serial.println( watch->power->getBattPercentage() );
}

void Leituras()
{
   LePower(); /*Le funcionalidades de interrupcao AXP202 */
   Le_Touch();
   Le_Energia(); /*Coleta informação da energia*/
   if (maquina.estado_atual == EN_MIC)
   {
      if(touch.press)
      {
        Le_MIC();
      } else
      {
        printxy(10, 10, "Pressione a tela");
        if (buffer)
        EnviaVOZ();
      }
   }
   WifiConnected();
       
 
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
            if(maquina.estado_atual!= EN_MIC)
            {
              // 10 segundos se passaram
              Serial.println("MedeTempo em Repouso");           
            
              MudaEstado(&maquina, EN_REPOUSO); //Muda o estado da maquina de estado
            }
        }

  
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
  
      } else
      {
        if(descy<LIMITCLICK&&descy>-LIMITCLICK&&descx<-LIMITCLICK)
        {
          touch.move = TC_MOVERIGHT;
  
        } else
        {
          if(descy<LIMITCLICK&&descy>-LIMITCLICK&&descx>LIMITCLICK)
          {
            touch.move = TC_MOVELEFT;
  
          }
        }
      }
    }  
  }
  
}

//Analisa sinais obtidos de leituras
void Analisa()
{
  
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
        if(maquina.estado_atual == EN_WIFI)
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
       } else
       if(maquina.estado_atual == EN_SETCLOCK)
       {
         MudaEstado(&maquina,EN_WIFI);
       }

     } else 
     if(touch.move == TC_MOVEDOWN)
     {
       Serial.println("MOVE DOWN");
       if(maquina.estado_atual == EN_SETCLOCK)
       {
         MudaEstado(&maquina,EN_WATCH01);
       } else
       if(maquina.estado_atual == EN_WIFI)
       {
         MudaEstado(&maquina,EN_SETCLOCK);
       } 
       
     } else 
     if(touch.move == TC_MOVELEFT)
     {
      
       if(maquina.estado_atual==EN_WATCH01) 
       {
         MudaEstado(&maquina,EN_MIC);
       }
     } else 
     if(touch.move == TC_MOVERIGHT)
     {
      
       if (maquina.estado_atual==EN_MIC)
       {
         MudaEstado(&maquina,EN_WATCH01);
       }
     }  

     touch.move = TC_NONE;
   }
}

void MostraHora()
{
  if (millis() - interval > 1000) 
  {
    interval = millis();

    watch->tft->setTextColor(TFT_YELLOW, TFT_BLACK);

    watch->tft->drawString(rtc->formatDateTime(PCF_TIMEFORMAT_DD_MM_YYYY), 50, 200, 4);

    watch->tft->drawString(rtc->formatDateTime(PCF_TIMEFORMAT_HMS), 5, 118, 7);
  }
}

//Maquina de estado
void EstadoAtual()
{
  if(maquina.estado_atual != EN_REPOUSO)
  {
      if(maquina.estado_atual != EN_MIC)
      {
        MostraHora();
      }
      
      if(maquina.estado_atual == EN_WATCH01)
      {      
          //Serial.print('.');
          Display_Relogio();
          MedeTempo(); //Avalia se entra em modo de espera
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
      } else
      if(maquina.estado_atual == EN_WIFI)
      {
        MedeTempo(); //Avalia se entra em modo de espera

      } else
      if(maquina.estado_atual ==EN_MIC)
      {
        MedeTempo(); //Avalia se entra em modo de espera
      }

  } else
  {
    //Em repouso
    //sleep(100);
    //esp_sleep_enable_timer_wakeup(tempoDeSuspensao);
  }
  

}

void loop()
{  
  Leituras();
  Analisa();
  //Maquinas de Estado
  EstadoAtual();
}
