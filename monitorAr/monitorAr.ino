
/***************************************************************************
  Projeto MonitorAr
  Teste dos sensores BME280 e MQ-135 com Esp8266 NodeMcu Lolin V3

  BME280
  SDA > D2
  SCL > D1
  VCC > 3V
  GND > GND

  MQ-135
  AO > A0
  VCC > 3V
  GND > GND

  *** *** *** *** *** *** *** ***

  Example for BME280 Weather Station using two Sensors with I2C Communication
  written by Thiago Barros for BlueDot UG (haftungsbeschränkt)
  BSD License

  This sketch was written for the Bosch Sensor BME280.
  The BME280 is a MEMS device for measuring temperature, humidity and atmospheric pressure.
  For more technical information on the BME280, please go to ------> http://www.bluedot.space

 ***************************************************************************/
#include <Wire.h>
#include "BlueDot_BME280.h"
BlueDot_BME280 bme;                                     //Object for Sensor 1
int bme1Detected = 0;                                    //Checks if Sensor 1 is available

//-------------------------
// Este código usa a biblioteca MQTT da Adafruit.
/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>          //  Inclui a biblioteca ESP8266WiFi.
#include "Adafruit_MQTT.h"        //  Inclui a biblioteca Adafruit_MQTT.
#include "Adafruit_MQTT_Client.h" //  Inclui a biblioteca Adafruit_MQTT_Client.


//-------------------------
//------------------------- INFORMAÇÕES CONFIDENCIAIS ------------------------
//  Antes de compartilhar o código, apagar as informações de login e senha.

/************************* Ponto de acesso de WiFi *********************************/
#define WLAN_SSID       "Nome do WiFi"    //  Nome da rede de WiFi.
#define WLAN_PASS       "senha"           //  Senha da rede de WiFi.

/************************* Setup de acesso do Adafruit.io *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL


#define AIO_USERNAME  "nomeDeUsuario"
#define AIO_KEY       "chaveDaAdafruitIO"

//  Observação: AIO se refere a Adafruit.io.

//-------------------------
//  Configurações globais de acesso, não precisa alterar nada aqui.

// Cria uma classe do ESP8266 WiFiClient para conectar com o servidor MQTT
WiFiClient client;
//  Ou então usar WiFiFlientSecure para conexão SSL.
//WiFiClientSecure client;

//  Configura a classe de cliente MQTT passando os nomes e chaves de acesso, definidos anteriormente.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//-------------------------
//  Configura os feeds para publicação (publishing).
// O caminho (path) do MQTT para o Adafruit.io segue o padrão: <nomeDoUsuário>/feeds/<nomeDoFeed>

Adafruit_MQTT_Publish pubTemperatura = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatura");
Adafruit_MQTT_Publish pubUmidade = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidade");
Adafruit_MQTT_Publish pubPressao = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressao");
Adafruit_MQTT_Publish pubAltitude = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/altitude");
Adafruit_MQTT_Publish pubGases = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gases");

//-------------------------


int led = D4;
int gases = A0;
int mqGases;
float bmeTemperatura;
float bmeUmidade;
float bmePressao;
float bmeAltitude;

int mqGasesAnt;
float bmeTemperaturaAnt;
float bmeUmidadeAnt;
float bmePressaoAnt;
float bmeAltitudeAnt;


float mediaTemperatura;

unsigned long tempo;
int intervalo = 30000;


void setup() {
  Serial.begin(115200);
  Serial.println(F("Iniciando o MonitorAr..."));

  pinMode(led, OUTPUT);
  pinMode(gases, INPUT);


  //-------------------------
  Serial.println(F("Sistema usa o Adafruit MQTT")); // Imprime no Monitor Serial.
  Serial.println();
  Serial.println();

  //-------------------------
  // Conectar na rede de WiFi usando o nome "WLAN_SSID" e senha "WLAN_PASS" definidos anteriormente.
  Serial.print("Conectando na rede ");
  Serial.println(WLAN_SSID);          // Imprime no Monitor Serial o nome da rede de WiFi.
  WiFi.begin(WLAN_SSID, WLAN_PASS);   // Conecta na rede de WiFi.

  // Aguarda a conexão WiFi ser realizada.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Imprime no Monitor Serial a confirmação da conexão e o número IP do Esp8266.
  Serial.println();
  Serial.println("WiFi conectado!");
  Serial.println("endereco IP: ");
  Serial.println(WiFi.localIP());
  piscarLed(5);



  //*********************************************************************
  //*************BASIC SETUP - SAFE TO IGNORE****************************
  //This program is set for the I2C mode
  bme.parameter.communication = 0;                    //I2C communication for Sensor 1 (bme1)
  //*********************************************************************
  //*************BASIC SETUP - SAFE TO IGNORE****************************
  //Set the I2C address of your breakout board
  bme.parameter.I2CAddress = 0x76;                    //I2C Address for Sensor 1 (bme1)
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  //Now choose on which mode your device will run
  //On doubt, just leave on normal mode, that's the default value
  //0b00:     In sleep mode no measurements are performed, but power consumption is at a minimum
  //0b01:     In forced mode a single measured is performed and the device returns automatically to sleep mode
  //0b11:     In normal mode the sensor measures continually (default value)
  bme.parameter.sensorMode = 0b11;                    //Setup Sensor mode for Sensor 1
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  //Great! Now set up the internal IIR Filter
  //The IIR (Infinite Impulse Response) filter suppresses high frequency fluctuations
  //In short, a high factor value means less noise, but measurements are also less responsive
  //You can play with these values and check the results!
  //In doubt just leave on default
  //0b000:      factor 0 (filter off)
  //0b001:      factor 2
  //0b010:      factor 4
  //0b011:      factor 8
  //0b100:      factor 16 (default value)
  bme.parameter.IIRfilter = 0b100;                   //IIR Filter for Sensor 1
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  //Next you'll define the oversampling factor for the humidity measurements
  //Again, higher values mean less noise, but slower responses
  //If you don't want to measure humidity, set the oversampling to zero
  //0b000:      factor 0 (Disable humidity measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  bme.parameter.humidOversampling = 0b101;            //Humidity Oversampling for Sensor 1
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  //Now define the oversampling factor for the temperature measurements
  //You know now, higher values lead to less noise but slower measurements
  //0b000:      factor 0 (Disable temperature measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  bme.parameter.tempOversampling = 0b101;              //Temperature Oversampling for Sensor 1
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************
  //Finally, define the oversampling factor for the pressure measurements
  //For altitude measurements a higher factor provides more stable values
  //On doubt, just leave it on default
  //0b000:      factor 0 (Disable pressure measurement)
  //0b001:      factor 1
  //0b010:      factor 2
  //0b011:      factor 4
  //0b100:      factor 8
  //0b101:      factor 16 (default value)
  bme.parameter.pressOversampling = 0b101;             //Pressure Oversampling for Sensor 1
  //*********************************************************************
  //*************ADVANCED SETUP - SAFE TO IGNORE*************************

  //For precise altitude measurements please put in the current pressure corrected for the sea level
  //On doubt, just leave the standard pressure as default (1013.25 hPa);
  bme.parameter.pressureSeaLevel = 1015;

  //default value of 1013.25 hPa (Sensor 1)

  // Pressão Reduzida ao Nível Médio do Mar: 1015hPa - 20:05 - 05/07/2020
  // http://tempo1.cptec.inpe.br/cidades/tempoCidade/2826

  //Also put in the current average temperature outside (yes, really outside!)
  //For slightly less precise altitude measurements, just leave the standard temperature as default (15°C and 59°F);
  bme.parameter.tempOutsideCelsius = 23.7 ;               //default value of 15°C
  bme.parameter.tempOutsideFahrenheit = 74.66;            //default value of 59°F

  // Temperatura média (°C) de julho em Juazeiro do Norte: 23.7 - 20:05 - 05/07/2020
  // Temperatura média (°C) anual em Juazeiro do Norte: 25.2 - 20:05 - 05/07/2020
  // https://pt.wikipedia.org/wiki/Predefini%C3%A7%C3%A3o:Tabela_clim%C3%A1tica_de_Juazeiro_do_Norte

  //*********************************************************************
  //*************ADVANCED SETUP IS OVER - LET'S CHECK THE CHIP ID!*******
  if (bme.init() != 0x60)
  {
    Serial.println(F("Ops! First BME280 Sensor not found!"));
    bme1Detected = 0;
  }

  else
  {
    Serial.println(F("First BME280 Sensor detected!"));
    bme1Detected = 1;
    piscarLed(5);
  }


  if (bme1Detected == 0)
  {
    Serial.println();
    Serial.println();
    Serial.println(F("Troubleshooting Guide"));
    Serial.println(F("*************************************************************"));
    Serial.println(F("1. Let's check the basics: Are the VCC and GND pins connected correctly? If the BME280 is getting really hot, then the wires are crossed."));
    Serial.println();
    Serial.println(F("2. Did you connect the SDI pin from your BME280 to the SDA line from the Arduino?"));
    Serial.println();
    Serial.println(F("3. And did you connect the SCK pin from the BME280 to the SCL line from your Arduino?"));
    Serial.println();
    Serial.println(F("4. One of your sensors should be using the alternative I2C Address(0x76). Did you remember to connect the SDO pin to GND?"));
    Serial.println();
    Serial.println(F("5. The other sensor should be using the default I2C Address (0x77. Did you remember to leave the SDO pin unconnected?"));
    Serial.println();
    while (1);
  }
  Serial.println();
  Serial.println();


  if (bme1Detected)
  {
    bmeTemperaturaAnt = bme.readTempC();
    bmeUmidadeAnt = bme.readHumidity();
    bmePressaoAnt = bme.readPressure();
    bmeAltitudeAnt = bme.readAltitudeMeter();
    mqGasesAnt = analogRead(gases);
  }
  tempo = millis();
}

//*********************************************************************
//*************NOW LET'S START MEASURING*******************************
void loop() {
  digitalWrite(led, LOW);
  //  Estabelece e garante que a conexão com o servidor MQTT esteja "viva",
  //  fazendo a primeira conexão e reconectando automaticamente se for desconectado.
  //  Veja a definição da função MQTT_connect() abaixo do void loop()
  MQTT_connect();

  if (bme1Detected)
  {
    bmeTemperatura = (bme.readTempC() + bmeTemperaturaAnt) / 2;
    bmeUmidade = (bme.readHumidity() + bmeUmidadeAnt) / 2;
    bmePressao = (bme.readPressure() + bmePressaoAnt) / 2;
    bmeAltitude = (bme.readAltitudeMeter() + bmeAltitudeAnt) / 2;
    mqGases = (analogRead(gases) + mqGasesAnt) / 2;
  }
  else
  {
    Serial.print(F("Temperature Sensor 1 [°C]:\t\t"));
    Serial.println(F("Null"));
    Serial.print(F("Humidity Sensor 1 [%]:\t\t\t"));
    Serial.println(F("Null"));
    Serial.print(F("Pressure Sensor 1 [hPa]:\t\t"));
    Serial.println(F("Null"));
    Serial.print(F("Altitude Sensor 1 [m]:\t\t\t"));
    Serial.println(F("Null"));
    Serial.println(F("****************************************"));
  }


  if (millis() - tempo > intervalo) {
    tempo = millis();
    piscarLed(5);
    //*********************************************************************
    //////////////////// Envio dos dados

    // *************************************** bmeTemperatura
    //  Imprime no Monitor Serial o nome e valor de bmeTemperatura.
    Serial.print(F("\nSending bmeTemperatura "));
    Serial.print(bmeTemperatura);
    Serial.print("... ");

    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é impressa no Monitor Serial avisando.
    if (! pubTemperatura.publish(bmeTemperatura)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3); //  Executa a função piscarLed() piscando o led 3 vezes.
    }

    // Repete o mesmo procedimento de publicação dos feeds com os outros valores.

    // *************************************** bmeUmidade
    //  Imprime no Monitor Serial o nome e valor de bmeUmidade.
    Serial.print(F("\nSending bmeUmidade "));
    Serial.print(bmeUmidade);
    Serial.print("... ");

    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é impressa no Monitor Serial avisando.
    if (! pubUmidade.publish(bmeUmidade)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3); //  Executa a função piscarLed() piscando o led 3 vezes.
    }
    // *************************************** bmePressao
    //  Imprime no Monitor Serial o nome e valor de bmePressao.
    Serial.print(F("\nSending bmePressao "));
    Serial.print(bmePressao);
    Serial.print("... ");

    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é impressa no Monitor Serial avisando.
    if (! pubPressao.publish(bmePressao)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3); //  Executa a função piscarLed() piscando o led 3 vezes.
    }
    // *************************************** bmeAltitude
    //  Imprime no Monitor Serial o nome e valor de bmeAltitude.
    Serial.print(F("\nSending bmeAltitude "));
    Serial.print(bmeAltitude);
    Serial.print("... ");

    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é impressa no Monitor Serial avisando.
    if (! pubAltitude.publish(bmeAltitude)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3); //  Executa a função piscarLed() piscando o led 3 vezes.
    }

    // *************************************** mqGases
    //  Imprime no Monitor Serial o nome e valor de mqGases.
    Serial.print(F("\nSending mqGases "));
    Serial.print(mqGases);
    Serial.print("... ");
    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é impressa no Monitor Serial avisando.
    if (! pubGases.publish(mqGases)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3); //  Executa a função piscarLed() piscando o led 3 vezes.
    }

    // *************************************** tensaoV
    //  Imprime no Monitor Serial o nome e valor de tensaoV.
    Serial.print(F("\nSending tensaoV "));
    Serial.print(tensaoV);
    Serial.print("... ");
    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é impressa no Monitor Serial avisando.
    if (! pubTensao.publish(tensaoV)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3); //  Executa a função piscarLed() piscando o led 3 vezes.
    }
  }

  ////////////////////
  Serial.println(F("****************************************"));
  Serial.print("Presença de gases:\t\t\t\t");
  Serial.println(mqGases);
  Serial.print(F("Temperatura [°C]:\t\t\t\t\t"));
  Serial.println(bmeTemperatura);
  Serial.print(F("Umidade [%]:\t\t\t\t\t"));
  Serial.println(bmeUmidade);
  Serial.print(F("Pressão [hPa]:\t\t\t\t\t"));
  Serial.println(bmePressao);
  Serial.print(F("Altitude [m]:\t\t\t\t\t"));
  Serial.println(bmeAltitude);
  Serial.println(F("****************************************"));
  Serial.print("[Anterior] Presença de gases:\t\t");
  Serial.println(mqGasesAnt);
  Serial.print(F("[Anterior] Temperatura [°C]:\t\t\t"));
  Serial.println(bmeTemperaturaAnt);
  Serial.print(F("[Anterior] Umidade [%]:\t\t\t"));
  Serial.println(bmeUmidadeAnt);
  Serial.print(F("[Anterior] Pressão [hPa]:\t\t\t"));
  Serial.println(bmePressaoAnt);
  Serial.print(F("[Anterior] Altitude [m]:\t\t\t"));
  Serial.println(bmeAltitudeAnt);

  bmeTemperaturaAnt = bmeTemperaturaAnt;
  bmeUmidadeAnt = bmeUmidade;
  bmePressaoAnt = bmePressao ;
  bmeAltitudeAnt = bmeAltitude;
  mqGasesAnt = mqGases;

  ////////////////////
  delay(1000);
}


// -------------------------
//  Função para piscar led.

/*    É usado o laço for() para definir a quantidade de piscadas.
  A quantidade de piscada é passada entre parênteses,
  por exemplo, para piscar 3 vezes usar piscarLed(3).
  Em cada ciclo é executado um digitalWrite LOW e HIGH
  com intervalo de 100 milisegundos entre ligado e desligado.
*/

void piscarLed(int piscadas) {
  for (int i = 0; i < piscadas; i++) {
    digitalWrite(led, LOW);
    delay(100);
    digitalWrite(led, HIGH);
    delay(100);
  }
  delay(50);
}


//-------------------------
// Função da biblioteca MQTT para conectar e reconectar quando necessário no servidor da Adafruit.io.

void MQTT_connect() {
  int8_t ret;

  // Para se já estiver conectado.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Conectando ao MQTT... ");

  uint8_t retries = 3;

  // A conexão retornará 0 para conectado.
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Tentando a conexão MQTT em 5 segundos...");
    mqtt.disconnect();
    delay(5000);  // Espera 5 segundos.
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Conectado!");
  piscarLed(10);  //  Quando MQTT conectar, a função piscarLed() é executada 10 vezes.
}
