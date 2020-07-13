
/***************************************************************************

  Projeto MonitorAr

  O projeto MonitorAr é uma iniciativa de ciência cidadã de sensoriamento participativo sobre dados atmosféricos,
  desenvolvido por Jéssyca Rios e Artur V. Cordeiro em parceria com a ONG Abecê da Educação Ambiental
  https://ongabcambiental.com

  ***************************************************************************


  O kit MonitorAr é composto por um microcontrolador ESP8266 NodeMCU Lolin V3, e dois sensores, o BME280 e o MQ-135.

  Conexões do BME280 no NodeMCU Lolin V3:
  SDA > D2
  SCL > D1
  VCC > 3V
  GND > GND

  Conexões do MQ-135 no NodeMCU Lolin V3:
  AO > A0
  VCC > 3V
  GND > GND
*/

// Este código usa a biblioteca BME280 da BlueDot.
/*
  *** *** *** *** *** *** *** ***
  Example for BME280 Weather Station using two Sensors with I2C Communication
  written by Thiago Barros for BlueDot UG (haftungsbeschränkt)
  BSD License

  This sketch was written for the Bosch Sensor BME280.
  The BME280 is a MEMS device for measuring temperature, humidity and atmospheric pressure.
  For more technical information on the BME280, please go to ------> http://www.bluedot.space

 **************************************************************************
*/
#include <Wire.h> // Biblioteca para estabelecer a comunicação I2C usada no BME280.
#include "BlueDot_BME280.h"   // Biblioteca para usar o sensor BME280.
BlueDot_BME280 bme;           // Instância da biblioteca, chamada de bme.
int bme1Detected = 0;         // Variável para checar se o sensor está disponível.

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
#define WLAN_SSID       "Nome WiFi"       //  Nome da rede de WiFi.
#define WLAN_PASS       "senha"           //  Senha da rede de WiFi.

/************************* Setup de acesso do Adafruit.io *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL

#define AIO_USERNAME  "nomeDeUsuario"       //  Nome de usuário no servidor da AIO.
#define AIO_KEY       "chaveDaAdafruitIO"   //  Chave de acesso ao servidor da AIO.

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

// ATENÇÃO: substituir a letra N no final da linha pelo respectivo número da estação.
Adafruit_MQTT_Publish pubTemperatura = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperaturaN");
Adafruit_MQTT_Publish pubUmidade = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidadeN");
Adafruit_MQTT_Publish pubPressao = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressaoN");
Adafruit_MQTT_Publish pubAltitude = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/altitudeN");
Adafruit_MQTT_Publish pubGases = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gasesN");

//-------------------------


int led = D4;             // Variável especificando o pino do led azul do ESP8266.
int gases = A0;           // Variável especificando o pino analógico do sensor MQ-135 .
int mqGases;              // Variável para armazenar o valor de leitura do sensor MQ-135.
float bmeTemperatura;     // Variável para armazenar o valor de temperatura.
float bmeUmidade;         // Variável para armazenar o valor de umidade.
float bmePressao;         // Variável para armazenar o valor de pressão atmosférica.
float bmeAltitude;        // Variável para armazenar o valor de altitude.

int mqGasesAnt;           // Variável para armazenar o valor de leitura anterior do sensor MQ-135.
float bmeTemperaturaAnt;  // Variável para armazenar o valor anterior de temperatura.
float bmeUmidadeAnt;      // Variável para armazenar o valor anterior de umidade.
float bmePressaoAnt;      // Variável para armazenar o valor anterior de pressão atmosférica.
float bmeAltitudeAnt;     // Variável para armazenar o valor anterior de altitude.

// As variáveis de valores anteriores são usadas para calcular a média das leituras no intervalo do ciclo de envio de dados.

unsigned long tempo;      // Variável para armazenar a contagem de tempo.
int intervalo = 30000;    // Definição da duração do intervalo de tempo, para o ciclo de envio de dados.


void setup() {
  Serial.begin(115200);   // Inicia a comunicação serial.
  Serial.println(F("Iniciando o MonitorAr..."));  // Exibe no Monitor Serial.

  pinMode(led, OUTPUT);   // Define o pino do led como saída.
  pinMode(gases, INPUT);  // Define o pino analógico usado pelo sensor do gás como entrada.

  //-------------------------
  Serial.println(F("Sistema usa o Adafruit MQTT"));
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
  piscarLed(5); // Função para piscar 5 vezes o led azul do ESP8266.
  // A função 'piscarLed()' está definida abaixo, após o void loop().

  //-------------------------
  // Configurações da biblioteca BlueDot para usar o sensor BME280
  bme.parameter.communication = 0;          // Estabelece o padrão I2C de comunicação.
  bme.parameter.I2CAddress = 0x76;          // Define o endereço 0x76 para a comunicação I2C.
  bme.parameter.sensorMode = 0b11;          // Define modo normal de uso do sensor BME280.
  bme.parameter.IIRfilter = 0b100;          // Define fator padrão para o filtro Infinite Impulse Response do BME280.
  bme.parameter.humidOversampling = 0b101;  // Define o fator padrão para Oversampling de umidade do BME280.
  bme.parameter.tempOversampling = 0b101;   // Define o fator padrão para Oversampling de temperatura do BME280.
  bme.parameter.pressOversampling = 0b101;  // Define o fator padrão para Oversampling de pressão do BME280.

  // Para referência do cálculo de altitude,
  // está sendo usado o valor padrão de pressão atmosférica a nível do mar de 1013.25 hPa.
  bme.parameter.pressureSeaLevel = 1013.25;

  // Para referência do cálculo de altitude,
  // está sendo usado o valor da temperatura média anual em Juazeiro do Norte de 25.2°C.
  // Observação: o valor padrão usado na biblioteca é 15°C / 59°F.
  bme.parameter.tempOutsideCelsius = 25.2;
  bme.parameter.tempOutsideFahrenheit = 77.36;

  // Verificação da comunicação com o sensor BME280.
  if (bme.init() != 0x60)
  {
    Serial.println(F("Ops! Sensor BME280 não foi detectado"));
    bme1Detected = 0;
  }
  else
  {
    Serial.println(F("Sensor BME280 detectado!"));
    bme1Detected = 1;
    piscarLed(5);     // Executa a função piscarLed() piscando o led 5 vezes.
  }
  Serial.println();
  Serial.println();

  // Se o sensor BME280 for detectado, armazena a leitura de dados nas variáveis de valores anteriors.
  if (bme1Detected)
  {
    bmeTemperaturaAnt = bme.readTempC();
    bmeUmidadeAnt = bme.readHumidity();
    bmePressaoAnt = bme.readPressure();
    bmeAltitudeAnt = bme.readAltitudeMeter();
    mqGasesAnt = analogRead(gases);
  }
  tempo = millis(); // Define a variável tempo com o tempo corrente (millis()).
}

void loop() {
  digitalWrite(led, LOW); // Liga o led azul do ESP8266.

  //  Estabelece e garante que a conexão com o servidor MQTT esteja "viva",
  //  fazendo a primeira conexão e reconectando automaticamente se for desconectado.
  //  Veja a definição da função MQTT_connect() abaixo do void loop()
  MQTT_connect();

  // Calcula a média e armazena nas variáveis de valores.
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
    Serial.print(F("Sensor de temperatura [°C]:\t\t"));
    Serial.println(F("ERRO"));
    Serial.print(F("Sensor de umidade [%]:\t\t\t"));
    Serial.println(F("ERRO"));
    Serial.print(F("Sensor de pressão [hPa]:\t\t"));
    Serial.println(F("ERRO"));
    Serial.print(F("Sensor de altitude [m]:\t\t\t"));
    Serial.println(F("ERRO"));
    Serial.println(F("****************************************"));
  }

  // Verifica se o tempo decorrente após o último envio dos dados a plataforma da Adafruit
  // é superior ao intervalo de 30 segundos. Caso positivo, será feito o envio dos dados.
  if (millis() - tempo > intervalo) {
    tempo = millis();   // Reseta a variável tempo para a contagem de tempo presente.
    piscarLed(5);       // Executa a função piscarLed() piscando o led 5 vezes.

    //-------------------------
    //////////////////// Envio dos dados para a plataforma Adafruit IO

    // *************************************** bmeTemperatura
    //  Imprime no Monitor Serial o nome e valor de bmeTemperatura.
    Serial.print(F("\nSending bmeTemperatura "));
    Serial.print(bmeTemperatura);
    Serial.print("... ");
    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é exibida no Monitor Serial avisando.
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
  }

  //-------------------------
  // Exibe no monitor serial os valores dos dados
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


  // Atualiza os valores anteriores com os últimos valores calculados.
  bmeTemperaturaAnt = bmeTemperaturaAnt;
  bmeUmidadeAnt = bmeUmidade;
  bmePressaoAnt = bmePressao ;
  bmeAltitudeAnt = bmeAltitude;
  mqGasesAnt = mqGases;

  // Pausa de 1 segundo (1000 milisegundos).
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
