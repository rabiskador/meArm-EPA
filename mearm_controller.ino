
/*
  MeArm + ESP32 + Controle PS5 (DualSense) via Bluepad32
  + OLED SSD1306

  BIBLIOTECAS:
    - Bluepad32 (incluida na placa esp32_bluepad32)
    - ESP32Servo
    - Adafruit SSD1306
    - Adafruit GFX Library

  PINOS DOS SERVOS:
    BASE     -> GPIO 4
    OMBRO    -> GPIO 5
    COTOVELO -> GPIO 18
    GARRA    -> GPIO 19

  OLED:
    SDA -> GPIO 21
    SCL -> GPIO 22
    VCC -> 3V3
    GND -> GND

  ALIMENTACAO:
    AMARELO - GPIO DO ESP
    VERMELHO - 5v Fonte Externa
    MARROM - GND da fonte ao GND do ESP32.

  CONTROLES:
    Analógico esquerdo X -> Base
    Analógico esquerdo Y -> Ombro
    Analógico direito Y  -> Cotovelo
    R1 -> Abre garra
    L1 -> Fecha garra
    Quadrado -> Posição inicial
    Options -> Alterna velocidade

  IMPORTANTE:
    Base e cotovelo possuem limites provisórios.
    Calibre os limites antes de liberar movimentos maiores.
*/

#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =============== PINOS E NOMES ===============

const uint8_t PINOS[4] = {4, 5, 18, 19};

const char* NOMES[4] = {
  "BASE",
  "OMBRO",
  "COTOV",
  "GARRA"
};

// =============== LIMITES MECANICOS ===============

// Ordem: BASE, OMBRO, COTOVELO, GARRA

// BASE: faixa provisoria reduzida
float LIM_MIN[4] = {60, 10, 70, 10};
float LIM_MAX[4] = {120, 90, 110, 70};

// Posicoes iniciais
float HOME_POS[4] = {90, 50, 90, 10};

// Sentido de movimento de cada eixo
// Use -1 se algum eixo estiver invertido.
int8_t SENTIDO[4] = {1, 1, 1, 1};

// =============== VELOCIDADE ===============

const int ZONA_MORTA = 60;

const float PASSO_NORMAL = 2.0;
const float PASSO_LENTO  = 0.7;
const float PASSO_GARRA  = 1.0;

// =============== OLED ===============

#define LARGURA 128
#define ALTURA 64

Adafruit_SSD1306 oled(LARGURA, ALTURA, &Wire, -1);

// =============== VARIAVEIS ===============

Servo servos[4];

float ang[4];

bool modoLento = false;
bool oledOk = false;
bool optionsAnterior = false;

unsigned long tServo = 0;
unsigned long tOled = 0;

ControllerPtr controles[BP32_MAX_GAMEPADS];

// =============== BLUETOOTH ===============

void aoConectar(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controles[i] == nullptr) {
      controles[i] = ctl;

      Serial.println("Controle PS5 conectado!");
      return;
    }
  }
}

void aoDesconectar(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controles[i] == ctl) {
      controles[i] = nullptr;
      optionsAnterior = false;

      Serial.println("Controle PS5 desconectado!");
      return;
    }
  }
}

// =============== MOVIMENTACAO ===============

void moverServo(uint8_t i, float novo) {
  // Impede que o servo ultrapasse os limites configurados.
  ang[i] = constrain(novo, LIM_MIN[i], LIM_MAX[i]);

  servos[i].write((int)ang[i]);
}

void irParaHome() {
  for (uint8_t i = 0; i < 4; i++) {
    moverServo(i, HOME_POS[i]);
  }
}

float lerEixo(int valor) {
  if (abs(valor) < ZONA_MORTA) {
    return 0;
  }

  return constrain(valor / 512.0f, -1.0f, 1.0f);
}

// =============== TELA OLED ===============

void desenharTela(bool conectado) {
  if (!oledOk) return;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 0);
  oled.print("Controle PS5 ");
  oled.print(conectado ? "OK" : "--");

  oled.setCursor(0, 10);
  oled.print("Velocidade: ");
  oled.print(modoLento ? "LENTA" : "NORMAL");

  oled.drawLine(0, 19, 127, 19, SSD1306_WHITE);

  for (uint8_t i = 0; i < 4; i++) {
    int y = 22 + i * 10;

    oled.setCursor(0, y);
    oled.print(NOMES[i]);

    oled.setCursor(38, y);
    oled.print((int)ang[i]);

    float proporcao =
      (ang[i] - LIM_MIN[i]) /
      (LIM_MAX[i] - LIM_MIN[i]);

    int larguraBarra = (int)(proporcao * 48);

    oled.drawRect(75, y, 52, 7, SSD1306_WHITE);

    if (larguraBarra > 0) {
      oled.fillRect(
        77, y + 2,
        larguraBarra, 3,
        SSD1306_WHITE
      );
    }
  }

  oled.display();
}

void telaAguardando() {
  if (!oledOk) return;

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 0);
  oled.println("MeArm + ESP32");
  oled.println();
  oled.println("Controle PS5");
  oled.println("Aguardando conexao...");
  oled.println();
  oled.println("Segure PS + Share");
  oled.println("Luz azul piscando");

  oled.display();
}

// =============== SETUP ===============

void setup() {
  Serial.begin(115200);

  // Inicializa OLED
  Wire.begin(21, 22);

  oledOk = oled.begin(
    SSD1306_SWITCHCAPVCC,
    0x3C
  );

  if (!oledOk) {
    Serial.println(
      "OLED nao encontrado. Confira SDA, SCL e endereco."
    );
  }

  // Inicializa servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (uint8_t i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);

    servos[i].attach(
      PINOS[i],
      500,
      2400
    );

    ang[i] = HOME_POS[i];

    servos[i].write((int)ang[i]);
  }

  // Inicializa Bluepad32
  Serial.printf(
    "Firmware Bluepad32: %s\n",
    BP32.firmwareVersion()
  );

  BP32.setup(
    &aoConectar,
    &aoDesconectar
  );

  BP32.enableVirtualDevice(false);

  Serial.println("Sistema iniciado.");
  Serial.println("Aguardando Controle PS5...");

  telaAguardando();
}

// =============== LOOP PRINCIPAL ===============

void loop() {
  // Sempre processar o Bluetooth
  BP32.update();

  unsigned long agora = millis();

  // Localiza controle conectado
  ControllerPtr c = nullptr;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (
      controles[i] != nullptr &&
      controles[i]->isConnected() &&
      controles[i]->hasData()
    ) {
      c = controles[i];
      break;
    }
  }

  // Sem controle: mantém os servos parados
  if (c == nullptr) {
    optionsAnterior = false;

    if (agora - tOled >= 500) {
      tOled = agora;
      telaAguardando();
    }

    vTaskDelay(1);
    return;
  }

  // Atualiza movimentos a cada 20 ms
  if (agora - tServo >= 20) {
    tServo = agora;

    float passo = modoLento
      ? PASSO_LENTO
      : PASSO_NORMAL;

    // BASE: analógico esquerdo X
    moverServo(
      0,
      ang[0] +
      SENTIDO[0] * lerEixo(c->axisX()) * passo
    );

    // OMBRO: analógico esquerdo Y
    moverServo(
      1,
      ang[1] -
      SENTIDO[1] * lerEixo(c->axisY()) * passo
    );

    // COTOVELO: analógico direito Y
    moverServo(
      2,
      ang[2] -
      SENTIDO[2] * lerEixo(c->axisRY()) * passo
    );

    // GARRA: R1 abre e L1 fecha
    if (c->r1()) {
      moverServo(
        3,
        ang[3] + SENTIDO[3] * PASSO_GARRA
      );
    }

    if (c->l1()) {
      moverServo(
        3,
        ang[3] - SENTIDO[3] * PASSO_GARRA
      );
    }

    // QUADRADO: retorna a posicao inicial
    if (c->x()) {
      irParaHome();
    }

    // OPTIONS: alterna velocidade ao pressionar
    bool options = c->miscStart();

    if (options && !optionsAnterior) {
      modoLento = !modoLento;

      Serial.print("Velocidade: ");
      Serial.println(
        modoLento ? "LENTA" : "NORMAL"
      );
    }

    optionsAnterior = options;
  }

  // Atualiza OLED a cada 100 ms
  if (agora - tOled >= 100) {
    tOled = agora;
    desenharTela(true);
  }

  vTaskDelay(1);
}
