/*
  MeArm + ESP32 - PROJETO COMPLETO
  Controle pelo PS3 (DualShock 3 / Sixaxis, via Bluepad32) e pelo Monitor Serial.
 
  Usa a calibração dos servos e a cinemática inversa (IK) da biblioteca meArm
  (York Hackspace / Nick Moriarty, licença MIT), portadas para o ESP32.
  NÃO precisa instalar a biblioteca meArm: tudo está neste arquivo.

  PREPARAÇÃO (Arduino IDE)
    - Placa: "ESP32 + Bluepad32" (pacote esp32_bluepad32) > ESP32 Dev Module
    - Colar em Arquivo -> Preferencias -> URLS = https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
    - Biblioteca: ESP32Servo (Gerenciador de Bibliotecas)
    - Para compilar SEM o controle (qualquer placa ESP32), troque
      USAR_CONTROLE para 0 logo abaixo.

  LIGAÇÕES
    Sinal (laranja): BASE D4 | OMBRO D5 | COTOVELO D18 | GARRA D19
    Servos vermelho -> +5V da fonte externa (2 A ou mais) + capacitor 470-1000 uF
    Servos marrom   -> GND da fonte externa + GND do ESP32

  PAREAR O PS3 (só na primeira vez - o controle PS3 NÃO pareia sozinho):
    1) Abra o Monitor Serial (115200) com o ESP32 ligado e anote o endereço na
       linha "BTstack up and running at XX:XX:XX:XX:XX:XX".
    2) Ligue o controle PS3 por cabo USB no computador.
    3) Rode a ferramenta sixaxispairer (do Bluepad32) ou o SixaxisPairTool
       (Windows) e grave o endereço do passo 1 dentro do controle.
    4) Desconecte o cabo e aperte o botão PS: o ESP32 conecta (pode levar 10 s).

  CONTROLE PS3
    Dois modos (Triângulo alterna; começa em JUNTAS):
      JUNTAS  : analógico esquerdo <-> gira a base
                analógico esquerdo ^v ombro
                analógico direito  ^v cotovelo
      POSICAO : analógico esquerdo move a garra para os lados e para frente/trás
                analógico direito ^v sobe/desce (o código calcula os ângulos)
    R2 abre a garra | L2 fecha a garra (quanto mais apertar o gatilho, mais rápido)
    Círculo abre | X fecha (vai direto)
    Quadrado = home | Triângulo = troca de modo
    Start = velocidade lenta/normal | Select = mostra a posição no Monitor Serial

  MONITOR SERIAL (115200; aperte Enter; pode digitar vários de uma vez, ex.: iii)
    Articulações (graus do servo):
      q / a  base      + / -
      w / s  ombro     sobe / desce   (subir = diminuir os graus)
      e / d  cotovelo  + / -
      r / f  garra     + / -
    Posição da garra no espaço (mm, a partir do centro da base):
      i / k  frente / trás | j / l  esquerda / direita | u / n  sobe / desce
    Outros:
      h home (0,100,50)   o abre   c fecha   m demonstração   x para
      g posição de montagem (para reencaixar as hastes)
      t passo 1/5/10      z modo livre (10 a 170, só para calibrar)
      p mostra a posição  ? ajuda
*/

#define USAR_CONTROLE 1

#if USAR_CONTROLE
#include <Bluepad32.h>
#endif
#include <ESP32Servo.h>
#include <math.h>

// ---------------- PINOS ----------------
const uint8_t PINOS[4] = {4, 21, 18, 19};
const char*   NOMES[4] = {"BASE", "OMBRO", "COTOVELO", "GARRA"};
enum { BASE = 0, OMBRO = 1, COTOVELO = 2, GARRA = 3 };

// ---------------- GEOMETRIA DO BRAÇO (mm) ----------------
// Valores do meArm padrão (ver Geometry.md da biblioteca)
float L1 = 80;   // ombro -> cotovelo
float L2 = 80;   // cotovelo -> pulso
float L3 = 68;   // pulso -> ponta da garra + centro da base -> ombro

// ---------------- CALIBRAÇÃO DOS SERVOS ----------------
// nMin/nMax = graus do servo quando a junta está em aMin/aMax (radianos).
// Valores originais da biblioteca meArm. Posição de montagem de referência:
//   BASE 97 = reta para frente | OMBRO 70 = braço vertical | COTOVELO = antebraço horizontal
//
// COTOVELO_HORIZONTAL = graus do servo com o antebraço na HORIZONTAL.
// Na biblioteca é 90, mas no seu braço o cotovelo anda de 0 (para baixo) a 80
// (para cima), então começamos com 40 (meio da faixa). AJUSTE esse número:
// use o servo_configurar.ino (ou os comandos do Monitor Serial) para achar em
// que ângulo o seu antebraço fica na horizontal e coloque aqui.
const float COTOVELO_HORIZONTAL = 40;

struct Calib { float nMin, nMax, aMin, aMax, gain, zero; };
Calib cal[3] = {
  { 145,  49, -PI / 4,     PI / 4, 0, 0 },   // BASE
  { 118,  22,  PI / 4, 3 * PI / 4, 0, 0 },   // OMBRO
  { COTOVELO_HORIZONTAL + 54, COTOVELO_HORIZONTAL - 54, PI / 4, -PI / 4, 0, 0 }   // COTOVELO
};

// Garra: valores do seu braço
const float GARRA_ABERTA  = 0;
const float GARRA_FECHADA = 80;

// Limites de cada servo em graus
//   OMBRO: 20 a 80 (medido: começa a zumbir em 15)
//   COTOVELO: 5 a 75 (medido: vai de 0 a 80, 0 = antebraço para baixo, 80 = para cima; 5 graus de folga)
//                    BASE OMBRO COTOVELO GARRA
float LIM_MIN[4] = {   10,   20,       5,     0 };
float LIM_MAX[4] = {  150,   80,      75,    80 };
bool  modoLivre  = false;
float limMin(uint8_t i) { if (i == GARRA) return LIM_MIN[GARRA]; return modoLivre ? 10.0f  : LIM_MIN[i]; }
float limMax(uint8_t i) { if (i == GARRA) return LIM_MAX[GARRA]; return modoLivre ? 170.0f : LIM_MAX[i]; }

// Posição inicial / home (mm)
const float HOME_X = 0, HOME_Y = 100, HOME_Z = 50;

// ---------------- ESTADO ----------------
Servo servos[4];
float ang[4];                    // graus atuais de cada servo
float cx = HOME_X, cy = HOME_Y, cz = HOME_Z;   // posição atual da garra (mm)
const int PASSOS[3] = {1, 5, 10};
uint8_t idxPasso = 2;            // passo do Monitor Serial: começa em 10
bool abortar = false;

// ---------------- CONVERSÃO ÂNGULO DA JUNTA <-> GRAUS DO SERVO ----------------
// (o arredondamento para inteiro é feito só na hora de escrever no servo)
float anguloParaServo(const Calib& c, float a) { return c.zero + c.gain * a; }
float servoParaAngulo(const Calib& c, float g) { return (g - c.zero) / c.gain; }

// ---------------- CINEMÁTICA (portada da biblioteca meArm) ----------------
void cart2polar(float a, float b, float& r, float& theta) {
  r = sqrtf(a * a + b * b);
  if (r == 0) { theta = 0; return; }
  theta = atan2f(b, a);
}

// Lei dos cossenos: ângulo oposto ao lado "opp"
bool cosangle(float opp, float adj1, float adj2, float& theta) {
  float den = 2 * adj1 * adj2;
  if (den == 0) return false;
  float c = (adj1 * adj1 + adj2 * adj2 - opp * opp) / den;
  if (c > 1 || c < -1) return false;
  theta = acosf(c);
  return true;
}

// (x, y, z) -> ângulos das 3 juntas em radianos
bool solve(float x, float y, float z, float& a0, float& a1, float& a2) {
  float r, th0;
  cart2polar(y, x, r, th0);
  r -= L3;                                   // desconta o comprimento do pulso

  float angP, R;
  cart2polar(r, z, R, angP);

  float B, C;
  if (!cosangle(L2, L1, R, B)) return false;
  if (!cosangle(R, L1, L2, C)) return false;

  a0 = th0;
  a1 = angP + B;
  a2 = C + a1 - PI;
  return true;
}

// ângulos das juntas -> (x, y, z)
void unsolve(float a0, float a1, float a2, float& x, float& y, float& z) {
  float u = L1 * cosf(a1) + L2 * cosf(a2) + L3;
  float v = L1 * sinf(a1) + L2 * sinf(a2);
  y = u * cosf(a0);
  x = u * sinf(a0);
  z = v;
}

// ---------------- MOVIMENTO ----------------
void escreverServo(uint8_t i, float graus) {
  ang[i] = constrain(graus, limMin(i), limMax(i));
  servos[i].write((int)(ang[i] + 0.5f));
}

void atualizarXYZ() {
  unsolve(servoParaAngulo(cal[BASE], ang[BASE]),
          servoParaAngulo(cal[OMBRO], ang[OMBRO]),
          servoParaAngulo(cal[COTOVELO], ang[COTOVELO]),
          cx, cy, cz);
}

// Calcula os graus dos 3 servos para um ponto. false = fora do alcance do braço.
bool calcularServos(float x, float y, float z, float s[3]) {
  float a0, a1, a2;
  if (!solve(x, y, z, a0, a1, a2)) return false;
  s[BASE]     = anguloParaServo(cal[BASE], a0);
  s[OMBRO]    = anguloParaServo(cal[OMBRO], a1);
  s[COTOVELO] = anguloParaServo(cal[COTOVELO], a2);
  for (uint8_t i = 0; i < 3; i++)
    if (s[i] < limMin(i) - 0.5f || s[i] > limMax(i) + 0.5f) return false;
  return true;
}

bool irDireto(float x, float y, float z) {
  float s[3];
  if (!calcularServos(x, y, z, s)) return false;
  for (uint8_t i = 0; i < 3; i++) escreverServo(i, s[i]);
  cx = x; cy = y; cz = z;
  return true;
}

// Se o ponto não é alcançável, tenta eixo por eixo (o braço "desliza" na borda)
void irDiretoDeslizando(float x, float y, float z) {
  if (irDireto(x, y, z)) return;
  irDireto(x, cy, cz);
  irDireto(cx, y, cz);
  irDireto(cx, cy, z);
}

void checarAbortar() {
  while (Serial.available()) {
    if (Serial.read() == 'x') abortar = true;
  }
#if USAR_CONTROLE
  BP32.update();   // mantém o Bluetooth vivo durante movimentos longos
#endif
}

// Vai em linha reta até o ponto (como o gotoPoint da biblioteca)
bool irSuave(float x, float y, float z) {
  float s[3];
  if (!calcularServos(x, y, z, s)) return false;
  float x0 = cx, y0 = cy, z0 = cz;
  float dist = sqrtf((x - x0) * (x - x0) + (y - y0) * (y - y0) + (z - z0) * (z - z0));
  int passos = (int)(dist / 5.0f);
  if (passos < 1) passos = 1;
  for (int i = 1; i <= passos && !abortar; i++) {
    float t = (float)i / passos;
    irDireto(x0 + (x - x0) * t, y0 + (y - y0) * t, z0 + (z - z0) * t);
    delay(25);
    checarAbortar();
  }
  return true;
}

void abrirGarra()  { escreverServo(GARRA, GARRA_ABERTA);  delay(300); }
void fecharGarra() { escreverServo(GARRA, GARRA_FECHADA); delay(300); }

// ---------------- INTERFACE SERIAL ----------------
#if USAR_CONTROLE
bool modoPosicao = false;   // false = JUNTAS, true = POSICAO (IK)
bool modoLento   = false;
#endif

void imprimir() {
  Serial.printf("Servos: BASE=%d OMBRO=%d COTOVELO=%d GARRA=%d | Garra em x=%d y=%d z=%d mm | passo=%d%s",
                (int)(ang[BASE] + 0.5f), (int)(ang[OMBRO] + 0.5f),
                (int)(ang[COTOVELO] + 0.5f), (int)(ang[GARRA] + 0.5f),
                (int)lroundf(cx), (int)lroundf(cy), (int)lroundf(cz),
                PASSOS[idxPasso], modoLivre ? " | MODO LIVRE" : "");
#if USAR_CONTROLE
  Serial.printf(" | PS3: modo %s, %s", modoPosicao ? "POSICAO" : "JUNTAS", modoLento ? "lento" : "normal");
#endif
  Serial.println();
}

void ajuda() {
  Serial.println();
  Serial.println("Juntas : q/a base | w/s ombro (sobe/desce) | e/d cotovelo | r/f garra");
  Serial.println("Espaco : i/k frente/tras | j/l esquerda/direita | u/n sobe/desce");
  Serial.println("Outros : h home | o abre | c fecha | m demo | x para | g montagem | t passo | z modo livre | p posicao | ? ajuda");
}

void moverJunta(uint8_t eixo, int dir) {
  float antes = ang[eixo];
  escreverServo(eixo, ang[eixo] + dir * PASSOS[idxPasso]);
  if (ang[eixo] == antes) Serial.printf("%s no limite (%d)\n", NOMES[eixo], (int)(antes + 0.5f));
  if (eixo != GARRA) atualizarXYZ();
  imprimir();
}

void moverXYZ(float dx, float dy, float dz) {
  float p = PASSOS[idxPasso];
  if (!irDireto(cx + dx * p, cy + dy * p, cz + dz * p))
    Serial.println("Fora do alcance do braco (ou dos limites dos servos)");
  imprimir();
}

// Ponto da demonstração: vai suave e avisa se não for alcançável
void demoIr(float x, float y, float z) {
  if (abortar) return;
  if (!irSuave(x, y, z))
    Serial.printf("Ponto (%d, %d, %d) fora do alcance, pulando\n", (int)x, (int)y, (int)z);
}

void demo() {
  abortar = false;
  Serial.println("Demo iniciada (x para parar)");
  for (int i = 0; i < 2 && !abortar; i++) { abrirGarra(); fecharGarra(); checarAbortar(); }
  if (!abortar) abrirGarra();
  demoIr(-60, 115, 50);                   // desce à esquerda
  if (!abortar) fecharGarra();            // "pega"
  demoIr(-50, 120, 75);                   // sobe
  demoIr(50, 120, 75);                    // leva para a direita
  demoIr(60, 115, 50);                    // desce
  if (!abortar) abrirGarra();             // "solta"
  demoIr(HOME_X, HOME_Y, HOME_Z);         // volta ao home
  Serial.println(abortar ? "Demo interrompida" : "Demo finalizada");
  imprimir();
}

// Posição de montagem: serve para reencaixar as hastes (ver servo_configurar.ino)
void posicaoMontagem() {
  escreverServo(BASE,     anguloParaServo(cal[BASE], 0));
  escreverServo(OMBRO,    anguloParaServo(cal[OMBRO], PI / 2));
  escreverServo(COTOVELO, anguloParaServo(cal[COTOVELO], 0));
  atualizarXYZ();
  Serial.println("Posicao de montagem: base reta para frente, ombro vertical, antebraco horizontal");
  imprimir();
}

// ---------------- CONTROLE PS3 (Bluepad32) ----------------
#if USAR_CONTROLE
ControllerPtr controles[BP32_MAX_GAMEPADS];

void aoConectar(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controles[i] == nullptr) {
      controles[i] = ctl;
      Serial.printf("Controle conectado (slot %d)\n", i);
      return;
    }
  }
}

void aoDesconectar(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controles[i] == ctl) {
      controles[i] = nullptr;
      Serial.printf("Controle desconectado (slot %d). Servos mantem a posicao.\n", i);
      return;
    }
  }
}

// Analógicos do Bluepad32 vão de -511 a 512 (para baixo e para a direita = positivo)
// (o DS3 manda 0-255; o Bluepad32 já converte para essa faixa)
const int   ZONA_MORTA      = 70;
const float VEL_JUNTA       = 1.5f;   // graus por ciclo de 20 ms, analógico no máximo
const float VEL_JUNTA_LENTA = 0.5f;
const float VEL_MM          = 2.0f;   // mm por ciclo de 20 ms
const float VEL_MM_LENTA    = 0.6f;
const float VEL_GARRA       = 3.0f;   // graus por ciclo com o gatilho todo apertado

float lerEixo(int v) {
  if (abs(v) < ZONA_MORTA) return 0;
  return constrain(v / 512.0f, -1.0f, 1.0f);
}

void processarControle(ControllerPtr c) {
  static bool trianguloAnt = false, optionsAnt = false, createAnt = false, quadradoAnt = false;
  static unsigned long tPrint = 0;

  // ---- botões de ação (só na borda de subida) ----
  bool tri = c->y(), opt = c->miscStart() /*Start*/, cre = c->miscSelect() /*Select*/, quad = c->x();
  if (tri && !trianguloAnt) {
    modoPosicao = !modoPosicao;
    Serial.println(modoPosicao ? "Modo POSICAO (analogicos movem a garra no espaco)"
                               : "Modo JUNTAS (analogicos movem cada servo)");
  }
  if (opt && !optionsAnt) {
    modoLento = !modoLento;
    Serial.println(modoLento ? "Velocidade LENTA" : "Velocidade NORMAL");
  }
  if (cre && !createAnt) imprimir();
  if (quad && !quadradoAnt) {
    Serial.println("Home...");
    abortar = false;
    irSuave(HOME_X, HOME_Y, HOME_Z);
  }
  trianguloAnt = tri; optionsAnt = opt; createAnt = cre; quadradoAnt = quad;

  // ---- analógicos ----
  float lx = lerEixo(c->axisX());     // + = direita
  float ly = -lerEixo(c->axisY());    // + = para cima
  float ry = -lerEixo(c->axisRY());   // + = para cima
  bool moveu = false;

  if (modoPosicao) {
    float v = modoLento ? VEL_MM_LENTA : VEL_MM;
    if (lx != 0 || ly != 0 || ry != 0) {
      irDiretoDeslizando(cx + lx * v, cy + ly * v, cz + ry * v);   // direita/frente/cima = positivo
      moveu = true;
    }
  } else {
    float v = modoLento ? VEL_JUNTA_LENTA : VEL_JUNTA;
    if (lx != 0) { escreverServo(BASE,     ang[BASE]     - lx * v); moveu = true; }   // direita = graus menores
    if (ly != 0) { escreverServo(OMBRO,    ang[OMBRO]    - ly * v); moveu = true; }   // para cima = sobe = graus menores
    if (ry != 0) { escreverServo(COTOVELO, ang[COTOVELO] + ry * v); moveu = true; }   // para cima = antebraço sobe
    if (moveu) atualizarXYZ();
  }

  // ---- garra: L2 fecha, R2 abre (gatilhos de 0 a 1023: quanto mais aperta, mais rápido) ----
  float fecha = c->brake() / 1023.0f;      // L2
  float abre  = c->throttle() / 1023.0f;   // R2
  if (fecha < 0.05f) fecha = 0;            // zona morta dos gatilhos
  if (abre  < 0.05f) abre  = 0;
  if (fecha != 0 || abre != 0) {
    escreverServo(GARRA, ang[GARRA] + (fecha - abre) * VEL_GARRA);   // fechar = graus maiores
    moveu = true;
  }
  if (c->b())  { escreverServo(GARRA, GARRA_ABERTA);  moveu = true; }
  if (c->a())  { escreverServo(GARRA, GARRA_FECHADA); moveu = true; }

  // mostra a posição no monitor enquanto você mexe (a cada 0,5 s)
  if (moveu && millis() - tPrint > 500) { tPrint = millis(); imprimir(); }
}
#endif

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 3; i++) {
    cal[i].gain = (cal[i].nMax - cal[i].nMin) / (cal[i].aMax - cal[i].aMin);
    cal[i].zero = cal[i].nMin - cal[i].gain * cal[i].aMin;
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (uint8_t i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(PINOS[i], 544, 2400);   // mesma faixa de pulso do Servo.h original
  }

  // Posição inicial: home com a garra aberta. Atenção: os servos vão direto
  // para essa posição ao ligar.
  irDireto(HOME_X, HOME_Y, HOME_Z);
  escreverServo(GARRA, GARRA_ABERTA);

#if USAR_CONTROLE
  Serial.printf("Bluepad32 firmware: %s\n", BP32.firmwareVersion());
  BP32.setup(&aoConectar, &aoDesconectar);
  // Se o controle nao reconectar depois de parear em outro aparelho,
  // descomente a linha abaixo, grave uma vez e comente de novo:
  // BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);   // desliga o mouse virtual
#endif

  Serial.println("MeArm pronto.");
  ajuda();
  imprimir();
}

void loop() {
#if USAR_CONTROLE
  BP32.update();   // processa o Bluetooth; chame sempre

  static unsigned long tControle = 0;
  if (millis() - tControle >= 20) {
    tControle = millis();
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      ControllerPtr c = controles[i];
      if (c && c->isConnected()) { processarControle(c); break; }
    }
  }
#endif

  while (Serial.available()) {
    char c = Serial.read();
    if (c >= 'A' && c <= 'Z') c += 32;   // aceita maiúsculas também
    switch (c) {
      case 'q': moverJunta(BASE, +1);     break;
      case 'a': moverJunta(BASE, -1);     break;
      case 'w': moverJunta(OMBRO, -1);    break;   // sobe = diminui os graus
      case 's': moverJunta(OMBRO, +1);    break;   // desce = aumenta os graus
      case 'e': moverJunta(COTOVELO, +1); break;
      case 'd': moverJunta(COTOVELO, -1); break;
      case 'r': moverJunta(GARRA, +1);    break;
      case 'f': moverJunta(GARRA, -1);    break;

      case 'i': moverXYZ(0, +1, 0); break;   // frente
      case 'k': moverXYZ(0, -1, 0); break;   // trás
      case 'j': moverXYZ(-1, 0, 0); break;   // esquerda
      case 'l': moverXYZ(+1, 0, 0); break;   // direita
      case 'u': moverXYZ(0, 0, +1); break;   // sobe
      case 'n': moverXYZ(0, 0, -1); break;   // desce

      case 'h':
        abortar = false;
        if (!irSuave(HOME_X, HOME_Y, HOME_Z)) Serial.println("Home fora do alcance (confira os limites)");
        imprimir();
        break;
      case 'o': abrirGarra();  imprimir(); break;
      case 'c': fecharGarra(); imprimir(); break;
      case 'm': demo(); break;
      case 'g': posicaoMontagem(); break;
      case 't':
        idxPasso = (idxPasso + 1) % 3;
        Serial.printf("Passo: %d (graus nas juntas, mm no espaco)\n", PASSOS[idxPasso]);
        break;
      case 'z':
        modoLivre = !modoLivre;
        Serial.println(modoLivre ? "MODO LIVRE (10 a 170 graus): cuidado, sem protecao dos limites!"
                                 : "Limites ATIVOS");
        if (!modoLivre) {
          for (uint8_t i = 0; i < 4; i++) escreverServo(i, ang[i]);
          atualizarXYZ();
        }
        imprimir();
        break;
      case 'p': imprimir(); break;
      case '?': ajuda();    break;
      default:
        // ignora Enter/espaço; avisa se for outro caractere
        if (c > ' ') Serial.printf("Comando '%c' nao existe (digite ? para ver a lista)\n", c);
        break;
    }
  }

#if USAR_CONTROLE
  vTaskDelay(1);   // evita watchdog
#endif
}
