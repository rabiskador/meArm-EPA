/*
  MeArm + ESP32 - controle pelo Monitor Serial (sem OLED, sem PS5)

  Usa a calibração dos servos e a cinemática inversa (IK) da biblioteca meArm
  (York Hackspace / Nick Moriarty, licença MIT), portadas para o ESP32.
  NÃO precisa instalar a biblioteca meArm: tudo está neste arquivo.
  Só precisa da biblioteca ESP32Servo (Gerenciador de Bibliotecas).

  LIGAÇÕES
    Sinal (laranja): BASE D4 | OMBRO D5 | COTOVELO D18 | GARRA D19
    Servos vermelho -> +5V da fonte externa
    Servos marrom   -> GND da fonte externa + GND do ESP32

  COMANDOS (Monitor Serial em 115200; pode digitar vários de uma vez, ex.: iii)
    Articulações (graus do servo):
      q / a  base      + / -
      w / s  ombro     + / -
      e / d  cotovelo  + / -
      r / f  garra     + / -
    Posição da garra no espaço (mm, medidos a partir do centro da base):
      i / k  frente / trás
      j / l  esquerda / direita
      u / n  sobe / desce
    Outros:
      h  home (0, 100, 50)      o  abre a garra      c  fecha a garra
      m  demonstração           x  para a demonstração
      t  passo 1 / 5 / 10       z  limites da biblioteca <-> modo livre (10 a 170)
      p  mostra a posição       ?  ajuda
*/

#include <ESP32Servo.h>
#include <math.h>

// ---------------- PINOS ----------------
const uint8_t PINOS[4] = {4, 5, 18, 19};
const char*   NOMES[4] = {"BASE", "OMBRO", "COTOVELO", "GARRA"};
enum { BASE = 0, OMBRO = 1, COTOVELO = 2, GARRA = 3 };

// ---------------- GEOMETRIA DO BRAÇO (mm) ----------------
// Valores do meArm padrão (ver Geometry.md da biblioteca)
float L1 = 80;   // ombro -> cotovelo
float L2 = 80;   // cotovelo -> pulso
float L3 = 68;   // pulso -> ponta da garra + centro da base -> ombro

// ---------------- CALIBRAÇÃO DOS SERVOS ----------------
// nMin/nMax = graus do servo quando a junta está em aMin/aMax (radianos).
// São os valores originais da biblioteca meArm. Se o SEU braço for montado
// diferente, é aqui (e em LIM_MIN/LIM_MAX) que você ajusta.
struct Calib { float nMin, nMax, aMin, aMax, gain, zero; };
Calib cal[3] = {
  { 145,  49, -PI / 4,     PI / 4, 0, 0 },   // BASE
  { 118,  22,  PI / 4, 3 * PI / 4, 0, 0 },   // OMBRO
  { 144,  36,  PI / 4,    -PI / 4, 0, 0 }    // COTOVELO
};

const float GARRA_ABERTA  = 75;
const float GARRA_FECHADA = 115;

// Limites de cada servo em graus (BASE, OMBRO, COTOVELO, GARRA)
float LIM_MIN[4] = {  49,  22,  36,  75 };
float LIM_MAX[4] = { 145, 118, 144, 115 };
bool  modoLivre  = false;
float limMin(uint8_t i) { return modoLivre ? 10.0f  : LIM_MIN[i]; }
float limMax(uint8_t i) { return modoLivre ? 170.0f : LIM_MAX[i]; }

// ---------------- ESTADO ----------------
Servo servos[4];
float ang[4];                    // graus atuais de cada servo
float cx = 0, cy = 100, cz = 50; // posição atual da garra (mm)
const int PASSOS[3] = {1, 5, 10};
uint8_t idxPasso = 2;            // começa em 10
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

void checarAbortar() {
  while (Serial.available()) {
    if (Serial.read() == 'x') abortar = true;
  }
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
void imprimir() {
  Serial.printf("Servos: BASE=%d OMBRO=%d COTOVELO=%d GARRA=%d | Garra em x=%d y=%d z=%d mm | passo=%d%s\n",
                (int)(ang[BASE] + 0.5f), (int)(ang[OMBRO] + 0.5f),
                (int)(ang[COTOVELO] + 0.5f), (int)(ang[GARRA] + 0.5f),
                (int)lroundf(cx), (int)lroundf(cy), (int)lroundf(cz),
                PASSOS[idxPasso], modoLivre ? " | MODO LIVRE" : "");
}

void ajuda() {
  Serial.println();
  Serial.println("Juntas : q/a base | w/s ombro | e/d cotovelo | r/f garra");
  Serial.println("Espaco : i/k frente/tras | j/l esquerda/direita | u/n sobe/desce");
  Serial.println("Outros : h home | o abre | c fecha | m demo | x para | t passo | z modo livre | p posicao | ? ajuda");
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

void demo() {
  abortar = false;
  Serial.println("Demo iniciada (x para parar)");
  for (int i = 0; i < 3 && !abortar; i++) { abrirGarra(); fecharGarra(); checarAbortar(); }
  if (!abortar) abrirGarra();
  if (!abortar) delay(500);
  if (!abortar) irSuave(-80, 100, 140);   // sobe e vai para a esquerda
  if (!abortar) fecharGarra();
  if (!abortar) irSuave(70, 200, 10);     // desce, avança e vai para a direita
  if (!abortar) abrirGarra();
  if (!abortar) irSuave(0, 100, 50);      // volta ao home
  Serial.println(abortar ? "Demo interrompida" : "Demo finalizada");
  imprimir();
}

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

  // Posição inicial: home (0, 100, 50) com a garra aberta. Atenção: os servos
  // vão direto para essa posição ao ligar.
  irDireto(0, 100, 50);
  escreverServo(GARRA, GARRA_ABERTA);

  Serial.println("MeArm pronto.");
  ajuda();
  imprimir();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= 'A' && c <= 'Z') c += 32;   // aceita maiúsculas também
    switch (c) {
      case 'q': moverJunta(BASE, +1);     break;
      case 'a': moverJunta(BASE, -1);     break;
      case 'w': moverJunta(OMBRO, +1);    break;
      case 's': moverJunta(OMBRO, -1);    break;
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
        if (!irSuave(0, 100, 50)) Serial.println("Home fora do alcance (confira limites)");
        imprimir();
        break;
      case 'o': abrirGarra();  imprimir(); break;
      case 'c': fecharGarra(); imprimir(); break;
      case 'm': demo(); break;
      case 't':
        idxPasso = (idxPasso + 1) % 3;
        Serial.printf("Passo: %d (graus nas juntas, mm no espaco)\n", PASSOS[idxPasso]);
        break;
      case 'z':
        modoLivre = !modoLivre;
        Serial.println(modoLivre ? "MODO LIVRE (10 a 170 graus): cuidado, sem protecao da biblioteca!"
                                 : "Limites da biblioteca ATIVOS");
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
}
