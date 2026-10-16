# 🤖 MeArm ESP32 — Controle PS5 com OLED

Projeto de controle de um braço robótico **MeArm utilizando ESP32, controle PlayStation 5 (DualSense), servomotores e display OLED SSD1306**, com comunicação Bluetooth por meio da biblioteca Bluepad32.

O sistema permite movimentar os eixos do braço robótico em tempo real, controlar a abertura e o fechamento da garra, alternar a velocidade de movimento e acompanhar os ângulos dos servos pelo display OLED.

## 🎯 Objetivos do projeto

- Controlar um braço robótico MeArm com um controle DualSense.
- Utilizar Bluetooth para comunicação entre o controle e o ESP32.
- Controlar quatro servomotores de forma independente.
- Implementar limites de movimentação para reduzir o risco de colisões mecânicas.
- Exibir o estado da conexão, a velocidade e os ângulos dos servos em um display OLED.
- Desenvolver conhecimentos de robótica, sistemas embarcados, programação C++ e comunicação sem fio.

## 🧰 Componentes utilizados

| Componente | Função |
|---|---|
| ESP32 DevKit V1 — ESP-WROOM-32 | Controlador principal |
| Controle PS5 DualSense | Interface de controle |
| 4 servomotores | Movimentação da base, ombro, cotovelo e garra |
| Display OLED SSD1306 I2C 128 × 64 | Interface de monitoramento |
| Fonte externa de 5 V | Alimentação dos servos |
| Cabos jumper e protoboard | Conexões elétricas |
| Estrutura MeArm | Estrutura mecânica do braço |

## 🔌 Ligações elétricas

### Servomotores

| Eixo | GPIO do ESP32 |
|---|---|
| Base | GPIO 4 |
| Ombro | GPIO 5 |
| Cotovelo | GPIO 18 |
| Garra | GPIO 19 |

Cada servo possui três conexões: sinal, alimentação positiva e GND. Confira as cores dos fios conforme o modelo do servo.

### Display OLED SSD1306

| OLED | ESP32 |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| VCC | 3V3 |
| GND | GND |

O código utiliza o endereço I2C `0x3C`. Alguns módulos podem utilizar outro endereço.

### Alimentação dos servos

- Fios positivos dos servos: positivo da fonte externa de 5 V.
- GND dos servos: negativo da fonte externa.
- GND da fonte externa: GND do ESP32, formando uma referência comum.
- O ESP32 pode ser alimentado por USB durante os testes, desde que a fonte externa dos servos seja adequada.

**Não alimente os quatro servos diretamente pelo pino 3V3 do ESP32.** A fonte deve suportar a corrente exigida pelos motores, inclusive os picos de partida. Evite conectar duas fontes de 5 V em paralelo.

## 🎮 Mapeamento dos controles

| Controle DualSense | Ação |
|---|---|
| Analógico esquerdo — eixo X | Girar a base |
| Analógico esquerdo — eixo Y | Movimentar o ombro |
| Analógico direito — eixo Y | Movimentar o cotovelo |
| R2 | Abrir a garra |
| L2 | Fechar a garra |
| Quadrado | Retornar à posição inicial |
| Options | Alternar velocidade lenta/normal |

Os sentidos de movimento podem ser invertidos individualmente por meio da configuração `SENTIDO` no código.

## 📐 Limites de movimentação

Os limites abaixo são os valores iniciais configurados no programa. Os limites da base e do cotovelo ainda precisam ser validados fisicamente.

| Eixo | Mínimo | Máximo | Posição inicial |
|---|---:|---:|---:|
| Base | 60° | 120° | 90° |
| Ombro | 10° | 90° | 50° |
| Cotovelo | 70° | 110° | 90° |
| Garra | 10° | 70° | 10° |

A posição inicial é definida pela variável `HOME_POS`. Os limites são definidos por `LIM_MIN` e `LIM_MAX`.

**Atenção:** os limites são provisórios, não uma garantia de segurança mecânica. Verifique o sentido, o curso e os pontos de colisão de cada eixo antes de operar o braço. A faixa da garra deverá ser ampliada somente após confirmar que o mecanismo permite isso sem forçar.

## 🖥️ Display OLED

O display apresenta:

- Estado da conexão do Controle PS5.
- Modo de velocidade selecionado.
- Ângulo atual de cada servomotor.
- Barras gráficas representando a posição de cada eixo.

Quando o controle não está conectado, a tela exibe uma mensagem de espera.

## ⚙️ Tecnologias e bibliotecas

- **C++ / Arduino:** lógica de controle e programação do ESP32.
- **Bluepad32:** comunicação com controles Bluetooth compatíveis.
- **ESP32Servo:** geração dos sinais de controle dos servos.
- **Adafruit SSD1306:** gerenciamento do display OLED.
- **Adafruit GFX Library:** recursos gráficos para a interface.
- **Wire:** comunicação I2C com o display.

## 🚀 Instalação e configuração

### 1. Preparar a Arduino IDE

Instale a Arduino IDE e adicione a URL do pacote de placas Bluepad32 em:

`Arquivo → Preferências → URLs adicionais para Gerenciadores de Placas`

URL:

`https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`

### 2. Instalar a placa

Abra o Gerenciador de Placas, procure pelo pacote `esp32_bluepad32` e instale uma versão compatível com o projeto.

Selecione a placa ESP32 Dev Module correspondente ao pacote instalado.

### 3. Instalar as bibliotecas

No Gerenciador de Bibliotecas, instale:

- ESP32Servo
- Adafruit SSD1306
- Adafruit GFX Library

Utilize o Bluepad32 fornecido pelo pacote de placas, evitando instalações duplicadas incompatíveis.

### 4. Carregar o programa

1. Conecte o ESP32 ao computador por USB.
2. Selecione a porta serial correta.
3. Abra o código do projeto.
4. Compile e envie o programa.
5. Abra o Monitor Serial em 115200 baud para acompanhar as mensagens de inicialização.

### 5. Conectar o Controle PS5

1. Desligue o controle, caso esteja ligado.
2. Pressione simultaneamente os botões **PS + Create** até a luz começar a piscar rapidamente.
3. Aguarde a conexão Bluetooth com o ESP32.
4. Confira a mensagem de conexão no Monitor Serial e no OLED.

O pareamento pode variar conforme o estado anterior do controle e as chaves Bluetooth armazenadas.

## 🛡️ Segurança e cuidados

- Faça os primeiros testes sem carga e com o braço livre de obstáculos.
- Mantenha a mão próxima ao interruptor da alimentação dos servos.
- Não force os servos contra os limites mecânicos.
- Utilize uma fonte de alimentação dimensionada para os motores.
- Confira o GND comum entre ESP32 e fonte externa.
- Desconecte a alimentação antes de alterar a fiação.
- Valide os limites mecânicos antes de aumentar a faixa de movimentação.
- Os limites definidos no código não substituem proteções físicas ou uma parada de emergência.

## 🔧 Melhorias futuras

- Calibração individual dos limites mecânicos.
- Ajuste independente da velocidade de cada eixo.
- Inclusão de uma parada de emergência física.
- Exibição do estado individual dos eixos no OLED.
- Gravação e reprodução de posições do braço.
- Criação de sequências automáticas de movimentos.
- Implementação de modos manual e automático.

## 📚 Aprendizados

Este projeto reúne conhecimentos de programação embarcada, comunicação Bluetooth, controle de servomotores, interfaces I2C, desenvolvimento de sistemas interativos e integração entre hardware e software.

É um projeto em desenvolvimento, com foco em aprendizado prático e evolução gradual da segurança e da precisão dos movimentos.

---

**Status:** Em desenvolvimento  
**Plataforma:** ESP32 DevKit V1  
**Controle:** PlayStation 5 DualSense  
**Linguagem:** C++ (Arduino)  
**Comunicação:** Bluetooth via Bluepad32
