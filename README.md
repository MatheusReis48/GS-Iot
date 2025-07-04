## 🔧 Componentes Utilizados

- ESP32 DevKit v1
- Sensor DHT22 (Temperatura e Umidade)
- Botão (com resistor pull-up interno)
- LED (indicador de alerta)
- Broker MQTT público: `broker.hivemq.com`

## 🧩 Diagrama de Conexões

- **DHT22**
  - VCC → 3V3
  - GND → GND
  - DATA → GPIO 4 (D4)

- **Botão**
  - Um terminal → GPIO 15 (D15)
  - Outro terminal → GND

- **LED**
  - Anodo (+) → GPIO 2 (D2) através de resistor de 220Ω
  - Catodo (–) → GND

## 🖥️ Código-Fonte

O código principal está no arquivo `sketch.ino`. Ele realiza as seguintes funções:

- Conecta-se à rede Wi-Fi `Wokwi-GUEST`.
- Conecta-se ao broker MQTT `broker.hivemq.com`.
- Lê dados de temperatura e umidade a cada 10 segundos e publica no tópico `sos_climatech_simple/env/data`.
- Detecta o pressionamento do botão e, ao ser pressionado:
  - Envia um alerta no tópico `sos_climatech_simple/alert/trigger`.
  - Acende o LED conectado ao GPIO 2.

## 🚀 Como Executar

1. Clone este repositório:
   ```bash
   git clone https://github.com/MatheusReis48/GS-Iot.git
   ```

2. Abra o projeto na [Wokwi](https://wokwi.com/) ou na IDE Arduino.

3. Certifique-se de que as bibliotecas necessárias estão instaladas:
   - `WiFi.h`
   - `PubSubClient.h`
   - `DHT.h`
   - `ArduinoJson.h`

4. Compile e carregue o código no ESP32.

## 📂 Arquivos

- `sketch.ino`: Código principal do projeto.
- `diagram.json`: Diagrama de conexões para simulação no Wokwi.
- `flow-node.json`: Arquivo de fluxo para integração com Node-RED (opcional).
