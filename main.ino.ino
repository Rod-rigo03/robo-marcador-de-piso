#include <AccelStepper.h>    // Biblioteca para controle de motores de passo com drivers (STEP/DIR).
#include <ESP8266WiFi.h>     // Biblioteca para gerenciar Wi-Fi no ESP8266.

// -------------------------------------------------
// ========== Configuração da rede Wi-Fi (AP) =======
// -------------------------------------------------
const char* ssid = "ESP_ROBO";        // Nome (SSID) da rede que o ESP criará.
const char* password = "12345678";    // Senha da rede.
WiFiServer server(8080);              // Cria um servidor TCP na porta 8080.

// -------------------------------------------------
// ========== Pinos e definição de motores =========
// -------------------------------------------------
// Define pinos usados como STEP e DIR para cada eixo.

#define X_STEP D2
#define X_DIR  D5
#define Y_STEP D3
#define Y_DIR  D6
#define Z_STEP D4
#define Z_DIR  D7
#define A_STEP D12
#define A_DIR  D13
#define ENABLE_PIN D8   // Pino usado para ENABLE dos drivers (ativa/desativa saída dos motores).

// Instânciamos objetos AccelStepper para cada motor.
// Usamos AccelStepper::DRIVER para dizer que esses pinos serão controlados por um driver no caso o DRV8255
AccelStepper motor_X(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper motor_Y(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper motor_Z(AccelStepper::DRIVER, Z_STEP, Z_DIR);
AccelStepper motor_A(AccelStepper::DRIVER, A_STEP, A_DIR);

// -------------------------------------------------
// ========== Variáveis de controle de estado =======
// -------------------------------------------------
bool movimentoEmAndamento = false;    // Se true -> um movimento foi comandado e está em execução.
bool movimentoIniciado = false;       // Marca quando já foi enviado 'Movimento iniciado' ao cliente.
bool modoCruz = false;                // Se true -> o robô está executando a rotina "CRUZ".
bool movimentoConcluido = false;      // Marca fim da rotina 'CRUZ'.
int etapa = 0;                        // Etapa atual da rotina CRUZ (0..7).
unsigned long ultimaAtualizacao = 0;  // Timestamp para prints periódicos (millis()).
unsigned long ultimoEnvio = 0;        // Timestamp para limitar a frequência de envio de progresso ao cliente.

WiFiClient clientAtivo;               // Guarda o client que está conectado para enviar mensagens de volta.

long inicioX = 0, inicioY = 0, inicioZ = 0, inicioA = 0; // Posições iniciais antes do movimento (para cálculo %).
long destinoX = 0, destinoY = 0, destinoZ = 0, destinoA = 0; // Posições alvo definidas por comandos.

bool moveX = false, moveY = false, moveZ = false, moveA = false; // Flags se cada eixo deve se mover.
int num_passos = 400;                 // Quantidade de passos usada pela rotina CRUZ (padrão).

// ====================== setup() ======================
// Função chamada ao ligar/reiniciar a placa.
// Configura Wi-Fi, cria rede, inicia servidor e configura pinos/motores.
void setup() {
  Serial.begin(115200);               // Inicia porta serial para debug.
  WiFi.mode(WIFI_AP);                 // Ajusta o ESP para modo Access Point (criar rede).
  WiFi.softAP(ssid, password);        // Cria a rede com SSID e senha definidos.

  IPAddress myIP = WiFi.softAPIP();   // Recupera o IP do ESP no modo AP (geralmente 192.168.4.1).
  Serial.println("Rede criada!");
  Serial.print("Nome: "); Serial.println(ssid);
  Serial.print("Senha: "); Serial.println(password);
  Serial.print("IP do ESP: "); Serial.println(myIP);

  server.begin();                     // Inicia o servidor TCP para aceitar conexões.

  pinMode(ENABLE_PIN, OUTPUT);        // Configura pino de ENABLE como saída.
  digitalWrite(ENABLE_PIN, HIGH);     // Desabilita os drivers por padrão (HIGH = desabilitado).

  configurarMotores();                // Ajusta aceleração e velocidade máximas.
}


// Função principal que roda continuamente.
// Aguarda receber comandos e executa movimentos.
void loop() {
  WiFiClient client = server.available(); // Verifica se há um cliente TCP conectado ao servidor.

  if (client) {
    // Quando um cliente se conecta, guardamos o client em clientAtivo para enviar respostas async.
    Serial.println("Cliente conectado!");
    clientAtivo = client;

    // Enquanto o cliente permanecer conectado:
    while (client.connected()) {
      // Se o cliente enviou dados:
      if (client.available()) {
        String data = client.readStringUntil('\n'); // Lê até a quebra de linha.
        data.trim();                               // Remove espaços extras.

        Serial.println("\nDados recebidos do Python:");
        Serial.println(data);

        // --- Comando de emergência: STOP ---
        if (data.equalsIgnoreCase("STOP")) {
          digitalWrite(ENABLE_PIN, HIGH); // Desativa drivers (parar motores)
          movimentoEmAndamento = false;
          movimentoIniciado = false;
          modoCruz = false;
          client.println("Motores parados.");
          continue; // volta ao loop para ler próximo comando
        }

        // --- Comando especial: CRUZ (rotina predefinida) ---
        if (data.equalsIgnoreCase("CRUZ")) {
          Serial.println("Ativando modo CRUZ...");
          client.println("Executando movimento em CRUZ...");
          iniciarMovimentoCruz(); // prepara e inicia a rotina CRUZ
          modoCruz = true;
          continue;
        }

        // --- Movimento normal recebido via string de parâmetros ---
        // Espera formato parecido com "VX=100;VY=100;DX=800;DY=0;..." etc.
        float xVel = getValueFloat(data, "VX"); // velocidade máxima para cada motor
        float yVel = getValueFloat(data, "VY");
        float zVel = getValueFloat(data, "VZ");
        float aVel = getValueFloat(data, "VA");

        // Passos solicitados em cada eixo (podem ser positivos ou negativos)
        int xSteps = int(getValueFloat(data, "DX"));
        int ySteps = int(getValueFloat(data, "DY"));
        int zSteps = int(getValueFloat(data, "DZ"));
        int aSteps = int(getValueFloat(data, "DA"));

        Serial.println("Valores interpretados:");
        Serial.printf("  DX = %d | DY = %d | DZ = %d | DA = %d\n", xSteps, ySteps, zSteps, aSteps);

        // Define se cada eixo tem movimento a executar
        moveX = (xSteps != 0);
        moveY = (ySteps != 0);
        moveZ = (zSteps != 0);
        moveA = (aSteps != 0);

        // Se algum eixo tem comando, ativa o movimento:
        if (moveX || moveY || moveZ || moveA) {
          digitalWrite(ENABLE_PIN, LOW); // Ativa drivers (LOW = ativo em muitos drivers)
          // Salva posições de início (usadas para calcular progresso %)
          inicioX = motor_X.currentPosition();
          inicioY = motor_Y.currentPosition();
          inicioZ = motor_Z.currentPosition();
          inicioA = motor_A.currentPosition();

          // Define destinos relativos (adiciona aos valores atuais)
          destinoX = inicioX + xSteps;
          destinoY = inicioY + ySteps;
          destinoZ = inicioZ + zSteps;
          destinoA = inicioA + aSteps;

          // Define velocidade máxima para cada motor (unidade: passos por segundo)
          motor_X.setMaxSpeed(xVel);
          motor_Y.setMaxSpeed(yVel);
          motor_Z.setMaxSpeed(zVel);
          motor_A.setMaxSpeed(aVel);

          // Comanda o movimento até a posição alvo
          motor_X.moveTo(destinoX);
          motor_Y.moveTo(destinoY);
          motor_Z.moveTo(destinoZ);
          motor_A.moveTo(destinoA);

          movimentoEmAndamento = true;   // Sinaliza que há movimento em andamento
          movimentoIniciado = false;     // Reinicia flag para enviar "movimento iniciado"
        } else {
          client.println("Nenhum parâmetro válido recebido.");
        }
      } 

      // Chama execução da rotina conforme o modo atual:
      if (modoCruz) executarCruz();
      else if (movimentoEmAndamento) executarMovimentoPadrao();
    } // fim while client.connected

    client.stop(); // cliente desconectou: fecha conexão
    Serial.println("Cliente desconectado.");
  } 
}

// =================== FUNÇÕES AUXILIARES ===================

// configurarMotores: define aceleração e velocidade máxima iniciais
void configurarMotores() {
  motor_X.setAcceleration(800); // aceleração em passos/s² 
  motor_Y.setAcceleration(800);
  motor_Z.setAcceleration(800);
  motor_A.setAcceleration(800);

  motor_X.setMaxSpeed(1600); // velocidade máxima (passos/s) padrão de segurança
  motor_Y.setMaxSpeed(1600);
  motor_Z.setMaxSpeed(1600);
  motor_A.setMaxSpeed(1600);
}

// resetarPosicoes: zera as posições internas dos motores (útil ao iniciar rotinas)
void resetarPosicoes() {
  motor_X.setCurrentPosition(0);
  motor_Y.setCurrentPosition(0);
  motor_Z.setCurrentPosition(0);
  motor_A.setCurrentPosition(0);
}

// iniciarMovimentoCruz: prepara e inicia a rotina predefinida "CRUZ"
void iniciarMovimentoCruz() {
  digitalWrite(ENABLE_PIN, LOW); // ativa drivers
  configurarMotores();           // garante parâmetros consistentes
  resetarPosicoes();             // zera posição atual

  etapa = 0;
  movimentoConcluido = false;
  movimentoEmAndamento = true;

  // Primeira ação: mover X para frente e A no sentido oposto, por num_passos
  motor_X.move(num_passos);
  motor_A.move(-num_passos);
  Serial.println("Etapa 1: Indo para frente...");
}

// executarCruz: rotina que caminha por etapas pré-definidas até completar o "X" em cruz
void executarCruz() {
  // Se já acabou e modoCruz foi desligado, sai
  if (movimentoConcluido && !modoCruz) return;

  if (movimentoEmAndamento) {
    // Chama run() frequentemente para gerar os pulsos STEP (não bloqueante).
    motor_X.run();
    motor_Y.run();
    motor_Z.run();
    motor_A.run();

    // Atualizações periódicas de progresso exibidas no Serial
    if (millis() - ultimaAtualizacao >= 100) {
      ultimaAtualizacao = millis();

      float percX = 0, percA = 0, percY = 0, percZ = 0;

      if (num_passos != 0) {
        // Calcula percentual simples baseado na posição absoluta em relação ao num_passos.
        percX = abs(motor_X.currentPosition()) * 100.0 / num_passos;
        percA = abs(motor_A.currentPosition()) * 100.0 / num_passos;
        percY = abs(motor_Y.currentPosition()) * 100.0 / num_passos;
        percZ = abs(motor_Z.currentPosition()) * 100.0 / num_passos;
      }

      // Exibe no Serial dependendo da fase (radial vs tangencial)
      if (etapa <= 3)
        Serial.printf("Progresso (Radial): X: %.0f%% | A: %.0f%%\n", percX, percA);
      else if (etapa >= 4 && etapa <= 7)
        Serial.printf("Progresso (Tangencial): Y: %.0f%% | Z: %.0f%%\n", percY, percZ);
    }

    // Verifica se todos os motores pararam (distanceToGo() == 0)
    if (todosMotoresParados()) {
      movimentoEmAndamento = false;
      Serial.println("Etapa concluída!");
      delay(500); // pequena pausa entre etapas para estabilizar
      etapa++;    // avança para a próxima etapa

      // Define ações da etapa seguinte (switch ordena sequências de ida/volta para formar cruz)
      switch (etapa) {
        case 1: motor_X.move(-num_passos); motor_A.move(num_passos); Serial.println("Etapa 2: Voltando frente"); break;
        case 2: motor_X.move(-num_passos); motor_A.move(num_passos); Serial.println("Etapa 3: Indo trás"); break;
        case 3: motor_X.move(num_passos); motor_A.move(-num_passos); Serial.println("Etapa 4: Voltando trás"); break;
        case 4: motor_Y.move(num_passos); motor_Z.move(-num_passos); Serial.println("Etapa 5: Indo direita"); break;
        case 5: motor_Y.move(-num_passos); motor_Z.move(num_passos); Serial.println("Etapa 6: Voltando centro"); break;
        case 6: motor_Y.move(-num_passos); motor_Z.move(num_passos); Serial.println("Etapa 7: Indo esquerda"); break;
        case 7: motor_Y.move(num_passos); motor_Z.move(-num_passos); Serial.println("Etapa 8: Voltando centro"); break;

        default:
          // Quando não existem mais etapas: finaliza rotina CRUZ
          movimentoConcluido = true;
          modoCruz = false;
          digitalWrite(ENABLE_PIN, HIGH); // desliga motores

          // Notifica cliente conectado (se existir)
          if (clientAtivo && clientAtivo.connected()) {
            clientAtivo.println("Movimento concluído.");
            delay(100);
            clientAtivo.stop(); // fecha a conexão do lado do servidor
          }

          Serial.println("Movimento em CRUZ finalizado!");
          break;
      } // fim switch

      // Caso haja nova etapa, reativa o estado de movimento para continuar
      if (!movimentoConcluido) movimentoEmAndamento = true;
    } // fim if todosMotoresParados
  } // fim if movimentoEmAndamento
} // fim executarCruz

// executarMovimentoPadrao: função que roda movimentos arbitrários enviados pela rede
void executarMovimentoPadrao() {
  // Chama run() constantemente para cada motor realizar o movimento definido por moveTo()
  motor_X.run();
  motor_Y.run();
  motor_Z.run();
  motor_A.run();

  // Se ainda não enviamos a mensagem de início, envia ao cliente
  if (!movimentoIniciado) {
    clientAtivo.println("Movimento iniciado...");
    movimentoIniciado = true;
  }

  // Imprime progresso no Serial a cada 100 ms
  if (millis() - ultimaAtualizacao >= 100) {
    ultimaAtualizacao = millis();

    auto printPerc = [](AccelStepper& m, long inicio, long destino, const char* nome) {
      float total = abs(destino - inicio);                 // total de passos a percorrer
      float atual = abs(m.currentPosition() - inicio);     // passos já percorridos desde o início
      if (total > 0) {
        float perc = (atual / total) * 100.0;
        Serial.printf("%s: %.0f%% | ", nome, perc);
      }
    };

    // Usa lambda para reduzir repetição e imprimir as 4 leituras
    printPerc(motor_X, inicioX, destinoX, "X");
    printPerc(motor_Y, inicioY, destinoY, "Y");
    printPerc(motor_Z, inicioZ, destinoZ, "Z");
    printPerc(motor_A, inicioA, destinoA, "A");
    Serial.println();
  }

  // Se todos motores concluíram o deslocamento:
  if (todosMotoresParados()) {
    movimentoEmAndamento = false;
    movimentoIniciado = false;
    digitalWrite(ENABLE_PIN, HIGH);   // desliga drivers para não esuqentar os drivers
    clientAtivo.println("Movimento concluído.");
    Serial.println("Movimento concluído!");
  }
}

// getValueFloat: lê valores do formato key=valor;key2=valor2; ...
// Ex.: getValueFloat("VX=100;DX=500;", "DX") -> 500
float getValueFloat(String data, String key) {
  int start = data.indexOf(key + "="); // encontra início de "KEY="
  if (start == -1) return 0.0;         // se não existir, retorna 0
  start += key.length() + 1;           // posiciona após o '='
  int end = data.indexOf(";", start);  // procura o separador ';'
  if (end == -1) end = data.length();  // se não houver ';', pega até o fim da string
  return data.substring(start, end).toFloat(); // converte substring para float
}

// todosMotoresParados: verifica se não há distância a percorrer para nenhum motor
bool todosMotoresParados() {
  return (motor_X.distanceToGo() == 0 &&
          motor_Y.distanceToGo() == 0 &&
          motor_Z.distanceToGo() == 0 &&
          motor_A.distanceToGo() == 0);
}
