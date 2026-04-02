# Distributed Real-Time Control Systems Project

This repository contains the software for a distributed illumination control system. The project has been consolidated into a streamlined architecture optimized for the **Raspberry Pi Pico** (RP2040), utilizing hardware dual-core processing and CAN-bus communication.

## Project Structure

The codebase has been refactored from a multi-file architecture into a more cohesive structure:

* **`part2.ino`**: The core application file containing the complete system logic. It integrates all the functional modules:
  * **Auto-Addressing & Boot**: Dynamic node addressing using the Raspberry Pi Pico's hardware Unique ID. The node with the lowest ID automatically becomes the Hub (Master).
  * **Distributed Calibration**: State machines for background illuminance measurement and cross-gain (K-matrix) calibration across the network.
  * **CAN Communication & Protocol**: Asynchronous CAN message passing, network routing, and software loopback for seamless local/network command execution.
  * **PID & Control Loop**: A dedicated `PIDController` class with anti-windup, coupled with a cooperative distributed control algorithm that factors in external illuminance from neighboring nodes.
  * **Dual-Core Execution**: 
    * **Core 0** (`setup()` / `loop()`): Handles CAN messages, state machines (boot/calibration), and the Serial User Interface.
    * **Core 1** (`setup1()` / `loop1()`): Runs the strict 100Hz real-time control loop (sensor reading, PID computation, LED actuation).
    * *Concurrency*: Utilizes hardware mutexes (`pico/mutex.h`) for thread-safe variable sharing between cores, preventing race conditions.
  * **Metrics & UI**: Real-time computation of comfort and performance metrics (Energy, Visibility, and Flicker), accessible via a robust serial command menu at the Hub.

* **`mcp2515.h`**: Header library file for the MCP2515 CAN controller, responsible for SPI communication and CAN physical layer configuration.

## Features overview
* **Plug & Play Network**: Nodes auto-discover each other, share Unique IDs, and assign addresses automatically.
* **Safe Concurrency**: Strict mutex-protected data access between the asynchronous network/UI workloads and the 100Hz synchronous control loop.
* **Cooperative Control**: Dynamically adjusts LED PWM based on local LDR feedback and feed-forward data coming from neighbors over the CAN bus.





Vou ser brutalmente honesto: o teu colega percebe da poda, mas deixou aí umas autênticas "bombas-relógio" que vão fazer o vosso sistema implodir se não forem corrigidas. 

A arquitetura geral é muito ambiciosa e inteligente, mas peca por alguns erros de quem misturou conceitos avançados (RTOS, dual-core, matrizes de consenso) com funções amadoras do Arduino.

Aqui tens o raio-x da situação:

### O que está excelente (Dá-lhe os parabéns)
* **Arquitetura Dual-Core:** Usar o `loop()` (Core 0) para gerir as comunicações CAN e o interface Serial, enquanto o `loop1()` (Core 1) fica exclusivamente a correr a matemática pesada do PID a 100Hz é uma decisão de arquitetura perfeita para o Raspberry Pi Pico.
* **Proteção de Memória:** O uso de `mutex_t` para proteger as variáveis que saltam entre o Core 0 e o Core 1 é mandatório e ele fê-lo bem. Sem isto, iam ter corrupção de dados aleatória.

---

### As Bombas-Relógio (Onde têm de mexer já)

**1. O bloqueio fatal do `Serial.readStringUntil('\n')`**
Isto é um crime num sistema destes. Essa função do Arduino é bloqueante. Se o interface enviar um comando e faltar o `\n` (newline), o Core 0 vai ficar parado à espera até atingir o *timeout* (que por defeito é 1 segundo!). Durante esse segundo inteiro, a vossa placa deixa de ler o CAN bus, de responder aos vizinhos e de processar calibrações.
* **A solução:** Ler o buffer do Serial char a char (com `Serial.read()`) para um array de forma não-bloqueante.

**2. A matemática da Matriz está invertida (Erro de Física)**
No `loop1()`, quando ele tenta estimar a iluminância local (`y_est`), ele faz isto:
`y_est += snap_gain[my_addr - 1][j] * snap_net_u[j];`
* **O Problema:** A matriz `gain_matrix[i][j]` representa a luz que o LED `i` atira para o sensor `j`. O código dele está a multiplicar o efeito da vossa luz no vizinho pelo PWM do vizinho! Fisicamente, isto não faz sentido nenhum.
* **A solução:** Se querem calcular a luz dos vizinhos a bater no vosso sensor (`my_addr - 1`), têm de inverter os índices: `snap_gain[j][my_addr - 1] * snap_net_u[j]`. (Nota que ele fez isto bem na linha 533 do Feedforward, mas escorregou no `y_est` na linha 553).

**3. Salada russa de `volatile` com `mutex`**
Ele declarou uma data de variáveis como `volatile` (ex: `volatile float system_gain`). Em C++, o `volatile` **não garante thread-safety** num sistema multi-core, apenas impede o compilador de otimizar a variável. Como ele já está (e bem) a usar `mutex_enter_blocking()` antes de mexer nessas variáveis, o `volatile` é completamente inútil e só torna os acessos à memória desnecessariamente mais lentos.

**4. Saturação extrema do Mutex**
Apesar de usar o mutex ser a atitude correta, ele está a trancar e a destrancar a porta vezes demais. Na função `updateMetrics` (chamada a 100Hz), ele entra no mutex só para somar variáveis. Isto pode causar *lock contention*, onde o Core 1 fica à espera que o Core 0 acabe de processar comandos no Serial, rebentando com a estabilidade de tempo real do PID (os vossos 10ms passam a durar mais e o PID começa a oscilar).

A estrutura está lá, mas precisa de uma valente limpeza para passar de um código de laboratório para um código robusto. Queres que te mostre como reescrever a leitura do Serial para não bloquear o sistema inteiro?
