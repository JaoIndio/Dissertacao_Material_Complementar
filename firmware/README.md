# Código Fonte do MCU

Este é o firmware desenvolvido. Contem todo o código fonte implmentado para manipular o sensor AS7341, motor de passo, ADC, DMA, I2C e FreeRTOS.

O arquivo `hello.c` contem a lógica do sistema. Inicializa os periféricos necessários, cria as tarefas do FreeRTOS, inicializa o escalonador e gerencia a lógica geral da captura e envio de dados amostrados.

Dentro do diretório `/SpecLibs` encontra-se todos os drivers e subsistemas desenvolvidos.

-**ADC_DMA_BurstMode:** Configuração do ADC do microcontrolador, incluindo suas interrupções, parametrizações e suporte para DMA.
-**AS7341:** Configuração e manipulação do sensor AS7341.
-**DRV8825:** Configuração e manipulação do motor de passo.
-**LinearMov:** Struct básica para auxiliar na manipução do motor de passo.
-**SpecResult:** Configuração e manipulação do UART. Integrasse via DMA com o driver `/ADC_DMA_BurstMode`.
