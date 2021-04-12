# ESP_MODBUS_GSAP_30min
Código para execução do controle da planta do Lamp(Laboratório da UFRN) e aquisição dos parâmetros da rede via GSAP e parâmetros de controle via MODBUS usando o ESP32.
Implementação do controle usando o ESP32, aquisição dos dados de controle via MODBUS ao gateway da YOKOGAWA, aquisição dos parâmetros da rede via GSAP, e comunicação com um sistema de aquisição implemetado no Node-Red via MODBUS.
A biblioteca GSAP usada pode ser encontrada em outro repositório nesse própio perfil, sendo esse um dos códigos implementados para o teste da biblioteca que como descrito no outro repositório, há um problema na questão do tempo de execução do código no ESP sem travamento, sendo o tempo de execução dessa implementação do controle de 30 minutos. 
