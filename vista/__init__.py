"""Vista — a lógica que roda no carro.

Só usa o que o carro tem: a imagem da câmera, a telemetria do ESP32 (encoders,
MPU6050, odometria), o mapa oficial e a calibração da câmera. Nada de verdade
de campo — essa fica no simulador, para medir o Vista.

* ``mapa.py``       — em que rua o carro está e onde nela (grafo + odometria);
* ``cruzamento.py`` — atravessa o cruzamento pela curva do mapa, com odometria;
* ``placas.py``     — detector no laço: qual placa é da MINHA aproximação, semáforo;
* ``piloto.py``     — a máquina de estados que junta tudo e manda rpm de cada roda.

A decisão por placa é ``vista_sim/rules.py`` e o controle na faixa é
``vista_sim/controle.py`` — os mesmos módulos que o simulador usa.
"""
