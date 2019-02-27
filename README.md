# Afinador de guitarra

Afinador digital de guitarra desenvolvido em C, implementado em um 
kit de desenvolvimento [STM32f4](https://media.rs-online.com/t_large/F9107951-01.jpg)

Utilizando técnicas de DSP, é realizada a FFT do sinal em tempo real,
e pelo buffer da FFT, busca-se as determidadas frequêcias das notas.


São utilizados 3 leds do kit para sinalizar:

- Nota abaixo: led da esquerda
- Nota correta: led central
- Nota acima: led da direita


