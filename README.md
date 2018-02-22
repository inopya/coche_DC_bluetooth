# Coche DC bluetooth
Pequeña plataforma base para vehiculos de traccion en un solo eje y rueda loca.
Diseño básico de traccion con motores de corriente continua (con reductora incorporada) pero facilmente adaptable a tracción con servomotores, lo que le daria algo mas de precisión en los moviminetos y liberaria algunos pines de arduino que ya no tendria que controlar el puente H con lo que permitiria incorporar algun sensor más.
Actualmente dispone de un solo sensor ultrasonico con el que interactuar con el medio.

Dos modos de funcionamiento: Manual y Automatico.
* En modo manual se controla mediante la aplicacion android que se encuentra en la capeta correspondiente. 
El led indicador de nivel de bateria parpadea con una cadencia de 2 segundos. Evita los choques frontales desatendiendo momentaneamente las ordenes de avance y realizando un movimiento de evasion hacia atras cuando se haya muy proximo a un objeto.
* En modo Automatico el led indicador de nivel se mantiene encendido de forma permanente. 
Se mueve hacia objetos o personas que se muevan frente a el y se para un poco antes de llegar, o se retira si esos objetos o personas estan muy proximos.

La aplicacion android es muy básica (es la primera vez que tomo contacto con android) y no permite la pulsación de dos teclas a la vez. En la carpeta android se encuentra tambien el fichero fuente (aia) de app_inventor para que se puedan realizar los cambios que se desee.
