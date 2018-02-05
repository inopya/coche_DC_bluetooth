/*
 #       _\|/_   A ver..., ¿que tenemos por aqui?
 #       (O-O)
 # ---oOO-(_)-OOo---------------------------------

 ####################################################
 # ************************************************ #
 # *  ------------------------------------------  * #
 # *                                              * #
 # *         Autor: Eulogio Lopez Cayuela         * #
 # *      Versión 0.2 BT   Fecha: 19/12/2017      * #
 # *   *****  COCHE ROBOTICO MINI con BT  *****   * #
 # ************************************************ #
 ####################################################
 */

/*
 * NOTAS DE LA VERSION:

x.x)- para un futuro, se puede mejorar la deteccion de movomiento poniendo mas sensores
      y/o cambiando estos por PIR
  
0.2)- Modo Auto modificado:
      Hasta ahora huía si te acercabas mucho o si habia peligro de choque con un obstaculo durante el avance
      Vamos a tratar de hacer que se mueva hacia cosas que esten a una cierta distancia, solo si estas se mueven.
      Igualmente habrá de salir huyendo si se acerca mucho a ellas para no chocar.
      El modo manual se mantiene igual, solo comandos blueTooth.
0.1)- Huye si te acercas mucho o si ve peligro de choque con un obstaculo durante el avance.
0.0)- Implementado medidor de estado de bateria mediate lecturas de esta en la patilla analogica A0.
      RECORDAR que usamos para este coche una bateria reciclada de una tablet (a 3.7v) 
      y de la que obtenemos 5v para los motores mediante un DC-DC booster
      Por tanto podemos aplicar el positivo de la bateria (que no excederá de 4 con algo) 
      a la entrada analogica A0 para tener controlado su potencial.

 */

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/
//------------------------------------------------------
//algunas definiciones personales para mi comodidad al escribir codigo
//------------------------------------------------------
#define AND &&
#define OR ||
#define NOT !
#define ANDbit &
#define ORbit |
#define XORbit ^
#define NOTbit ~



//------------------------------------------------------
//Importacion de las biblioteca necesarias
//------------------------------------------------------

#include <SoftwareSerial.h>  //Librería que permite comunicación serie en pines distintos de RX y TX.

/*Conectaremos los pins RXD,TDX del módulo Bluetooth a los pines 3 y 4 */
SoftwareSerial BT(3,4);      // [3 RX arduino ---> TX modulo BT] y [4 arduino---> RX modulo BT]
                             // La patilla RX del modulo conectada mediante un divisor de tension para que este a 3'3V 

// 1=1200 , 2=2400, 3=4800, 4=9600, 5=19200, 6=38400, 7=57600, 8=115200



//------------------------------------------------------
// PINES ULTRASONIDO
//------------------------------------------------------

#define PIN_LED_OnBOARD     13  //como vamos justos de pines se usa para el ultrasonido

// --- Ultrasonidos ----
//debido a que vamos justisimos de pines en el arduino UNO, usamos el sensor de ultrasonidos
//compartiendo un unico pin para lanzar la señal de eco y para recibirla
#define PIN_RADAR_1_TRIG    13
#define PIN_RADAR_1_ECO     13


#define MAX_RANGE  1500         // Limitar maxima distancia en mm que deseo medir (20-4000)
#define TIMEOUT 24000           // TimeOut de 24000 µs (el tiempo en recorrer 4m ida y vuelta)
                                // porque si no, pulseIn espera hasta 1s antes de contestar 


//------------------------------------------------------
//  PINES RUEDAS
//------------------------------------------------------

#define PIN_motorDerechoAvance        6
#define PIN_motorDerechoRetroceso     7
#define PIN_DerechoVelocidad_PWM     11
    
#define PIN_motorIzquierdoAvance      8
#define PIN_motorIzquierdoRetroceso   9
#define PIN_IzquierdoVelocidad_PWM    5

//valores empiricos por si hay desfase entre las velocidades de ambos motores
#define VELOCIDAD_OFFSET_IZQ 0  //
#define VELOCIDAD_OFFSET_DER 0  //

//------------------------------------------------------
//  PINES LED RGB
//------------------------------------------------------
//uso de un led RGB para indicar el estado de la bateria y el modo de funcionamoiento
#define PIN_led_rojo       2
#define PIN_led_verde     10
#define PIN_led_azul      12


//------------------------------------------------------
// VARIABLES GLOBALES
//------------------------------------------------------

boolean FLAG_comando_recibido = false;  //veremos...
boolean FLAG_mode_manual = true;  //en true, se corta la alimentacion principal del motor


boolean FLAG_avance = false;
boolean FLAG_retroceso = false;
boolean FLAG_derecha = false;
boolean FLAG_izquierda = false;

boolean FLAG_RUNNING = false;  //empleada en algunas funciones pero sin efecto real por ahora

//estas variables han de modificarse con los valores obtenidos en la calibracion
int ERROR_GIRO_DER = 82;          //valor pedido en consola que genera giro real de 90º DERECHA   (-)
int ERROR_GIRO_IZQ = 85;          //valor pedido en consola que genera giro real de 90º IZQUIERDA (+)
int ERROR_MOVER_AVANCE = 30;      //valor pedido en consola que genera un avance 30 cm
int ERROR_MOVER_TRETROCESO = 30;  //valor pedido en consola que genera un retroceso 30 cm

#define BATERIA_1  A0

#define INTERVALO_TEST_BATERIA   120000   //comprobar el estado de la bateria cada dos minutos (120000ms) 
unsigned long momento_comprobar_bateria = millis() + INTERVALO_TEST_BATERIA;




//***************************************************************************************************
//  SECCION OPERATIVA DEL PROGRAMA
//***************************************************************************************************


//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  FUNCION DE CONFIGURACION
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void setup() //inicializacion del sistema
{
  Serial.begin(19200);      //inicializar el puerto serial para 'DEBUG' (y calibraciones)
  //Serial.println("file >>  coche_DC_mini_v0.2_modoAuto.ino");  // --->  para DEBUG

  BT.begin(19200);        //inicializar el modulo BT

  //------------------------------------------------
  
  pinMode(PIN_LED_OnBOARD, OUTPUT);
  digitalWrite(PIN_LED_OnBOARD, LOW); //apagar el led indicador de giro del motor

  //------------------------------------------------

  pinMode(PIN_led_rojo, OUTPUT);
  pinMode(PIN_led_verde, OUTPUT);
  pinMode(PIN_led_azul, OUTPUT);
  
  //------------------------------------------------
  
  pinMode(PIN_motorDerechoAvance, OUTPUT);
  pinMode(PIN_motorDerechoRetroceso, OUTPUT);
  pinMode(PIN_DerechoVelocidad_PWM, OUTPUT);
    
  pinMode(PIN_motorIzquierdoAvance, OUTPUT);
  pinMode(PIN_motorIzquierdoRetroceso, OUTPUT);
  pinMode(PIN_IzquierdoVelocidad_PWM, OUTPUT); 
   
  digitalWrite(PIN_motorDerechoAvance, LOW);
  digitalWrite(PIN_motorDerechoRetroceso, LOW);
  analogWrite(PIN_DerechoVelocidad_PWM, 0);
    
  digitalWrite(PIN_motorIzquierdoAvance, LOW);
  digitalWrite(PIN_motorIzquierdoRetroceso, LOW);
  analogWrite(PIN_IzquierdoVelocidad_PWM, 0);
  
  //------------------------------------------------

  Serial.print("BATERIA: ");Serial.println(medir_bateria());
  //medir_bateria();
}




//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  BUCLE PRINCIPAL DEL PROGRAMA   (SISTEMA VEGETATIVO)
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void loop()
{
//  while(true){
//    calibracion_movimiento_cm();
//  } 
 

  /* leer datos procedentes del BT */
  leerPuertoBT();

  /* ---- INICIO MODO AUTO ---- */
  //comprobar la distancia a objetos
  int distancia_choque = activarMurcielago(PIN_RADAR_1_TRIG, PIN_RADAR_1_ECO);  //si hay error de medida, devuelve -100
  //Serial.println(distancia_choque);
  //delay(500);

  /* comprobar si algo se ha movido frente a mi */
  if (comprobar_movimiento_objetos() > 80){
    distancia_choque = activarMurcielago(PIN_RADAR_1_TRIG, PIN_RADAR_1_ECO);
    mover(255, 0);
    while (distancia_choque > 250){
      /* Prevenir posibles colisiones */
      distancia_choque = activarMurcielago(PIN_RADAR_1_TRIG, PIN_RADAR_1_ECO);
    }
    parar();
    delay(500);
    movimiento_evasion(); //retroceso para no chocar
    girar_grados(30);
    parar();
    delay(1000); //pausa para estabilizarnos
  }

  /* revisar el estado de la bateria */
  if(millis() > momento_comprobar_bateria){
    medir_bateria();
    momento_comprobar_bateria = millis() + INTERVALO_TEST_BATERIA;
  }
  /* ---- FIN MODO AUTO ---- */ 

  
  /* ---- INICIO MODO MANUAL ---- */
  if(FLAG_mode_manual == true){
    parar(); //bloquear el coche hasta nueva orden
  }
  
  unsigned long momento_parada = millis();
  boolean FLAG_cambiar_led = true;
    
  while (FLAG_mode_manual == true){
    distancia_choque = activarMurcielago(PIN_RADAR_1_TRIG, PIN_RADAR_1_ECO);
    //si hay error de medida, devuelve -100
    /* Prevenir posibles colisiones */
    if(distancia_choque >0 AND distancia_choque <= 250){
      movimiento_evasion(); //retroceso para no chocar
      parar();
      delay(1000);
    }

    leerPuertoBT();
    /* realizar ordenes procedentes del BT */
    aplicar_comando_BT();
    
    /* parpadeo del led de la bateria (indica modo manual)*/
    if ((millis() > momento_parada + 1000)){
      momento_parada += 1000;
      FLAG_cambiar_led = NOT FLAG_cambiar_led;
      if (FLAG_cambiar_led == true){
        medir_bateria();
      }
      else{
        color_led(0,0,0);
      }
    }
  }
  /* ---- FIN MODO MANUAL ---- */ 
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        MOVIMIENTO (FUNCIONES MAQUINA)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  PARAR COMPLETAMENTE CUALQUIER MOVIMIENTO
//========================================================

void parar()
{ 
  /* 
   Bloquea los giros en cualquier sentido y deshabilita las señales PWM  para ambos motores
   */
  digitalWrite(PIN_motorIzquierdoAvance, LOW);
  digitalWrite(PIN_motorIzquierdoRetroceso, LOW);  
  digitalWrite(PIN_motorDerechoAvance, LOW);
  digitalWrite(PIN_motorDerechoRetroceso, LOW);
  digitalWrite(PIN_IzquierdoVelocidad_PWM, LOW);  // pwm motor izquierdo --> Disabled
  digitalWrite(PIN_DerechoVelocidad_PWM, LOW);    // pwm motor derecho   --> Disabled

  FLAG_RUNNING = false;
}


//========================================================
//  ESTABLECER SENTIDOS DE GIRO PARA LOS MOTORES
//========================================================

void setWay(byte izquierda, byte derecha)
{
  /*
  Establecemos los sentidos de giro de cada rueda
  basandonos en la siguiente 'tabla de verdad':
  (Pero no se realiza ningun movimiento)

  Serial.print("4 % 2  "); Serial.println(4%2);  // 0
  Serial.print("4 % 3  "); Serial.println(4%3);  // 1
  Serial.print("9 % 2  "); Serial.println(9%2);  // 1
  Serial.print("9 % 3  "); Serial.println(9%3);  // 0
  */
    
  digitalWrite(PIN_motorIzquierdoAvance, izquierda%2);
  digitalWrite(PIN_motorIzquierdoRetroceso, izquierda%3);  
  digitalWrite(PIN_motorDerechoAvance, derecha%2);
  digitalWrite(PIN_motorDerechoRetroceso, derecha%3);
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        MOVIMIENTO (FUNCIONES HUMANAS)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  MOVER DURANTE UN PERIODO DE TIEMPO (mover funcion completa)
//========================================================

void mover(int velocidad, int tiempo)
{
  /*
   * recibe dos parametros: velocidad y tiempo
    - Si la velocidad es Posiviva-->Avanza, si es Negativa-->Retrocede
    - Si el tiempo es mayor que CERO, se aplica una pausa igual a ese tiempo y despues se PARA
      Si el tiempo es negatico o CERO, no se aplica y el movimiento continua en ese sentido 
      hasta una orden de paro u otro cambio de direccion o sentido.
   */

  FLAG_RUNNING = true;
  
  if (velocidad > 0){
    setWay(9, 9);
  }

  if (velocidad < 0){
    setWay(4, 4);
  }

  velocidad = abs(velocidad);  //apartir de este punto nos interesa solo el modulo. El sentido ya se ha usado

  //comprovaciones para que la velocidad no pase de 255 en niongun caso
  int velocidad_corregida_IZQ = VELOCIDAD_OFFSET_IZQ + velocidad;
  if (velocidad_corregida_IZQ >255){
    velocidad_corregida_IZQ = 255;
  }

  int velocidad_corregida_DER = VELOCIDAD_OFFSET_DER + velocidad;
  if (velocidad_corregida_DER >255){
    velocidad_corregida_DER = 255;
  } 
   
  analogWrite(PIN_IzquierdoVelocidad_PWM, velocidad_corregida_IZQ);  //motor izquierdo   
  analogWrite(PIN_DerechoVelocidad_PWM, velocidad_corregida_DER);    //motor derecho

  if (tiempo > 0){
    delay (tiempo); 
    parar();
  }
}


//========================================================
//  MOVER un determinado numero de CENTIMETROS
//========================================================

void mover_cm(int centimetros)
{
  /*
   * recibe un unico parametro: los centimetros que se convierten en tiempo que dura ese desplazamiento.
    - el signo de la magnitud recibida indica el sentido del movimiento.
    - si (+) avanzar, si (-) retroceder
   */


  FLAG_RUNNING = true;
  
  if (centimetros > 0){
    setWay(9, 9);
    centimetros = (centimetros * ERROR_MOVER_AVANCE)/30; 
  }

  if (centimetros < 0){
    setWay(4, 4);
    centimetros = (centimetros * ERROR_MOVER_TRETROCESO)/30; 
  }

  centimetros = abs(centimetros);  //apartir de este punto nos interesa solo el modulo. El sentido ya se ha usado 
   
  analogWrite(PIN_IzquierdoVelocidad_PWM, 255);  //motor izquierdo   
  analogWrite(PIN_DerechoVelocidad_PWM, 255);    //motor derecho

  delay (centimetros * 100); 
  parar();
}


//========================================================
//  GIRAR un determinado numero de GRADOS
//========================================================

void girar_grados(int grados)
{
  /*
   * recibe un unico parametro: los grados de giro que se convierten en tiempo que dura ese giro.
    - el signo de la magnitud recibida indica el sentido de giro.
    - si (+) giro a izquirda, si (-) giro a la derecha
   */
  FLAG_RUNNING = true;

  //----------------------------
  
  if (grados <0){ //derecha
    setWay(4, 9);
    grados = (grados * ERROR_GIRO_DER)/90;  //ERROR_GIRO_DER = 82
  }
  if (grados >0){ //izquierda
    setWay(9, 4);
    grados = (grados * ERROR_GIRO_IZQ)/90;  //ERROR_GIRO_IZQ = 85
  } 
  
  //----------------------------
  analogWrite(PIN_IzquierdoVelocidad_PWM, 255);  //motor izquierdo   
  analogWrite(PIN_DerechoVelocidad_PWM, 255);    //motor derecho
  //---------------------------- 

  //2200ms es lo que tarda en girar 90º (valor previamente calculado de  forma empirica)
  float tiempo = (220.0*abs(grados))/9.0;   
  delay(int(tiempo));
  parar();  
}


//========================================================
//  GIRAR siempre, BT
//========================================================

void girar_BT(int grados)
{
  /* --> reutilizacion de "girar_grados(int grados)" <--
   * recibe un unico parametro: un entero del que solo se aprovecha el signo
    - el signo de la magnitud recibida indica el sentido de giro.
    - si (+) giro a izquirda, si (-) giro a la derecha
   */
   
  FLAG_RUNNING = true;

  //----------------------------
  
  if (grados <0){ //derecha
    setWay(4, 9);
  }
  if (grados >0){ //izquierda
    setWay(9, 4);
  } 
  
  //----------------------------
  analogWrite(PIN_IzquierdoVelocidad_PWM, 200);  //motor izquierdo   
  analogWrite(PIN_DerechoVelocidad_PWM, 200);    //motor derecho
  //----------------------------   
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   BATERIA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// MONITORIZAR EL ESTADO DE LA BATERIA
//========================================================

float medir_bateria()
{
  float voltios = (analogRead(BATERIA_1)*5.0*4.1)/(1024.0*4.16);    // read the input pin A0
  //Serial.print("BATERIA: ");Serial.println(voltios);
  
  digitalWrite(PIN_led_rojo, LOW);   //mostar led rojo
  digitalWrite(PIN_led_verde, LOW);   //mostar led verde
  digitalWrite(PIN_led_azul, LOW);   //mostar led azul

    
  if (voltios >= 4.0){  //--------------------------> nivel perfecto
    digitalWrite(PIN_led_azul, HIGH);   //mostar led azul
    //Serial.println("AZUL ");
  }
  
  else if (voltios >=3.8 AND voltios < 4.0){  //--> nivel bueno
    digitalWrite(PIN_led_verde, HIGH);   //mostar led verde
    //Serial.println("VERDE ");
  }
  else{  //----------------------------------------> nivel critico
    digitalWrite(PIN_led_rojo, HIGH);   //mostar led rojo
    //Serial.println("ROJO ");
  }
   
  return voltios;     // por si queremos ahcer algo con el dato dentro del programa
}


//========================================================
// MOSTAR COLOR EN LED RGB
//========================================================

void color_led(byte rojo, byte verde , byte azul)
{
  analogWrite(PIN_led_rojo, rojo);      //cantidad de rojo
  analogWrite(PIN_led_verde, verde);    //cantidad de verde
  analogWrite(PIN_led_azul, azul);      //cantidad de azul
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        ULTRASONIDOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  ACCESO AL SENSOR ULTRASONICO PARA MEDIR LA DISTANCIA
//========================================================

int activarMurcielago(int userTrigPin, int userEchoPin) 
{
  /* Llamada a un de sensor ultrasonico:
   * Version adaptada para usar UN LOLO PIN de la clasica FUNCION MURCIELAGO
   * que devuelve la DISTANCIA medida EN MILIMETROS por un sensor de ultrasonidos.
   * Realiza la MEDIANA de 5 mediciones para mejorar la precision.
   */

  int  duracion, temp, i, j;
  float distanciaMedida;

  int listaDeMediciones[] = {
    0, 0, 0, 0, 0, 0
  };
  // bucle para realizar la mediana de 5 medidas
  int errores = 0;
  for (i = 0; i < 5; i++) {
    if (errores > 3) {
      //admito hasta 3 errores.
      //Si una medida produjo más errores en su obtencion, la descarto
      //devolviendo una medida 'imposible' para que sea despreciada durante su analisis
      return -100;
    }

    //Generador de pulsos para ecolocalizacion
    pinMode(userTrigPin, OUTPUT);
    digitalWrite(userTrigPin, LOW);
    delayMicroseconds(2); //ponemos a cero la salida por precaucion antes de emitir el pulso
    digitalWrite(userTrigPin, HIGH); //iniciar pulso necesario para el proceso de medicion
    delayMicroseconds(10); //pulso para iniciar medicion
    digitalWrite(userTrigPin, LOW); //terminar pulso de medicion

    //Leer el tiempo que tarda en recibirse el eco
    //TimeOut de 24000 µs, el tiempo en recorrer 4m ida y vuelta
    //porque si no, pulseIn espera hasta 1s antes de contestar
    pinMode(userEchoPin, INPUT);
    duracion = pulseIn(userEchoPin, HIGH, TIMEOUT);

    listaDeMediciones[i] = duracion;
    if (duracion <= 0) {
      //a veces el sensor ante un error devuelve una medida negativa (imposible)
      errores += 1; // en ese caso incremento el contador de errores
      i--;//si hay error de medicion, se anula y se repite la medida
    }
  }

  //Calcular la MEDIANA de las 5 mediciones, para ello ordenamos las mediciones
  //de menor a mayor y nos quedamos con la del centro de la lista (la tercera).
  for (i = 0; i < 5; i++) {
    for (j = 0; j < (4 - i ); j++) {
      if (listaDeMediciones[j] > listaDeMediciones[j + 1]) {
        temp = listaDeMediciones[j];
        listaDeMediciones[j] = listaDeMediciones[j + 1];
        listaDeMediciones[j + 1] = temp;
      }
    }
  }
  duracion = listaDeMediciones[2]; // cogemos el 3º valor de la lista, es decir la mediana de 5 lecturas
  // calcula la distancia en milimetros. El programa trabaja en cm, pero hago la conversion despues
  //y reutilizo esta funcion tal cual la diseñé cuando aprendí a manejar el ultrasonido.
  distanciaMedida = duracion / 5.9;
  return distanciaMedida;   //recordar que esto son mmilimetros
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL REMOTO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// FUNCION PARA ACCESO Y LECTURA 'rapida' DEL PUERTO BLUETOOTH
//========================================================

void leerPuertoBT()
{
  /* 
   *  Funcion para leer comando recibidos por el puerto (BT)
   */

  FLAG_comando_recibido = false;
  
  if(BT.available()) {
    char dato=BT.read();         //Leemos y asignamos los datos carácter a carácter en la variable "dato"
    //Serial.print("dato << ");  // --->  para DEBUG
    //Serial.println(dato);      // --->  para DEBUG 

    FLAG_comando_recibido = true;
    
    FLAG_avance = false;
    FLAG_retroceso = false;
    FLAG_derecha = false;
    FLAG_izquierda = false;

    if(dato == 'a') {
      FLAG_mode_manual = false;
      medir_bateria();
    }
    
    if(dato == 'b') {
      parar();
      FLAG_mode_manual = true;
      
    } 
    
    if(dato == '0') {
      parar();
    }
    
    if(dato == '1') {  //avanzar
      FLAG_avance = true;
    }
    
    if(dato == '2') {  //retroceso
      FLAG_retroceso = true;
    }

    if(dato == '3') {  //giro izquierda
      FLAG_izquierda = true;
    }
    
    if(dato == '4') {  //giro derecha
      FLAG_derecha = true;
    }
  }
}


//========================================================
//   TOMA DE DECISIONES SEGUN COMANDOS BLUETOOTH
//========================================================

void aplicar_comando_BT()
{
  /*
   * Si hay un comando recibido por BT, aplicarlo
   */
   
  if(FLAG_avance == true) {     //avance
    mover(255,0);
  }
  
  if(FLAG_retroceso == true) {  //retroceso
    mover(-255,0);
  }
  
  if( FLAG_izquierda== true) {  //giro izquierda
    girar_BT(+1);
  }
  
  if(FLAG_derecha == true) {  //giro derecha
    girar_BT(-1);
  }
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    OPERACIONES EN MODO AUTOMATICO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// COMPROBAR SI UNA PERSONA U OBJETO SE MUEVE FRENTE A EL
//========================================================

int comprobar_movimiento_objetos()
{
  /*
   * Se controlara el movimiento de objetos frente al vehiculo (modo Automatico)
   * leyendo dos veces con un pequeño intervalo de tiempo el sensor de ultrasonido
   * y comprobando si hay alguna diferencia.
  */
  
  int distancia_d0 = 0;
  int distancia_d1 = 0;
  int diferencia = 0;
  
  distancia_d0 = activarMurcielago(PIN_RADAR_1_TRIG, PIN_RADAR_1_ECO);  //si hay error de medida, devuelve -100
  if(distancia_d0 < 0 OR distancia_d0 > MAX_RANGE){
    distancia_d0 = MAX_RANGE;
  }
  delay(200);
  distancia_d1 = activarMurcielago(PIN_RADAR_1_TRIG, PIN_RADAR_1_ECO);  //si hay error de medida, devuelve -100
  if(distancia_d1 < 0 OR distancia_d1 > MAX_RANGE){
    distancia_d1 = MAX_RANGE;
  }
  
  diferencia = abs(distancia_d0 - distancia_d1);
  
  return diferencia;   //si es mayor que CERO es que algo se ha movido
}


//========================================================
// MOVIMIENTO DE EVASION PARA NO CHOCAR
//========================================================

void movimiento_evasion()
{
  parar();
  mover_cm(-10);
  delay(600);
  FLAG_avance = false;
}




/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    ZONA PARA CALIBRACION
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  CALIBRAR DESPLAZAMIENTOS LINEALES en cm
//========================================================

void calibracion_movimiento_cm()
{
  /*     mover(int cm)    */

  ERROR_MOVER_AVANCE = 30;    //reset del error desplazamiento     
  ERROR_MOVER_TRETROCESO = 30;
  
  if (Serial.available() > 0){   
    int centimetros = Serial.parseInt();
    int mierda = Serial.parseInt();  //para evitar errores por culpa de la pulsacion de intro
    
    Serial.print("Orden>>");
    Serial.print(" centimetros:  ");Serial.println(centimetros);
    
    mover_cm(centimetros);
    parar();

  }  
}



//========================================================
//  CALIBRAR GIROS EN GRADOS
//========================================================

void calibracion_giro_grados()
{
  /*
  
  girar(int grados)

  Lectura del puerto serie (para recibir ordenes de giro).

   todos los valores deben ser numeros enteros 
   ya que solo podemos recibir enteros atraves de la funcion parseInt().
   */

  ERROR_GIRO_DER = 90;  //reset del error de giro
  ERROR_GIRO_IZQ = 90;
  
  if (Serial.available() > 0){

    int grados = Serial.parseInt();
    int mierda = Serial.parseInt();    //para evitar errores por culpa de la pulsacion de intro
    
    Serial.print("Orden>>");
    Serial.print("  grados: "); Serial.println(grados);

    
    girar_grados(grados);
    parar();  //redundante
  }  
}



//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************

