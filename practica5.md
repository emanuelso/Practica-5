César Roldan Moros
*Grup 13*
# PRÀCTICA 5: Buses de comunicaciones I
____ 
##### Objectivo
El objetivo de esta práctica es entender el funcionamiento de los buses de sistemas de comunicación entre periféricos internos o externos.
### Ejercicio 1. ESCÁNER I2C

##### Codigo
```
#include <Arduino.h>
#include <Wire.h>
void setup()
{
  Wire.begin();
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("\nI2C Scanner");
}
void loop()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {
    Serial.print("I2C device found at address 0x");
    if (address<16)
      Serial.print("0");
    Serial.print(address,HEX);
    Serial.println(" !");
    nDevices++;
  }
  else if (error==4)
  {
    Serial.print("Unknown error at address 0x");
    if (address<16)
      Serial.print("0");
    Serial.println(address,HEX);
  }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  delay(5000); // wait 5 seconds for next scan
}
```
##### Funcionamiento
Para empezar, añadimos la librería Arduino.h y la Wire.h que nos sirve para comunicar la ESP32 con los dispositivos que trabajan con el protocolo I2C.
Primero inicializamos el IC2 y establecemos la velocidad de bits por la transmisión.
En el loop definimos un byte de error y otro de dirección y un número entero de dispositivos que se inicializa en 0.
Imprimimos el mensaje de que está escaneando
Hacemos un foro donde inicializamos la dirección 1 y decidimos que mientras la dirección sea menor a 127 a la dirección se le suma una. El IC2 scanner utiliza el valor de retorno de Write.endTransmission para ver si un dispositivo ha reconocido la dirección. Asignamos a la variable error cuando se pare la transmisión. Si el error es 0 imprimimos por pantalla que el dispositivo IC2 ha encontrado la dirección Ox y si entonces la dirección es menor que 16, que imprima una dirección en formato hexadecimal y se le suma 1 a la variable del número de dispositivos.
Si el número de errores es igual a 4, le decimos que hay un error.
Por último, decimos que si el número de dispositivos es 0 que imprima que no ha encontrado ningún dispositivo y se acaba el programa.
Hay un delay de 5 segundos entre escáner y escáner.


### Ejercicio 2 MPU6050-Sensor Acelerómetro i Giroscopio

##### Codigo
```
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;
void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}
void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g;
  mpu.getEvent(&a, &g);
  /* Print out the values */
  Serial.print("Acceleration X:");
  Serial.print(a.acceleration.x);
  Serial.print(", Y:");
  Serial.print(a.acceleration.y);
  Serial.print(", Z:");
  Serial.print(a.acceleration.z);
  Serial.print(" m/s^2");
  Serial.print("Rotation X:");
  Serial.print(g.gyro.x);
  Serial.print(", Y:");
  Serial.print(g.gyro.y);
  Serial.print(", Z:");
  Serial.print(g.gyro.z);
  Serial.println("");
  delay(10);
}
```
##### Funcionamiento
Primero añadimos las bibliotecas necesarias para el sensor MPU-6050 y creamos un objeto de tipo Adafruit_MPU6050 que le llamamos mpu.
En el void setup() inicializamos el monitor serie a una velocidad de 115200 bauds y también inicializamos el sensor.
Configuramos el rango de medición del acelerómetro y del giroscopio.
Configuramos el ancho de banda del filtro
En el void loop() obtendremos las lecturas de los sensores y las mostraremos por pantalla. Primero obtenemos los nuevos eventos del señor con las lecturas actuales y finalmente imprimimos las lecturas del acelerómetro y el giroscopio en los tres ejes (x,y,z).
Hacemos que las lecturas del sensor se muestren cada 500 milisegundos.
