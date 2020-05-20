#include <MPU6050_tockn.h> // libreria del MPU6050
#include <Wire.h>
MPU6050 mpu6050(Wire);
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
//Nota de prueba

#define in1 6//   PUENTE H DE LA LLANTA DERECHA PWM DELANTE(LPWM)     
#define in2 9 //  PUENTE H DE LA LLANTA IZQUIERDA PWM REVERSA(RPWM)         
#define in3 10 // PUENTE H DE LA LLANTA IZQUIERDA PWM DELANTE(LPWM)             
#define in4 5 // PUENTE H DE LA LLANTA DERECHA PWM REVERSA(RPWM)                 

int motorSpeedA = 0;
int motorSpeedB = 0;
int vellineal = 0;
int velangular = 0;


int btnvelbajo = 46;
int btnvelalto = 47;
int btnregresarActuador = 31;
int btnsubirActuador = 30;
int pinRelayA = 40;
int pinRelayB = 41 ;
int angulo;
int var = 2;
int varbip;
int varmov;
int varmax;     //Velocidad máxima de la silla de ruedas
bool engaged = false; //Bandera para indicar que la plataforma está en movimiento
bool rising = false;  //Bandera para indicar que el sistema de bipedestación está activo

void setup()
{
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(5, 0);
  lcd.print("UPIIZ  IPN");
  lcd.setCursor(0, 1);
  lcd.print("SILLA AUTOMATICA CON");
  lcd.setCursor(5, 2);
  lcd.print("SISTEMA DE");
  lcd.setCursor(3, 3);
  lcd.print("BIPEDESTACION");
  delay (5000);
  pinMode(btnvelalto, INPUT);
  pinMode(btnvelbajo, INPUT);
  pinMode(btnregresarActuador, INPUT);
  pinMode(btnsubirActuador, INPUT);
  pinMode(pinRelayA, OUTPUT);
  pinMode(pinRelayB, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop()
{
  mpu6050.update();
  angulo = mpu6050.getAngleX();
  int xAxis = analogRead(A0); // Read Joysticks X-axis
  int yAxis = analogRead(A1); // Read Joysticks Y-axis

  //Pantalla inicial
  if (!engaged && !rising)
  {
    estadoEspera();
  }

  //Velocidad máxima para la silla
  if (digitalRead(btnvelalto) == HIGH && angulo < 30)
  {
    velmax = 87; //5km/h equivale al 87.5% de la velocidad máxima de los motores
  }
  if (digitalRead(btnvelbajo) == HIGH && angulo < 30)
  {
    velmax = 52; //3km/h equivale al 52.5% de la velocidad máxima de los motores
  }
  if (angulo >= 30)
  {
    velmax = 17; //1km/h equivale al 17.5% de la velocidad máxima de los motores
  }

  //Movimiento de la silla
  if (!rising) //Si no está en proceso de bipedestación
  {
    if (yAxis > 517) //Adelante
    {
      vellineal = map(yAxis - 511, 0, 511, 0, 255); //velocidad lineal positiva
    }
    if (yAxis < 505) //Atras
    {
      vellineal = map(-yAxis + 511, 0, 511, 0, -255); //velocidad lineal negativa
    }
    if (xAxis > 517) //Giro a la derecha - horario - giro negativo en Z
    {
      velangular = map(xAxis - 511, 0, 511, 0, 255); //velocidad angular positiva
    }
    if (yAxis < 505) ////Giro a la izquierda - antihorario - giro positivo en Z
    {
      velangular = map(-xAxis + 511, 0, 511, 0, -255); //velocidad angular negativa
    }
    motorSpeedA = vellineal - velangular / 2; //velocidad total para llanta izquierda
    motorSpeedB = vellineal + velangular / 2; //velocidad total para llanta derecha
    if (motorSpeedA < 0)
    {
      engaged = true;
      digitalWrite(in3, LOW);
      analogWrite(in2, (-motorSpeedA * velmax) / 100);
    }
    else
    {
      engaged = true;
      digitalWrite(in2, LOW);
      analogWrite(in3, (motorSpeedA * velmax) / 100);
    }
    if (motorSpeedB < 0)
    {
      engaged = true;
      digitalWrite(in1, LOW);
      analogWrite(in4, (-motorSpeedB * velmax) / 100);
    }
    else
    {
      engaged = true;
      digitalWrite(in4, LOW);
      analogWrite(in1, (motorSpeedB * velmax) / 100);
    }
    
    estadoMovimiento(velmax);

    if (xAxis >= 505 && xAxis <= 517 && yAxis >= 505 && yAxis <= 517) //Joystick en el centro, motores con velocidad cero
    {
      engaged = false;
      vellineal = 0;
      velangular = 0;
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
    }
  }

  // Accionamiento del actuador lineal en estado de reposo
  if (!engaged && (digitalRead(btnvelbajo) == LOW && digitalRead(btnvelalto) == LOW)) //Si el joystock está en el centro y los botones de bipedestar sin accionar
  {
    rising = true;
    if (digitalRead(btnsubirActuador) == HIGH || digitalRead(btnregresarActuador) == HIGH)
    {
      var = 1;
    }
    else
    {
      var = 0;
    }
  }
  if ((digitalRead(btnvelbajo) == HIGH || digitalRead(btnvelalto) == HIGH))
  {
    digitalWrite(pinRelayA, HIGH);
    digitalWrite(pinRelayB, HIGH);
    var = 2;
  }

  if (yAxis > 517) //Cambiar la condición
    switch (var)
    {
      case 0:
        lcd.clear();
        estadoEspera();
        estadocero();
        break;
      case 1:
        lcd.clear();
        estadoBipedestacion();
        accionBipedestacion();
        break;
      case 2:
        lcd.clear();
        if (angulo > 30)
        {
          estadoMovimiento2(); // NIVEL BAJO PORQUE EL ANGULO ES MAYOR A 30
          accionMovimiento2();
        }
        else
        {
          estadoMovimiento1();
          accionMovimiento1();
        }
        break;
      default:
        lcd.clear();
        estadoEspera();
        estadocero();
        break;
    }
}// Fin del loop

void accionBipedestacion()
{

  if (digitalRead(btnsubirActuador) == HIGH && digitalRead(btnregresarActuador) == LOW)
  {
    varbip = 1;
  }
  else if (digitalRead(btnregresarActuador) == HIGH && digitalRead(btnsubirActuador) == LOW)
  {
    varbip = 2;
  }
  else if (digitalRead(btnsubirActuador) == LOW && digitalRead(btnsubirActuador) == LOW)
  {
    var = 0;
  }
  else
  {
    varbip = 0;
  }
  switch (varbip)
  {
    case 0:
      digitalWrite(pinRelayA, HIGH);
      digitalWrite(pinRelayB, HIGH);
      break;
    case 1:
      digitalWrite(pinRelayA, LOW);
      break;
    case 2:
      digitalWrite(pinRelayB, LOW);
      break;
    default:
      digitalWrite(pinRelayA, HIGH);
      digitalWrite(pinRelayB, HIGH);
  }
}

void estadoBipedestacion()
{
  if (angulo >= 30) {
    lcd.clear();
    lcd.setCursor(5, 0);
    lcd.print("UPIIZ  IPN");
    lcd.setCursor(3, 1);
    lcd.print("ANGULO  ACTUAL");
    lcd.setCursor(5, 2);
    lcd.print(mpu6050.getAngleX());
    lcd.setCursor(5, 3);
    lcd.print("PRECAUCION");
    delay(250);

  }
  else
  {
    lcd.setCursor(5, 0);
    lcd.print("UPIIZ  IPN");
    lcd.setCursor(3, 1);
    lcd.print("ANGULO  ACTUAL");
    lcd.setCursor(5, 2);
    lcd.print(mpu6050.getAngleX());
    delay(250);
  }
}

void estadoEspera()
{
  lcd.setCursor(5, 0);
  lcd.print("UPIIZ  IPN");
  lcd.setCursor(2, 1);
  lcd.print("VELOCIDAD MAXIMA");
  lcd.setCursor(2, 2);
  lcd.print("1 KM/H    3 KM/H");
  lcd.setCursor(2, 3);
  lcd.print("BAJO      ALTO ");
  delay(500);
}

void estadoCero()
{
  digitalWrite(pinRelayA, HIGH);
  digitalWrite(pinRelayB, HIGH);
}

void estadoMovimiento(int velmax)
{ //NIVEL BAJO porque es mayor de 30 grados
  if (velmax == 17)
  {
    lcd.setCursor(5, 0);
    lcd.print("UPIIZ  IPN");
    lcd.setCursor(0, 1);
    lcd.print("VELOCIDAD MAX ACTUAL");
    lcd.setCursor(6, 2);
    lcd.print("1 KM/H");
    lcd.setCursor(5, 3);
    lcd.print("PRECAUCION");
    delay(500);
  }
  if (velmax == 52)
  {
    lcd.setCursor(5, 0);
    lcd.print("UPIIZ  IPN");
    lcd.setCursor(0, 1);
    lcd.print("VELOCIDAD MAX ACTUAL");
    lcd.setCursor(6, 2);
    lcd.print("3.0 KM/H");
    delay(500);
  }
  if (velmax == 87)
  {
    if (velmax == 52)
    {
      lcd.setCursor(5, 0);
      lcd.print("UPIIZ  IPN");
      lcd.setCursor(0, 1);
      lcd.print("VELOCIDAD MAX ACTUAL");
      lcd.setCursor(6, 2);
      lcd.print("5.0 KM/H");
      delay(500);
    }
  }

  /*
    if (digitalRead(btnvelalto) == HIGH && digitalRead(btnvelbajo) == LOW)
    {
    varmov = 1;
    lcd.setCursor(5, 0);
    lcd.print("UPIIZ  IPN");
    lcd.setCursor(0, 1);
    lcd.print("VELOCIDAD MAX ACTUAL");
    lcd.setCursor(6, 2);
    lcd.print("3.0 KM/H");
    delay(500);
    }
    else if (digitalRead(btnvelalto) == LOW && digitalRead(btnvelbajo) == HIGH)
    {
    varmov = 2;
    lcd.setCursor(5, 0);
    lcd.print("UPIIZ  IPN");
    lcd.setCursor(0, 1);
    lcd.print("VELOCIDAD MAX ACTUAL");
    lcd.setCursor(6, 2);
    lcd.print("1.0 KM/H");
    delay(500);
    }
  */


  /*void accionMovimiento2() { // ------------------------------------------------NIVEL BAJO PORQUE EL ANGULO ES MAYOR A 30----------------------------------------------
  	int xAxis = analogRead(A0); // Read Joysticks X-axis
  	int yAxis = analogRead(A1); // Read Joysticks Y-axis

  	if (yAxis < 530) {
  		delay(100);
  		// Set Motor left backward
  		digitalWrite(in1, LOW); //
  		analogWrite(in2, motorSpeedA); //analogWrite(enA, motorSpeedA)
  		// Set Motor right backward
  		digitalWrite(in3, LOW); //
  		analogWrite(in4, motorSpeedB); //
  		//-------------------------------------------------------------REVERSA RANGO 1-------------------------------------------------------------
  		if (yAxis < 530 && yAxis > 477 ) {
  			motorSpeedA = map(yAxis, 530, 477, 0, 10);
  			motorSpeedB = map(yAxis, 530, 477, 0, 10);
  		}//-------------------------------------------------------------REVERSA RANGO 2-------------------------------------------------------------
  		else if (yAxis < 477 && yAxis > 424) {
  			motorSpeedA = map(yAxis, 477, 424, 10, 20);
  			motorSpeedB = map(yAxis, 477, 424, 10, 20);
  		} //-----------------------------------------------------------REVERSA RANGO 3----------------------------------------------------------------
  		else if (yAxis < 424 && yAxis > 371) {
  			motorSpeedA = map(yAxis, 424, 371, 20, 30);
  			motorSpeedB = map(yAxis, 424, 371, 20, 30);
  		}
  		//-----------------------------------------------------------REVERSA RANGO 4----------------------------------------------------------------
  		else if (yAxis < 371 && yAxis > 318) {
  			motorSpeedA = map(yAxis, 371, 318, 30, 40);
  			motorSpeedB = map(yAxis, 371, 318, 30, 40);
  		}

  		//-------------------------------------------------------------REVERSA RANGO 5-------------------------------------------------------------
  		else if (yAxis < 318 && yAxis > 265) {
  			motorSpeedA = map(yAxis, 318, 265, 40, 50);
  			motorSpeedB = map(yAxis, 318, 265, 40, 50);
  		}
  		//-------------------------------------------------------------REVERSA RANGO 6-------------------------------------------------------------
  		else if (yAxis < 265 && yAxis > 212) {
  			motorSpeedA = map(yAxis, 265, 212, 50, 60);
  			motorSpeedB = map(yAxis, 265, 212, 50, 60);
  		}
  		//-------------------------------------------------------------REVERSA RANGO 7-------------------------------------------------------------
  		else if (yAxis < 212 && yAxis > 159) {
  			motorSpeedA = map(yAxis, 212, 159, 60, 70);
  			motorSpeedB = map(yAxis, 212, 159, 60, 70);
  		}
  		//-------------------------------------------------------------REVERSA RANGO 8-------------------------------------------------------------
  		else if (yAxis < 159 && yAxis > 106) {
  			motorSpeedA = map(yAxis, 159, 106, 70, 80);
  			motorSpeedB = map(yAxis, 159, 106, 70, 80);
  		}
  		//-------------------------------------------------------------REVERSA RANGO 9-------------------------------------------------------------
  		else if (yAxis < 106 && yAxis > 53) {
  			motorSpeedA = map(yAxis, 106, 53, 80, 90);
  			motorSpeedB = map(yAxis, 106, 53, 80, 90);
  		}
  		//-------------------------------------------------------------REVERSA RANGO 10-------------------------------------------------------------
  		else if (yAxis < 53) {
  			motorSpeedA = map(yAxis, 53, 0, 90, 100);
  			motorSpeedB = map(yAxis, 53, 0, 90, 100);
  		}
  	}

  	//-------------------------------------------------------------SI EL JOYSTICK VA PARA DELANTE-------------------------------------------------------------
  	else if (yAxis > 494) {
  		delay(100);
  		// Set Motor A forward
  		analogWrite(in1, motorSpeedA); //
  		digitalWrite(in2, LOW); //
  		// Set Motor B forward
  		analogWrite(in3, motorSpeedB); //
  		digitalWrite(in4, LOW); //

  		//-------------------------------------------------------------DELANTE RANGO 1-------------------------------------------------------------
  		if (yAxis > 494 && yAxis < 547 ) {
  			motorSpeedA = map(yAxis, 494, 547, 0, 10);
  			motorSpeedB = map(yAxis, 494, 547, 0, 10);
  		}//-------------------------------------------------------------DELANTE RANGO 2-------------------------------------------------------------
  		else if (yAxis > 547 && yAxis < 600) {
  			motorSpeedA = map(yAxis, 547, 600, 10, 20);
  			motorSpeedB = map(yAxis, 547, 600, 10, 20);
  		} //-----------------------------------------------------------DELANTE RANGO 3----------------------------------------------------------------
  		else if (yAxis > 600 && yAxis < 653) {
  			motorSpeedA = map(yAxis, 600, 653, 20, 30);
  			motorSpeedB = map(yAxis, 600, 653, 20, 30);
  		}
  		//-----------------------------------------------------------DELANTE RANGO 4----------------------------------------------------------------
  		else if (yAxis > 653 && yAxis < 706) {
  			motorSpeedA = map(yAxis, 653, 706, 30, 40);
  			motorSpeedB = map(yAxis, 653, 706, 30, 40);
  		}

  		//-----------------------------------------------------------DELANTE RANGO 5----------------------------------------------------------------
  		else if (yAxis > 706 && yAxis < 759) {
  			motorSpeedA = map(yAxis, 706, 759, 40, 50);
  			motorSpeedB = map(yAxis, 706, 759, 40, 50);
  		}
  		//-----------------------------------------------------------DELANTE RANGO 6----------------------------------------------------------------
  		else if (yAxis > 759 && yAxis < 812) {
  			motorSpeedA = map(yAxis, 759, 812, 50, 60);
  			motorSpeedB = map(yAxis, 759, 812, 50, 60);
  		}
  		//-----------------------------------------------------------DELANTE RANGO 7----------------------------------------------------------------
  		else if (yAxis > 812 && yAxis < 865) {
  			motorSpeedA = map(yAxis, 812, 865, 60, 70);
  			motorSpeedB = map(yAxis, 812, 865, 60, 70);
  		}
  		//-----------------------------------------------------------DELANTE RANGO 8----------------------------------------------------------------
  		else if (yAxis > 865 && yAxis < 918) {
  			motorSpeedA = map(yAxis, 865, 918, 70, 80);
  			motorSpeedB = map(yAxis, 865, 918, 70, 80);
  		}
  		//-----------------------------------------------------------DELANTE RANGO 9----------------------------------------------------------------
  		else if (yAxis > 918 && yAxis < 971) {
  			motorSpeedA = map(yAxis, 918, 971, 80, 90);
  			motorSpeedB = map(yAxis, 918, 971, 80, 90);
  		}
  		//-----------------------------------------------------------DELANTE RANGO 10----------------------------------------------------------------

  		else if (yAxis > 971) {
  			motorSpeedA = map(yAxis, 971, 1024, 90, 100);
  			motorSpeedB = map(yAxis, 971, 1024, 90, 100);
  		}
  	}
  	//-----------------------------------------------------------DETENER----------------------------------------------------------------------------------------
  	else {
  		digitalWrite(motorSpeedA, LOW);
  		digitalWrite(motorSpeedB, LOW);
  	}


  	// -------------------------------------------------------------------- IZQUIERDA Y DERECHA ---------------------------------------------------------------------
  	if (xAxis < 494) { //
  		// Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
  		int xMapped = map(xAxis, 494, 0, 0, 100);
  		// Move to left - decrease left motor speed, increase right motor speed
  		motorSpeedA = motorSpeedA - xMapped; // - left
  		motorSpeedB = motorSpeedB + xMapped; // + right
  		// Confine the range from 0 to 255
  		if (motorSpeedA < 0) {
  			motorSpeedA = 0;
  		}
  		if (motorSpeedB > 100) {
  			motorSpeedB = 100;
  		}
  	}
  	if (xAxis > 530) { //
  		// Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
  		int xMapped = map(xAxis, 530, 1023, 0, 100);
  		// Move right - decrease right motor speed, increase left motor speed
  		motorSpeedA = motorSpeedA + xMapped; // + left
  		motorSpeedB = motorSpeedB - xMapped; // - right
  		// Confine the range from 0 to 255
  		if (motorSpeedA > 100) {
  			motorSpeedA = 100;
  		}
  		if (motorSpeedB < 0) {
  			motorSpeedB = 0;
  		}
  	}
  	if (motorSpeedA < 10) {
  		motorSpeedA = 0;
  	}
  	if (motorSpeedB < 10) {
  		motorSpeedB = 0;
  	}
    }

  */

  //------------------------------------------------------------------------ESTADOMOVIMIENTO1-------------------------------------------------------------------------
  /*void estadoMovimiento1() {
  	if (digitalRead(btnvelalto) == HIGH && digitalRead(btnvelbajo) == LOW) {
  		varmov = 1;
  		lcd.setCursor(5, 0);
  		lcd.print("UPIIZ  IPN");
  		lcd.setCursor(0, 1);
  		lcd.print("VELOCIDAD MAX ACTUAL");
  		lcd.setCursor(6, 2);
  		lcd.print("3.0 KM/H");
  		delay(500);
  	}
  	else if (digitalRead(btnvelalto) == LOW && digitalRead(btnvelbajo) == HIGH) {
  		varmov = 2;
  		lcd.setCursor(5, 0);
  		lcd.print("UPIIZ  IPN");
  		lcd.setCursor(0, 1);
  		lcd.print("VELOCIDAD MAX ACTUAL");
  		lcd.setCursor(6, 2);
  		lcd.print("1.0 KM/H");
  		delay(500);
  	}
    }
  */
  //----------------------------------------------------------------------ACCION MOVIMIENTO 1---------------------------------------------------------------------------
  /*void accionMovimiento1() { //NIVEL ALTO O BAJO
  	int xAxis = analogRead(A0); // Read Joysticks X-axis
  	int yAxis = analogRead(A1); // Read Joysticks Y-axis
  	switch (varmov) {
  	case 1:              //--------------------------------------------------------NIVEL ALTO------------------------------------------------------------

  		if (yAxis < 530) {
  			delay(100);
  			// Set Motor left backward
  			digitalWrite(in1, LOW); //
  			analogWrite(in2, motorSpeedA); //analogWrite(enA, motorSpeedA)
  			// Set Motor right backward
  			digitalWrite(in3, LOW); //
  			analogWrite(in4, motorSpeedB); //
  			//-------------------------------------------------------------REVERSA RANGO 1-------------------------------------------------------------
  			if (yAxis < 530 && yAxis > 477 ) {
  				motorSpeedA = map(yAxis, 530, 477, 0, 18);
  				motorSpeedB = map(yAxis, 530, 477, 0, 18);
  			}//-------------------------------------------------------------REVERSA RANGO 2-------------------------------------------------------------
  			else if (yAxis < 477 && yAxis > 424) {
  				motorSpeedA = map(yAxis, 477, 424, 18, 36);
  				motorSpeedB = map(yAxis, 477, 424, 18, 36);
  			} //-----------------------------------------------------------REVERSA RANGO 3----------------------------------------------------------------
  			else if (yAxis < 424 && yAxis > 371) {
  				motorSpeedA = map(yAxis, 424, 371, 36, 54);
  				motorSpeedB = map(yAxis, 424, 371, 36, 54);
  			}
  			//-----------------------------------------------------------REVERSA RANGO 4----------------------------------------------------------------
  			else if (yAxis < 371 && yAxis > 318) {
  				motorSpeedA = map(yAxis, 371, 318, 54, 72);
  				motorSpeedB = map(yAxis, 371, 318, 54, 72);
  			}

  			//-------------------------------------------------------------REVERSA RANGO 5-------------------------------------------------------------
  			else if (yAxis < 318 && yAxis > 265) {
  				motorSpeedA = map(yAxis, 318, 265, 72, 90);
  				motorSpeedB = map(yAxis, 318, 265, 72, 90);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 6-------------------------------------------------------------
  			else if (yAxis < 265 && yAxis > 212) {
  				motorSpeedA = map(yAxis, 265, 212, 90, 108);
  				motorSpeedB = map(yAxis, 265, 212, 90, 108);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 7-------------------------------------------------------------
  			else if (yAxis < 212 && yAxis > 159) {
  				motorSpeedA = map(yAxis, 212, 159, 108, 126);
  				motorSpeedB = map(yAxis, 212, 159, 108, 126);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 8-------------------------------------------------------------
  			else if (yAxis < 159 && yAxis > 106) {
  				motorSpeedA = map(yAxis, 159, 106, 126, 144);
  				motorSpeedB = map(yAxis, 159, 106, 126, 144);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 9-------------------------------------------------------------
  			else if (yAxis < 106 && yAxis > 53) {
  				motorSpeedA = map(yAxis, 106, 53, 144, 162);
  				motorSpeedB = map(yAxis, 106, 53, 144, 162);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 10-------------------------------------------------------------
  			else if (yAxis < 53) {
  				motorSpeedA = map(yAxis, 53, 0, 162, 180);
  				motorSpeedB = map(yAxis, 53, 0, 162, 180);
  			}
  		}

  		//-------------------------------------------------------------SI EL JOYSTICK VA PARA DELANTE-------------------------------------------------------------
  		else if (yAxis > 494) {
  			delay(100);
  			// Set Motor A forward
  			analogWrite(in1, motorSpeedA); //
  			digitalWrite(in2, LOW); //
  			// Set Motor B forward
  			analogWrite(in3, motorSpeedB); //
  			digitalWrite(in4, LOW); //

  			//-------------------------------------------------------------DELANTE RANGO 1-------------------------------------------------------------
  			if (yAxis > 494 && yAxis < 547 ) {
  				motorSpeedA = map(yAxis, 494, 547, 0, 18);
  				motorSpeedB = map(yAxis, 494, 547, 0, 18);
  			}//-------------------------------------------------------------DELANTE RANGO 2-------------------------------------------------------------
  			else if (yAxis > 547 && yAxis < 600) {
  				motorSpeedA = map(yAxis, 547, 600, 18, 36);
  				motorSpeedB = map(yAxis, 547, 600, 18, 36);
  			} //-----------------------------------------------------------DELANTE RANGO 3----------------------------------------------------------------
  			else if (yAxis > 600 && yAxis < 653) {
  				motorSpeedA = map(yAxis, 600, 653, 36, 54);
  				motorSpeedB = map(yAxis, 600, 653, 36, 54);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 4----------------------------------------------------------------
  			else if (yAxis > 653 && yAxis < 706) {
  				motorSpeedA = map(yAxis, 653, 706, 54, 72);
  				motorSpeedB = map(yAxis, 653, 706, 54, 72);
  			}

  			//-----------------------------------------------------------DELANTE RANGO 5----------------------------------------------------------------
  			else if (yAxis > 706 && yAxis < 759) {
  				motorSpeedA = map(yAxis, 706, 759, 72, 90);
  				motorSpeedB = map(yAxis, 706, 759, 72, 90);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 6----------------------------------------------------------------
  			else if (yAxis > 759 && yAxis < 812) {
  				motorSpeedA = map(yAxis, 759, 812, 90, 108);
  				motorSpeedB = map(yAxis, 759, 812, 90, 108);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 7----------------------------------------------------------------
  			else if (yAxis > 812 && yAxis < 865) {
  				motorSpeedA = map(yAxis, 812, 865, 108, 126);
  				motorSpeedB = map(yAxis, 812, 865, 108, 126);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 8----------------------------------------------------------------
  			else if (yAxis > 865 && yAxis < 918) {
  				motorSpeedA = map(yAxis, 865, 918, 126, 144);
  				motorSpeedB = map(yAxis, 865, 918, 126, 144);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 9----------------------------------------------------------------
  			else if (yAxis > 918 && yAxis < 971) {
  				motorSpeedA = map(yAxis, 918, 971, 144, 162);
  				motorSpeedB = map(yAxis, 918, 971, 144, 162);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 10----------------------------------------------------------------

  			else if (yAxis > 971) {
  				motorSpeedA = map(yAxis, 971, 1024, 162, 180);
  				motorSpeedB = map(yAxis, 971, 1024, 162, 180);
  			}
  		}
  		//-----------------------------------------------------------DETENER----------------------------------------------------------------------------------------
  		else {
  			digitalWrite(motorSpeedA, LOW);
  			digitalWrite(motorSpeedB, LOW);
  		}


  		// -------------------------------------------------------------------- IZQUIERDA Y DERECHA ---------------------------------------------------------------------
  		if (xAxis < 494) { //
  			// Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
  			int xMapped = map(xAxis, 494, 0, 0, 180);
  			// Move to left - decrease left motor speed, increase right motor speed
  			motorSpeedA = motorSpeedA - xMapped; // - left
  			motorSpeedB = motorSpeedB + xMapped; // + right
  			// Confine the range from 0 to 255
  			if (motorSpeedA < 0) {
  				motorSpeedA = 0;
  			}
  			if (motorSpeedB > 180) {
  				motorSpeedB = 180;
  			}
  		}
  		if (xAxis > 530) { //
  			// Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
  			int xMapped = map(xAxis, 530, 1023, 0, 180);
  			// Move right - decrease right motor speed, increase left motor speed
  			motorSpeedA = motorSpeedA + xMapped; // + left
  			motorSpeedB = motorSpeedB - xMapped; // - right
  			// Confine the range from 0 to 255
  			if (motorSpeedA > 180) {
  				motorSpeedA = 180;
  			}
  			if (motorSpeedB < 0) {
  				motorSpeedB = 0;
  			}
  		}
  		if (motorSpeedA < 18) {
  			motorSpeedA = 0;
  		}
  		if (motorSpeedB < 18) {
  			motorSpeedB = 0;
  		}
  		break;
  	case 2: //--------------------------------------------------------------------------------NIVEL BAJO-------------------------------------------------------------
  		if (yAxis < 530) {
  			delay(100);
  			// Set Motor left backward
  			digitalWrite(in1, LOW); //
  			analogWrite(in2, motorSpeedA); //analogWrite(enA, motorSpeedA)
  			// Set Motor right backward
  			digitalWrite(in3, LOW); //
  			analogWrite(in4, motorSpeedB); //
  			//-------------------------------------------------------------REVERSA RANGO 1-------------------------------------------------------------
  			if (yAxis < 530 && yAxis > 477 ) {
  				motorSpeedA = map(yAxis, 530, 477, 0, 10);
  				motorSpeedB = map(yAxis, 530, 477, 0, 10);
  			}//-------------------------------------------------------------REVERSA RANGO 2-------------------------------------------------------------
  			else if (yAxis < 477 && yAxis > 424) {
  				motorSpeedA = map(yAxis, 477, 424, 10, 20);
  				motorSpeedB = map(yAxis, 477, 424, 10, 20);
  			} //-----------------------------------------------------------REVERSA RANGO 3----------------------------------------------------------------
  			else if (yAxis < 424 && yAxis > 371) {
  				motorSpeedA = map(yAxis, 424, 371, 20, 30);
  				motorSpeedB = map(yAxis, 424, 371, 20, 30);
  			}
  			//-----------------------------------------------------------REVERSA RANGO 4----------------------------------------------------------------
  			else if (yAxis < 371 && yAxis > 318) {
  				motorSpeedA = map(yAxis, 371, 318, 30, 40);
  				motorSpeedB = map(yAxis, 371, 318, 30, 40);
  			}

  			//-------------------------------------------------------------REVERSA RANGO 5-------------------------------------------------------------
  			else if (yAxis < 318 && yAxis > 265) {
  				motorSpeedA = map(yAxis, 318, 265, 40, 50);
  				motorSpeedB = map(yAxis, 318, 265, 40, 50);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 6-------------------------------------------------------------
  			else if (yAxis < 265 && yAxis > 212) {
  				motorSpeedA = map(yAxis, 265, 212, 50, 60);
  				motorSpeedB = map(yAxis, 265, 212, 50, 60);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 7-------------------------------------------------------------
  			else if (yAxis < 212 && yAxis > 159) {
  				motorSpeedA = map(yAxis, 212, 159, 60, 70);
  				motorSpeedB = map(yAxis, 212, 159, 60, 70);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 8-------------------------------------------------------------
  			else if (yAxis < 159 && yAxis > 106) {
  				motorSpeedA = map(yAxis, 159, 106, 70, 80);
  				motorSpeedB = map(yAxis, 159, 106, 70, 80);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 9-------------------------------------------------------------
  			else if (yAxis < 106 && yAxis > 53) {
  				motorSpeedA = map(yAxis, 106, 53, 80, 90);
  				motorSpeedB = map(yAxis, 106, 53, 80, 90);
  			}
  			//-------------------------------------------------------------REVERSA RANGO 10-------------------------------------------------------------
  			else if (yAxis < 53) {
  				motorSpeedA = map(yAxis, 53, 0, 90, 100);
  				motorSpeedB = map(yAxis, 53, 0, 90, 100);
  			}
  		}

  		//-------------------------------------------------------------SI EL JOYSTICK VA PARA DELANTE-------------------------------------------------------------
  		else if (yAxis > 494) {
  			delay(100);
  			// Set Motor A forward
  			analogWrite(in1, motorSpeedA); //
  			digitalWrite(in2, LOW); //
  			// Set Motor B forward
  			analogWrite(in3, motorSpeedB); //
  			digitalWrite(in4, LOW); //

  			//-------------------------------------------------------------DELANTE RANGO 1-------------------------------------------------------------
  			if (yAxis > 494 && yAxis < 547 ) {
  				motorSpeedA = map(yAxis, 494, 547, 0, 10);
  				motorSpeedB = map(yAxis, 494, 547, 0, 10);
  			}//-------------------------------------------------------------DELANTE RANGO 2-------------------------------------------------------------
  			else if (yAxis > 547 && yAxis < 600) {
  				motorSpeedA = map(yAxis, 547, 600, 10, 20);
  				motorSpeedB = map(yAxis, 547, 600, 10, 20);
  			} //-----------------------------------------------------------DELANTE RANGO 3----------------------------------------------------------------
  			else if (yAxis > 600 && yAxis < 653) {
  				motorSpeedA = map(yAxis, 600, 653, 20, 30);
  				motorSpeedB = map(yAxis, 600, 653, 20, 30);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 4----------------------------------------------------------------
  			else if (yAxis > 653 && yAxis < 706) {
  				motorSpeedA = map(yAxis, 653, 706, 30, 40);
  				motorSpeedB = map(yAxis, 653, 706, 30, 40);
  			}

  			//-----------------------------------------------------------DELANTE RANGO 5----------------------------------------------------------------
  			else if (yAxis > 706 && yAxis < 759) {
  				motorSpeedA = map(yAxis, 706, 759, 40, 50);
  				motorSpeedB = map(yAxis, 706, 759, 40, 50);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 6----------------------------------------------------------------
  			else if (yAxis > 759 && yAxis < 812) {
  				motorSpeedA = map(yAxis, 759, 812, 50, 60);
  				motorSpeedB = map(yAxis, 759, 812, 50, 60);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 7----------------------------------------------------------------
  			else if (yAxis > 812 && yAxis < 865) {
  				motorSpeedA = map(yAxis, 812, 865, 60, 70);
  				motorSpeedB = map(yAxis, 812, 865, 60, 70);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 8----------------------------------------------------------------
  			else if (yAxis > 865 && yAxis < 918) {
  				motorSpeedA = map(yAxis, 865, 918, 70, 80);
  				motorSpeedB = map(yAxis, 865, 918, 70, 80);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 9----------------------------------------------------------------
  			else if (yAxis > 918 && yAxis < 971) {
  				motorSpeedA = map(yAxis, 918, 971, 80, 90);
  				motorSpeedB = map(yAxis, 918, 971, 80, 90);
  			}
  			//-----------------------------------------------------------DELANTE RANGO 10----------------------------------------------------------------

  			else if (yAxis > 971) {
  				motorSpeedA = map(yAxis, 971, 1024, 90, 100);
  				motorSpeedB = map(yAxis, 971, 1024, 90, 100);
  			}
  		}
  		//-----------------------------------------------------------DETENER----------------------------------------------------------------------------------------
  		else {
  			digitalWrite(motorSpeedA, LOW);
  			digitalWrite(motorSpeedB, LOW);
  		}


  		// -------------------------------------------------------------------- IZQUIERDA Y DERECHA ---------------------------------------------------------------------
  		if (xAxis < 494) { //
  			// Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
  			int xMapped = map(xAxis, 494, 0, 0, 100);
  			// Move to left - decrease left motor speed, increase right motor speed
  			motorSpeedA = motorSpeedA - xMapped; // - left
  			motorSpeedB = motorSpeedB + xMapped; // + right
  			// Confine the range from 0 to 255
  			if (motorSpeedA < 0) {
  				motorSpeedA = 0;
  			}
  			if (motorSpeedB > 100) {
  				motorSpeedB = 100;
  			}
  		}
  		if (xAxis > 530) { //
  			// Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
  			int xMapped = map(xAxis, 530, 1023, 0, 100);
  			// Move right - decrease right motor speed, increase left motor speed
  			motorSpeedA = motorSpeedA + xMapped; // + left
  			motorSpeedB = motorSpeedB - xMapped; // - right
  			// Confine the range from 0 to 255
  			if (motorSpeedA > 100) {
  				motorSpeedA = 100;
  			}
  			if (motorSpeedB < 0) {
  				motorSpeedB = 0;
  			}
  		}
  		if (motorSpeedA < 10) {
  			motorSpeedA = 0;
  		}
  		if (motorSpeedB < 10) {
  			motorSpeedB = 0;
  		}
  		break;
  	}
    }
  */

  //--------------------------------------------------------------------ESTADO BIPEDESTACION---------------------------------------------------------------------

