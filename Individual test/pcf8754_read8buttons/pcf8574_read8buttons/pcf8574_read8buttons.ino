#include <Adafruit_PCF8574.h>
#include <Wire.h>


/* Example for 8 input buttons that are connected from the GPIO expander pins to ground.
 * Note the buttons must be connected with the other side of the switch to GROUND. There is
 * a built in pull-up 'resistor' on each input, but no pull-down resistor capability.
 */
TwoWire myWire(0);

Adafruit_PCF8574 pcf;

bool is_pressed = false;

void scanI2C() {
    Serial.println("Scanning I2C bus...");
    byte count = 0;

    for (byte address = 1; address < 127; address++) {
        myWire.beginTransmission(address);
        if (myWire.endTransmission() == 0) {
            Serial.print("I2C device found at 0x");
            Serial.println(address, HEX);
            count++;
        }
        delay(2);
    }

    if (count == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.println("I2C scan complete.");
    }
}


void setup() {
  Serial.begin(9600);
  delay(1000); // Let Serial stabilize
  Serial.println("Adafruit PCF8574 button read test");
    myWire.begin(22, 23);  // SDA, SCL
    scanI2C();

  pinMode(35, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(35), blink, RISING);


  if (!pcf.begin(0x20, &myWire)) {
    Serial.println("Couldn't find PCF8574");
    while (1) {
      Serial.println("PCF8574 not found!");
      delay(1000);
    }
  }

  // Configure each pin as input with pull-up enabled

  for (uint8_t p = 0; p < 8; p++) {
    pcf.pinMode(p, INPUT);
    pcf.digitalWrite(p, HIGH); // enable pull-up
  }
}

void blink() {
  is_pressed=true;
}

void loop() {
  byte reponseI2C;

  // Envoi d'une demande de lecture de données, auprès du PCF8574
  myWire.requestFrom(0x20, 1);                    // Le "1" signifie qu'on envisage de lire 1 seul octet en retour

  // Récupération de l'octet en question
  if(myWire.available()) {
    reponseI2C = myWire.read();                                     // Lecture de l'octet qu'on attendait en retour
    Serial.println(reponseI2C);                     // Affichage sur le moniteur série de l'IDE Arduino
                                                                            // Les "1" diront que l'entrée est à l'état haut (+Vcc, par exemple)
                                                                            // Les "0" diront que l'entrée est à l'état bas (0 volt, par exemple)
  } else {
    Serial.println(F("[ERREUR] Impossible de récupérer la valeur I2C renvoyée par le PCF8574"));
	  Serial.println(F("Arrêt du programme."));
    while(1);
  }

  // … et on reboucle à l'infini (avec une petite pause au passage, pour ne pas surcharger le moniteur série)
  delay(1000);
  // }
  
}
